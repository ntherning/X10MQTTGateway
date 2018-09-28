/* 
 * X10 to MQTT gateway.
 * 
 * This piece of software emulates a PL513/PSC05/TW523 X10 transmitter.
 * Any received X10 commands will be retransmitted on an MQTT bus.
 * 
 * Arduino Interface to the X10 transmitting device:
 * - RJ11 pin 1 (BLK) -> Pin 2 (Interrupt 0) = Zero Crossing
 * - RJ11 pin 2 (RED) -> GND
 * - RJ11 pin 3 (GRN) -> Pin 3 = Arduino transmit (not used)
 * - RJ11 pin 4 (YEL) -> Pin 5 = Arduino receive
 * 
 * Arduino pin 3 is used to generate a 60 Hz square wave.
 * This should be connected to Arduino pin 2 and also RJ11 pin 1.
 * 
 * The Arduino must be connected to an ESP-01 WIFI module flashed with esp-link.
 */

#include <x10.h>                       // X10 lib is used for transmitting X10
#include <x10constants.h>              // X10 Lib constants

#include <ELClient.h>
#include <ELClientMqtt.h>


#define ZCROSS_PIN     2               // BLK pin 1 of RJ11 connector
#define TRANS_PIN      4               // GRN pin 3 of RJ11 connector (not used)
#define RCVE_PIN       5               // YEL pin 4 of RJ11 connector
#define LED_PIN        13              // for testing 

x10 SX10;// The x10 library instance:

// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
ELClient esp(&Serial, &Serial);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

static bool connected = false;

// Callback made from esp-link to notify of wifi status changes
void wifiCb(void* response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);

    if (status == STATION_GOT_IP) {
      Serial.println("WIFI CONNECTED");
    } else {
      Serial.print("WIFI NOT READY: ");
      Serial.println(status);
    }
  }
}

// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial.println("MQTT connected!");
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial.println("MQTT disconnected");
  connected = false;
}

// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;

  String receivedTopic = res->popString();
  String receivedData = res->popString();
  Serial.print("Received: topic=");
  Serial.print(receivedTopic);
  Serial.print(", data=");
  Serial.println(receivedData);
}

void mqttPublished(void* response) {
  Serial.println("MQTT published");
}

void initX10() {
  Serial.println("Initializing X10 interface...");
  // Generate a ~60 Hz PWM square wave using timer2 (PIN 3)
  // Adapted from https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
  // Output A frequency: 16 MHz / 1024 / 126 / 2 / 2 = 31 Hz
  // Output A duty cycle: 50%
  // Output B frequency: 16 MHz / 1024 / 156 / 2 = 62 Hz
  // Output B duty cycle: 63 / 126 = 50%
  pinMode(3, OUTPUT);  // Output B
  pinMode(11, OUTPUT); // Output A
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS22)| _BV(CS21)| _BV(CS20);
  OCR2A = 126;
  OCR2B = 63;

  SX10.init(ZCROSS_PIN,TRANS_PIN,RCVE_PIN,LED_PIN);
  Serial.println(SX10.version());
  delay(500);
  Serial.println("X10 interface initialization done!");
  Serial.println();
}

void initELClient() {
  Serial.println("Initializing EL-Client...");
  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
  while (!esp.Sync()) { // sync up with esp-link, blocks for up to 2 seconds
    Serial.println("EL-Client sync failed!");
  }
  Serial.println("EL-Client initialization done!");
  Serial.println();
}

void initELMQTT() {
  Serial.println("Initializing EL-MQTT...");
  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();
  Serial.println("EL-MQTT initialization done!");
  Serial.println();  
 
}

void setup() {
  Serial.begin(115200);

  initX10();
  initELClient();
  initELMQTT();
  
  Serial.println("X10 to MQTT Gateway started");
}

void loop(){
  char topic[64];
  if (SX10.received()) {
    SX10.debug();

    String topic = "x10/";
    topic += (char) SX10.houseCode();
    topic += '/';
    topic += SX10.unitCode();
    char *payload = SX10.cmndCode() == ON ? "ON" : "OFF";
    Serial.print("Publishing payload '");
    Serial.print(payload);
    Serial.print("' to MQTT topic '");
    Serial.print(topic);
    Serial.println("'");
    mqtt.publish(topic.c_str(), payload);

    SX10.reset();
  }
} 

