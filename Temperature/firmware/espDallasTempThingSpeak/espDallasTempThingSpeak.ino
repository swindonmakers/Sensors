/*
 * Wiring:
 * 
 *  VCC - 3.3V
 *  GND
 *  
 *  RST   - 10k - VCC
 *  CM_EN - 10k - VCC
 *  GPIO0 - 10k - VCC
 *  
 *  GPIO2 - TempSensor data (center pin) - 4.7K resistor to VCC
 * 
 *  RX  (1)  - Used for status LED when DEBUG = false
 *  TX  (3)
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include "ThingSpeak.h"

extern "C" {
#include "user_interface.h"

extern struct rst_info resetInfo;
}

#define DEBUG false

#define PIN_LED LED_BUILTIN
#define ONE_WIRE_BUS 2

#define READING_INTERVAL 10 * 60 // seconds

static ESP8266WebServer server(80);
static WiFiClient client;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float t = NAN;

unsigned long channelNumber = -1; // Configure this
const char * writeApiKey = "MY_WRITE_API_KEY"; // Configure this

void debugWrite(String msg)
{
  if (DEBUG)
    Serial.println(msg);
}

void ledOn()
{
  if (!DEBUG)
    digitalWrite(PIN_LED, LOW);
}

void ledOff()
{
  if (!DEBUG)
    digitalWrite(PIN_LED, HIGH);
}

void handleRoot() 
{
  String page = "Temp:";
  page += t;
  page += "\r\n";
  page += millis();
  
  server.send(200, "text/plain", page);
}

void sleep()
{
  debugWrite("Sleeping");
  ESP.deepSleep(READING_INTERVAL * 1000 * 1000, WAKE_RF_DEFAULT);

  delay(50);
}

void setup() 
{
  if (DEBUG) {
    Serial.begin(115200);
    Serial.println("espDallasTemp start");
    Serial.print("resetInfo.reason = ");
    Serial.println(resetInfo.reason);
    Serial.println(ESP.getResetInfo());
  } else {
    pinMode(PIN_LED, OUTPUT);
    ledOn();
  }

  debugWrite("Connecting wifi..");
  WiFi.begin("myap", "password");
  
  server.on("/", handleRoot);
  
  server.begin();
  sensors.begin();

  ThingSpeak.begin(client);

  ledOff();
}

bool sendData()
{
  debugWrite("Have sensor data, connected to wifi, posting data");
  ledOn();

  ThingSpeak.setField(1, t);
  ThingSpeak.setField(2, (long)millis());
  int stat = ThingSpeak.writeFields(channelNumber, writeApiKey);

  ledOff();
  if (stat == 200)
    return true;

  debugWrite("Send failed " + String(stat));
  return false;
}

void loop() 
{
  server.handleClient();

  if (isnan(t)) {
    sensors.requestTemperatures();
    t = sensors.getTempCByIndex(0);

    if (t == -127.0 || t == 85.0) // invalid
      t = NAN;

    debugWrite("Have temp: " + String(t));
  }

  if (WiFi.status() == WL_CONNECTED && !isnan(t)) {
    if (sendData()) {
      sleep();
    }
  }

  if (millis() > 10000) {
    debugWrite("Failure after 10s");
    if (WiFi.status() != WL_CONNECTED) debugWrite("Wifi not connected");
    if (isnan(t)) debugWrite("No temperature reading");

    sleep();
  }
}
