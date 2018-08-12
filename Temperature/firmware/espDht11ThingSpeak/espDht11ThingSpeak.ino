/*
 * Wiring:
 * 
 *  VCC - 3.3V
 *  GND
 *  
 *  DHT module out pin -> D5
 *  
 *  Jumper D0 -> Reset Pin
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "DHT.h"
#include "ThingSpeak.h"

#define PIN_LED D6

#define DHTPIN D5
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

#define READING_INTERVAL 10*60 // seconds

static ESP8266WebServer server(80);
static WiFiClient client;

DHT dht(DHTPIN, DHTTYPE);
float t = NAN;
float h = NAN;

unsigned long channelNumber = 0; // Configure this
const char * writeApiKey = ""; // Configure this

void debugWrite(String msg)
{
    Serial.println(msg);
}

void ledOn()
{
    digitalWrite(PIN_LED, LOW);
}

void ledOff()
{
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

  delay(500);
}

void setup() 
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println("espDht11ThingSpeak start");
  Serial.println(ESP.getResetInfo());
  pinMode(PIN_LED, OUTPUT);
  ledOn();

  debugWrite("Connecting wifi..");
  WiFi.begin("myap", "password");

  server.on("/", handleRoot);
  
  server.begin();
  dht.begin();

  ThingSpeak.begin(client);

  ledOff();
}

bool sendData()
{
  debugWrite("Have sensor data, connected to wifi, posting data");
  ledOn();

  ThingSpeak.setField(1, t);
  ThingSpeak.setField(2, (long)millis());
  ThingSpeak.setField(3, h);
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
    t = dht.readTemperature();
    debugWrite("Have temp: " + String(t));
  }

  if (isnan(h)) {
    h = dht.readHumidity();
    debugWrite("Have humi: " + String(h));
  }

  if (WiFi.status() == WL_CONNECTED && !isnan(t) && !isnan(h)) {
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
