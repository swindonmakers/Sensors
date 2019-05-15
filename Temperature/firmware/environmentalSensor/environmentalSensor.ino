/*
  Wiring:

    ## BME280
    GND -> GND
    3.3V -> VCC
    D21 -> SDA (CJMCU & BME280)
    D22 -> SCL (CJMCU & BME280)

    ## CJMCU
    GND -> GND
    GND -> Wake
    3.3V -> VIN
    D21 -> SDA (CJMCU & BME280)
    D22 -> SCL (CJMCU & BME280)


    ## Sharp Dust Sensor
    GND -> Black
    GND -> Green
    3.3V -> White
    3.3V -> R150 -> Red -> Cap220uF -> GND
    D32 -> Sharp LED (yellow)
    D35 -> Sharp Meausre (blue)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <PingKeepAlive.h>
#include <RhaNtp.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_CCS811.h"
#include "ThingSpeak.h"
#include "settings.h"

#define SHARP_MEASURE 35
#define SHARP_LED 32
#define SHARP_SAMPLE_TIME_MICROS 280
#define SHARP_DELTA_TIME_MICROS 40
#define SHARP_SLEEP_TIME_MICROS 9680

#define SEALEVELPRESSURE_HPA (1013.25)

#define THINGSPEAK_WRITE_FREQUENCY_MS 60 * 1000
unsigned long lastThingSpeakWrite = 0;

#define SENSOR_READ_FREQUENCY_MS 1000
unsigned long lastSensorRead = 0;

#define DEBUG_FREQUENCY_MS 1000
unsigned long lastDebug = 0;

Adafruit_CCS811 ccs;
Adafruit_BME280 bme;

static WebServer server(80);
static WiFiClient client;
PingKeepAlive pka;
RhaNtp ntp;

time_t bootTime = 0; // boot time

float temp;
uint16_t co2;
uint16_t tvoc;

float bmeTemp;
float bmePressure;
float bmeHumidity;

float voMeasured;
float calcVoltage;
float dustDensity;

time_t requestTime()
{
  ntp.updateTime();
  return 0;
}

void handleRoot() 
{
  String page = "Hi";
  page += "\r\n";
  page += millis();
  
  server.send(200, "text/plain", page);
}

bool sendData()
{
  ThingSpeak.setField(1, co2); // CO2
  ThingSpeak.setField(2, tvoc); // TOVC
  ThingSpeak.setField(3, bmeTemp); // Temperature
  ThingSpeak.setField(4, bmePressure); // PressureHpa
  ThingSpeak.setField(5, bmeHumidity); // Humidity
  ThingSpeak.setField(6, dustDensity); // DustDensity
  //ThingSpeak.setField(7, ); // NoiseLevel

  int stat = ThingSpeak.writeFields(channelNumber, writeApiKey);
  if (stat == 200)
    return true;

  // failed.
  return false;
}

void wifiDisconnect()
{
  
}

void wifiReconnect()
{
  
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("Makerspace Environmental Sensor");

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  pinMode(SHARP_LED, OUTPUT);
  pinMode(SHARP_MEASURE, INPUT);

  //calibrate temperature sensor
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  pka.onDisconnect(wifiDisconnect);
  pka.onReconnect(wifiReconnect);

  IPAddress timeServerIP;
  WiFi.hostByName(TIMESERVER, timeServerIP);
  ntp.init(timeServerIP, TIMEZONE);
  setSyncProvider(requestTime);
  setSyncInterval(60 * 60); // every hour

  server.on("/", handleRoot);
  server.begin();

  ThingSpeak.begin(client);
}

void loop() {
  server.handleClient();
  pka.loop();
  ntp.loop();
  if (bootTime == 0 && timeStatus() == timeSet)
    bootTime = ntp.localNow();

  if (millis() - lastSensorRead > SENSOR_READ_FREQUENCY_MS) {
    lastSensorRead = millis();
    if(ccs.available()){
      temp = ccs.calculateTemperature();
      if(!ccs.readData()){
        co2 = ccs.geteCO2();
        tvoc = ccs.getTVOC();
      }
      else{
        Serial.println("error reading ccs sensor");
      }
    }

    bmeTemp = bme.readTemperature();
    bmePressure = bme.readPressure() / 100.0F;
    bmeHumidity = bme.readHumidity();

    digitalWrite(SHARP_LED, LOW); // power on LED
    delayMicroseconds(SHARP_SAMPLE_TIME_MICROS);
    voMeasured = analogRead(SHARP_MEASURE);
    delayMicroseconds(SHARP_DELTA_TIME_MICROS);
    digitalWrite(SHARP_LED, HIGH); // power off LED
    delayMicroseconds(SHARP_SLEEP_TIME_MICROS);
    // 0 - 3.3V mapped to 0 - 1023 integer values
    calcVoltage = voMeasured * (3.3 / 1024);
    // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
    // Chris Nafis (c) 2012
    dustDensity = 0.17 * calcVoltage - 0.1;
  }

  if (millis() - lastDebug > DEBUG_FREQUENCY_MS) {
    lastDebug = millis();
    Serial.print("CO2: ");
    Serial.print(co2);
    Serial.print("ppm, TVOC: ");
    Serial.print(tvoc);
    Serial.print("ppb   Temp:");
    Serial.println(temp);

    Serial.print("BME:    Temp: ");
    Serial.print(bmeTemp);
    Serial.print("*C,     Pres: ");
    Serial.print(bmePressure);
    Serial.print("hPa,   Humi: ");
    Serial.print(bmeHumidity);
    Serial.println("%");
  
    Serial.print("SHARP:    Raw Val: ");
    Serial.print(voMeasured);
    Serial.print(",   Voltage: ");
    Serial.print(calcVoltage);
    Serial.print(",   Dust Density: ");
    Serial.println(dustDensity);

    Serial.println();
  }

  if (millis() - lastThingSpeakWrite > THINGSPEAK_WRITE_FREQUENCY_MS) {
    lastThingSpeakWrite = millis();
    bool ret = sendData();
    if (ret)
      Serial.println("ThingSpeak write ok");
    else
      Serial.println("ThingSpeak write failed");
  }
}
