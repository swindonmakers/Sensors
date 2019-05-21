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

    Reference Links:
    https://learn.adafruit.com/adafruit-ccs811-air-quality-sensor/arduino-wiring-test
    http://arduinodev.woofex.net/2012/12/01/standalone-sharp-dust-sensor/
    http://www.howmuchsnow.com/arduino/airquality/
    https://www.sparkfun.com/products/9689
    
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <ESP32HTTPUpdateServer.h>
#include <PingKeepAlive.h>
#include <RhaNtp.h>
#include <RemoteDebug.h>

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
static ESP32HTTPUpdateServer httpUpdater;
PingKeepAlive pka;
RhaNtp ntp;
RemoteDebug Debug;

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
  DEBUG("Running NTP Update\n");
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
  DEBUG("Sending data to Thingspeak\n");
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

String formatNumber(int x)
{
  if (x < 10)
    return String("0") + String(x);
  else
    return String(x);
}

String formatTime(time_t t)
{
  return String(year(t)) 
    + "-" + String(formatNumber(month(t))) 
    + "-" + String(formatNumber(day(t)))
    + " " + String(formatNumber(hour(t))) 
    + ":" + String(formatNumber(minute(t))) 
    + ":" + String(formatNumber(second(t)));
}

void taskNetworking(void * parameter)
{
  while(1) {
    Debug.handle();
    yield();
    server.handleClient();
    yield();
    pka.loop();
    yield();
    ntp.loop();
    yield();
    if (bootTime == 0 && timeStatus() == timeSet) {
      bootTime = ntp.localNow();
      DEBUG("Utc time is: %s\n", formatTime(now()).c_str());
      DEBUG("Local time is: %s\n", formatTime(ntp.localNow()).c_str());
    }
    yield();
    if (millis() - lastThingSpeakWrite > THINGSPEAK_WRITE_FREQUENCY_MS) {
      lastThingSpeakWrite = millis();
      bool ret = sendData();
      if (ret) {
        DEBUG("ThingSpeak write ok\n");
      } else {
        DEBUG_E("ThingSpeak write failed\n");
      }
    }
    yield();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Makerspace Environmental Sensor");

  // Connect to wifi
  Serial.print("Connecting to WIFI: ");
  Serial.println(WIFI_STA);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.printf("Connected to %s, IP %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

  // Setup telnet debug library
  Debug.begin("envirosense");
  Debug.setResetCmdEnabled(true);
  Debug.setSerialEnabled(true);
  //String rdbCmds = "dump\r\n";
  //rdbCmds.concat("set_offset <n>\r\n");
  //rdbCmds.concat("set_timeserver <n>\r\n");
  //Debug.setHelpProjectsCmds(rdbCmds);
  //Debug.setCallBackProjectCmds(&processRemoteDebugCmd);

  // Setup MDNS to response to envirosense.local address
  DEBUG("Setup MDNS\n");
  if (!MDNS.begin("envirosense")) {
    DEBUG_E("MDNS setup failed\n");
  } else {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("telnet", "tcp", 23);
  }

  // Set action callbacks for wifi connect and disconnect events
  pka.onDisconnect(wifiDisconnect);
  pka.onReconnect(wifiReconnect);

  // Setup time library to get time via ntp
  DEBUG("Initialise time library\n");
  IPAddress timeServerIP;
  WiFi.hostByName(TIMESERVER, timeServerIP);
  ntp.init(timeServerIP, TIMEZONE);
  setSyncProvider(requestTime);
  setSyncInterval(60 * 60); // every hour

  // Setup webserver callbacks
  DEBUG("Setting up webserver\n");
  server.on("/", handleRoot);
  httpUpdater.setup(&server);
  server.begin();

  // Init Thingspeak library for pushing out reading data
  DEBUG("Init Thingspeak\n");
  WiFiClient thingSpeakClient;
  ThingSpeak.begin(thingSpeakClient);

  // Start up sensors
  DEBUG("Init BME sensor\n");
  if (!bme.begin())
    DEBUG_E("Could not find a valid BME280 sensor, check wiring!\n");

  DEBUG("Init CCS sesnor\n");
  if(!ccs.begin())
    DEBUG_E("Failed to start ccs sensor! Please check your wiring.\n");

  // Calibrate temperature sensor 
  // TODO: put in main loop / do every sensor read?
  DEBUG("Set css temperature calibration\n");
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);

  DEBUG("Init Sharp Dust sensor\n");
  pinMode(SHARP_LED, OUTPUT);
  pinMode(SHARP_MEASURE, INPUT);

  DEBUG("Create RTOS tasks\n");
  xTaskCreatePinnedToCore(
              taskNetworking,   /* Task function. */
              "Networking",     /* String with name of task. */
              10000,            /* Stack size in bytes. */
              NULL,             /* Parameter passed as input of the task */
              1,                /* Priority of the task. */
              NULL,             /* Task handle. */
              tskNO_AFFINITY);  /* Core to run task on. */
                    
}

void loop() {
  // Read sensors
  // TODO: move these to RTOS tasks
  if (millis() - lastSensorRead > SENSOR_READ_FREQUENCY_MS) {
    lastSensorRead = millis();
    if(ccs.available()){
      temp = ccs.calculateTemperature();
      if(!ccs.readData()){
        co2 = ccs.geteCO2();
        tvoc = ccs.getTVOC();
      }
      else{
        DEBUG_E("error reading ccs sensor\n");
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
    DEBUG_D("CSS:   CO2: %dppm,  TVOC %dppb,  Temp:%s\n", co2, tvoc, String(temp).c_str());
    DEBUG_D("BME:   Temp: %s*C,  Pres: %shpa,  Humi: %s%%\n", String(bmeTemp).c_str(), String(bmePressure).c_str(), String(bmeHumidity).c_str());
    DEBUG_D("SHARP: Raw: %s,  Volt: %sV,  Dust Density: %s\n", String(voMeasured).c_str(), String(calcVoltage).c_str(), String(dustDensity).c_str());
  }

  yield();
}