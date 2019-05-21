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

#define SENSOR_READ_FREQUENCY_MS 1000
#define THINGSPEAK_WRITE_FREQUENCY_MS 60 * 1000

Adafruit_CCS811 ccs;
Adafruit_BME280 bme;

static WebServer server(80);
static ESP32HTTPUpdateServer httpUpdater;
PingKeepAlive pka;
RhaNtp ntp;
RemoteDebug Debug;

time_t bootTime = 0; // boot time

// Struct encapsulates all the values that make up a single reading
struct SensorReading {
  float temp;
  uint16_t co2;
  uint16_t tvoc;

  float bmeTemp;
  float bmePressure;
  float bmeHumidity;

  float voMeasured;
  float calcVoltage;
  float dustDensity;
};

// The most recent set of sensor readings
SensorReading lastReading;
// A mutex lock on lastReading - don't access it without holding the mutex
SemaphoreHandle_t xSemaphore_lastReading;

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
  }
}

void taskReadSensors(void * parameter)
{
  // Local variables get populated as part of reading the sensors before copying to the global variable
  SensorReading newData = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  while(1) {
    // Read sensors
    if(ccs.available()){
      newData.temp = ccs.calculateTemperature();
      if(!ccs.readData()){
        newData.co2 = ccs.geteCO2();
        newData.tvoc = ccs.getTVOC();
      }
      else{
        DEBUG_E("error reading ccs sensor\n");
      }
    }

    newData.bmeTemp = bme.readTemperature();
    newData.bmePressure = bme.readPressure() / 100.0F;
    newData.bmeHumidity = bme.readHumidity();

    digitalWrite(SHARP_LED, LOW); // power on LED
    delayMicroseconds(SHARP_SAMPLE_TIME_MICROS);
    newData.voMeasured = analogRead(SHARP_MEASURE);
    delayMicroseconds(SHARP_DELTA_TIME_MICROS);
    digitalWrite(SHARP_LED, HIGH); // power off LED
    delayMicroseconds(SHARP_SLEEP_TIME_MICROS);
    // 0 - 3.3V mapped to 0 - 1023 integer values
    newData.calcVoltage = newData.voMeasured * (3.3 / 1024);
    // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
    // Chris Nafis (c) 2012
    newData.dustDensity = 0.17 * newData.calcVoltage - 0.1;

    DEBUG_D("CSS:   CO2: %dppm,  TVOC %dppb,  Temp:%s\n", newData.co2, newData.tvoc, String(newData.temp).c_str());
    DEBUG_D("BME:   Temp: %s*C,  Pres: %shpa,  Humi: %s%%\n", String(newData.bmeTemp).c_str(), String(newData.bmePressure).c_str(), String(newData.bmeHumidity).c_str());
    DEBUG_D("SHARP: Raw: %s,  Volt: %sV,  Dust Density: %s\n", String(newData.voMeasured).c_str(), String(newData.calcVoltage).c_str(), String(newData.dustDensity).c_str());

    if (xSemaphoreTake(xSemaphore_lastReading, portMAX_DELAY)) {
      lastReading = newData;
      xSemaphoreGive(xSemaphore_lastReading);
      DEBUG_V("lastReading Updated\n");

    } else {
      DEBUG_E("Failed to get mutex lock on lastReading\n");
    }

    vTaskDelay(SENSOR_READ_FREQUENCY_MS / portTICK_PERIOD_MS);
  }
}

void taskUpdateThingspeak(void * parameter)
{
  // Init Thingspeak library for pushing out reading data
  DEBUG("Init Thingspeak\n");
  WiFiClient thingSpeakClient;
  ThingSpeak.begin(thingSpeakClient);

  while(1) {
    
    // Run delay first, so that the sensors have time to read
    vTaskDelay(THINGSPEAK_WRITE_FREQUENCY_MS / portTICK_PERIOD_MS);

    if (xSemaphoreTake(xSemaphore_lastReading, portMAX_DELAY)) {
      DEBUG("Sending data to Thingspeak\n");

      ThingSpeak.setField(1, lastReading.co2); // CO2
      ThingSpeak.setField(2, lastReading.tvoc); // TOVC
      ThingSpeak.setField(3, lastReading.bmeTemp); // Temperature
      ThingSpeak.setField(4, lastReading.bmePressure); // PressureHpa
      ThingSpeak.setField(5, lastReading.bmeHumidity); // Humidity
      ThingSpeak.setField(6, lastReading.dustDensity); // DustDensity
      //ThingSpeak.setField(7, ); // NoiseLevel

      xSemaphoreGive(xSemaphore_lastReading);

      int stat = ThingSpeak.writeFields(channelNumber, writeApiKey);
      if (stat == 200) {
        DEBUG("ThingSpeak write ok\n");
      } else {
        DEBUG_E("ThingSpeak write failed\n");
      }

    } else {
      DEBUG_E("Failed to get mutex lock on lastReading\n");
    }
  }
}

void setup() 
{
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

  // Create mutexes and tasks
  DEBUG("Create RTOS mutexes\n");
  xSemaphore_lastReading = xSemaphoreCreateMutex();

  DEBUG("Create RTOS tasks\n");
  xTaskCreatePinnedToCore(
              taskNetworking,   /* Task function. */
              "Networking",     /* String with name of task. */
              10000,            /* Stack size in bytes. */
              NULL,             /* Parameter passed as input of the task */
              1,                /* Priority of the task. */
              NULL,             /* Task handle. */
              tskNO_AFFINITY);  /* Core to run task on. */

  xTaskCreatePinnedToCore(
              taskReadSensors,   /* Task function. */
              "ReadSensors",     /* String with name of task. */
              10000,            /* Stack size in bytes. */
              NULL,             /* Parameter passed as input of the task */
              1,                /* Priority of the task. */
              NULL,             /* Task handle. */
              tskNO_AFFINITY);  /* Core to run task on. */

  xTaskCreatePinnedToCore(
              taskUpdateThingspeak,   /* Task function. */
              "UpdateThingspeak",     /* String with name of task. */
              10000,            /* Stack size in bytes. */
              NULL,             /* Parameter passed as input of the task */
              1,                /* Priority of the task. */
              NULL,             /* Task handle. */
              tskNO_AFFINITY);  /* Core to run task on. */
  
}

void loop() 
{
  // Nothing to do here, everything runs in tasks
  delay(1000);
}