/*
  ***
  ** TODO / Ideas for expansion:
  ***
   - find out why telnet debug disconnects after only a short while.
   - convert Sharp code into a library
   - check Sharp calculations (something about auto tuning the min value)
   - sometimes the CCS sensor doesn't init after flashing and just errors
   - consider keeping an average of readings to send to Thingspeak rather than just point in time
   - add some pretty charts and gauges to the website:
      - page with some gauges showing the current readings
      - page with some charts that build up history while the page is open (using d3js?)  Could also fetch immediate history from Thingspeak on page load?
   - add some physical level indicators, LED's, physical needle gauge, whatever
   - add noise level sensor
   - do something more useful with NTP time

  ***
  ** Building / Flashing:
  ***
    Using Arudino IDE 1.8.9 and ESP32 plugin version 1.0.2 (no reason why later / other versions shouldn't work as well)

    Update settings.h to include wifi details and Thingspeak API key (don't commit these to github!)

    Flash as ESP32 Dev Module (default settings)
     - 4MB (32Mb)
     - 240Mhz (Wifi/BT)
     - 80Mhz flash, QIO
     - Default 4MB with spiffs (1.2MB APP / 1.5MB SPIFFS)
     - PSRAM: disabled
    
    Flash sketch as usual using Ardinuo IDE, use the ESP32 sketch data upload tool to uploade 
    the files in the data folder to the SPIFFS for the webserver to serve out.  (remember to 
    close the Arduino serial monitor window when uploading the SPIFFS or it errors)

    Otherwise connect to envirosense.local/admin and use OTA updates

  ***
  ** Wiring:
  ***
    ESP32 Dev Module

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
#include "FS.h"
#include "SPIFFS.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_CCS811.h"
#include "ThingSpeak.h"
#include "settings.h"

// Timing paramters for reading the Sharp dust sensor.
#define SHARP_MEASURE 35
#define SHARP_LED 32
#define SHARP_SAMPLE_TIME_MICROS 280
#define SHARP_DELTA_TIME_MICROS 40
#define SHARP_SLEEP_TIME_MICROS 9680

// Frequencies for reading sensors and logging values
#define SENSOR_READ_FREQUENCY_MS 2000
#define THINGSPEAK_WRITE_FREQUENCY_MS 60 * 1000

// Create networking and housekeeping related things
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
// How to to wait for a lock on the semaphore
#define SEMAPHORE_WAIT_TIME_MS 1000

///
/// called by the time library when the time needs to be sync'd
///
time_t requestTime()
{
  DEBUG("Running NTP Update\n");
  ntp.updateTime();
  return 0;
}

///
/// format bytes
///
String formatBytes(size_t bytes) {
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  } else {
    return String(bytes / 1024.0 / 1024.0 / 1024.0) + "GB";
  }
}

///
/// returns the conent type for a file
///
String getContentType(String filename) {
    if (filename.endsWith(".htm")) {
        return "text/html";
    } else if (filename.endsWith(".html")) {
        return "text/html";
    } else if (filename.endsWith(".css")) {
        return "text/css";
    } else if (filename.endsWith(".png")) {
        return "image/png";
    }
    return "text/plain";
}

///
/// deal with serving files out from SPIFFS internal flash filesystem
///
bool handleFileRead(String path) {
  if (path.endsWith("/")) {
    path += "index.htm";
  }
  String contentType = getContentType(path);
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

///
/// return a small json result with the latest readings
///
void handleDataJson()
{
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "applicaiton/json", "{");

  if (xSemaphoreTake(xSemaphore_lastReading, portMAX_DELAY)) {
    server.sendContent("\"millis\":"); server.sendContent(String(millis()));
    server.sendContent(",\"co2\":"); server.sendContent(String(lastReading.co2));
    server.sendContent(",\"tvoc\":"); server.sendContent(String(lastReading.tvoc));
    server.sendContent(",\"bmeTemp\":"); server.sendContent(String(lastReading.bmeTemp));
    server.sendContent(",\"bmePressure\":"); server.sendContent(String(lastReading.bmePressure));
    server.sendContent(",\"bmeHumidity\":"); server.sendContent(String(lastReading.bmeHumidity));
    server.sendContent(",\"dustDensity\":"); server.sendContent(String(lastReading.dustDensity));
    //server.sendContent(",\"noise\":"); server.sendContent(String(lastReading.noise));

    xSemaphoreGive(xSemaphore_lastReading);
  }

  server.sendContent("}");
}

///
/// actions to take when wifi disconnects
///
void wifiDisconnect()
{

}

///
/// actions to take when wifi reconnects
///
void wifiReconnect()
{
  
}

///
/// returns x as a two digit number.  eg 5 -> 05, 10 -> 10
///
String formatNumber(int x)
{
  if (x < 10)
    return String("0") + String(x);
  else
    return String(x);
}

///
/// returns a string representation of the given time_t
///
String formatTime(time_t t)
{
  return String(year(t)) 
    + "-" + String(formatNumber(month(t))) 
    + "-" + String(formatNumber(day(t)))
    + " " + String(formatNumber(hour(t))) 
    + ":" + String(formatNumber(minute(t))) 
    + ":" + String(formatNumber(second(t)));
}

///
/// FreeRTOS task that takes care of the main networking and housekeeping things
///
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

///
/// FreeRTOS task that regularly reads the sensors and updates the latest reading 
///
void taskReadSensors(void * parameter)
{
  // Give other stuff time to start before starting sensors
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  // Create sensor objects
  Adafruit_CCS811 ccs;
  Adafruit_BME280 bme;

  // Start up sensors
  DEBUG("Init BME sensor\n");
  if (!bme.begin())
    DEBUG_E("Could not find a valid BME280 sensor, check wiring!\n");

  vTaskDelay(100 / portTICK_PERIOD_MS);

  DEBUG("Init CCS sesnor\n");
  if(!ccs.begin())
    DEBUG_E("Failed to start ccs sensor! Please check your wiring.\n");

  ccs.setDriveMode(CCS811_DRIVE_MODE_IDLE);

  DEBUG("Init Sharp Dust sensor\n");
  pinMode(SHARP_LED, OUTPUT);
  pinMode(SHARP_MEASURE, INPUT);

  // Local variables get populated as part of reading the sensors before copying to the global variable
  SensorReading newData = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  // Give sensors time to init before reading
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  while(1) {

    // Read bme280 to get temperate, humidity and pressure
    newData.bmeTemp = bme.readTemperature();
    newData.bmePressure = bme.readPressure() / 100.0F;
    newData.bmeHumidity = bme.readHumidity();

    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Read ccs sensor to get co2 and tvoc data
    if(ccs.available()){

      if (!isnan(newData.bmeHumidity) && !isnan(newData.bmeTemp)) {
        // Feed temp and Humidity into css calibration 
        // (note that the necessary +25 temperature offset is applied by the library)
        ccs.setEnvironmentalData((int)newData.bmeHumidity, newData.bmeTemp);
      }

      // Read the data
      newData.temp = ccs.calculateTemperature();
      if(!ccs.readData()){
        newData.co2 = ccs.geteCO2();
        newData.tvoc = ccs.getTVOC();
      }
      else {
        DEBUG_E("error reading ccs sensor, resetting\n");
        ccs.begin();
      }
    } else {
      DEBUG_E("CCS sensor not available resetting\n");
      ccs.begin();
    }

    // Read sharp dust sensor
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

    // Log out latest readings to serial and telnet
    DEBUG_D("CSS:   CO2: %dppm,  TVOC %dppb,  Temp:%s\n", newData.co2, newData.tvoc, String(newData.temp).c_str());
    DEBUG_D("BME:   Temp: %s*C,  Pres: %shpa,  Humi: %s%%\n", String(newData.bmeTemp).c_str(), String(newData.bmePressure).c_str(), String(newData.bmeHumidity).c_str());
    DEBUG_D("SHARP: Raw: %s,  Volt: %sV,  Dust Density: %s\n", String(newData.voMeasured).c_str(), String(newData.calcVoltage).c_str(), String(newData.dustDensity).c_str());

    // Update global latest reading value
    if (xSemaphoreTake(xSemaphore_lastReading, SEMAPHORE_WAIT_TIME_MS / portTICK_PERIOD_MS)) {
      lastReading = newData;
      xSemaphoreGive(xSemaphore_lastReading);
      DEBUG_V("lastReading Updated\n");

    } else {
      DEBUG_E("Failed to get mutex lock on lastReading\n");
    }

    // Wait a bit before reading again
    vTaskDelay(SENSOR_READ_FREQUENCY_MS / portTICK_PERIOD_MS);
  }
}

///
/// FreeRTOS task to regularly send the current readings to Thingspeak
///
void taskUpdateThingspeak(void * parameter)
{
  // Inital delay to let the sensors start
  vTaskDelay(15000 / portTICK_PERIOD_MS);

  // Init Thingspeak library for pushing out reading data
  DEBUG("Init Thingspeak\n");
  WiFiClient thingSpeakClient;
  ThingSpeak.begin(thingSpeakClient);

  while(1) {
    
    // Run delay first, so that the sensors have time to read
    vTaskDelay(THINGSPEAK_WRITE_FREQUENCY_MS / portTICK_PERIOD_MS);

    if (xSemaphoreTake(xSemaphore_lastReading, SEMAPHORE_WAIT_TIME_MS / portTICK_PERIOD_MS)) {
      DEBUG("Sending data to Thingspeak\n");

      // Copy current values into fields ready for sending
      ThingSpeak.setField(1, lastReading.co2); // CO2
      ThingSpeak.setField(2, lastReading.tvoc); // TOVC
      ThingSpeak.setField(3, lastReading.bmeTemp); // Temperature
      ThingSpeak.setField(4, lastReading.bmePressure); // PressureHpa
      ThingSpeak.setField(5, lastReading.bmeHumidity); // Humidity
      ThingSpeak.setField(6, lastReading.dustDensity); // DustDensity
      //ThingSpeak.setField(7, ); // NoiseLevel

      // Release mutex before doing the network call as it sometimes 
      // takes a little time and we don't want to block for too long
      xSemaphoreGive(xSemaphore_lastReading);

      // Make the call to send the data
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

///
/// Standard Arduino setup to get everything configured
///
void setup() 
{
  Serial.begin(115200);
  Serial.println("Makerspace Environmental Sensor");

  Wire.begin();
  Wire.setTimeout(100); // default is 50ms

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
  //Debug.setSerialEnabled(true);
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

  // Start FS and dump out files
  SPIFFS.begin();
  File root = SPIFFS.open("/");
  File f = root.openNextFile();
  while (f) {
      String fileName = f.name();
      size_t fileSize = f.size();
      Debug.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
      f = root.openNextFile();
  }
  Debug.printf("\n");

  // Setup webserver callbacks
  DEBUG("Setting up webserver\n");
  server.on("/data.json", handleDataJson);
  server.onNotFound( []() { 
    if (!handleFileRead(server.uri()))
        server.send ( 404, "text/plain", "page not found" ); 
    });
  httpUpdater.setup(&server);
  server.begin();

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
              0);  /* Core to run task on. */

  xTaskCreatePinnedToCore(
              taskReadSensors,   /* Task function. */
              "ReadSensors",     /* String with name of task. */
              10000,            /* Stack size in bytes. */
              NULL,             /* Parameter passed as input of the task */
              1,                /* Priority of the task. */
              NULL,             /* Task handle. */
              1);  /* Core to run task on. */

  xTaskCreatePinnedToCore(
              taskUpdateThingspeak,   /* Task function. */
              "UpdateThingspeak",     /* String with name of task. */
              10000,            /* Stack size in bytes. */
              NULL,             /* Parameter passed as input of the task */
              1,                /* Priority of the task. */
              NULL,             /* Task handle. */
              0);  /* Core to run task on. */
  
}

///
/// Standard Arduino loop, nothing to do because everything happens in FreeRTOS tasks
///
void loop() 
{
  // Nothing to do here, everything runs in tasks
}