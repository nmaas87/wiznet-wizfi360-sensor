/*
  WiFi Connected Environmental Sensor
  The following project pairs an WizFi360-EVB-Pico with a Bosch BME680 to allow for an WiFi 
  powered environmental sensor which can register temperature, humidity, air pressure 
  and air quality levels and sent it via MQTT to e.g. an InfluxDB / Grafana instance.
  Witten for the WIZnet WizFi360 Contest 2022 by Nico Maas @ 2022-08-08
  used libraries
  - earlephilhower arduino-pico  https://github.com/earlephilhower/arduino-pico  @ 2.3.3
  - WizFi360  https://github.com/Wiznet/WizFi360_arduino_library  @ current version
  - Adafruit BME680  https://github.com/adafruit/Adafruit_BME680  @ 2.0.2
  - ArduinoJson  https://github.com/bblanchon/ArduinoJson  @ 6.19.4
  re-used examples
  - WizFi360: ConnectWPA
  - BME680 Air Quality (https://draeger-it.blog/arduino-lektion-113-umweltsensor-bme680-fuer-rel-luftfeuchtigkeit-luftdruck-temperatur-und-luftqualitaet/)
*/

#include "WizFi360.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ArduinoJson.h>

// User settings to configure
// Sensor ID (set to different if multiple sensors in the network)
byte id = 1;
/* Wi-Fi info */
char ssid[] = "wiznet";       // your network SSID (name)
char pass[] = "0123456789";   // your network password
/* MQTT info */
// MQTTSET
String mqttUserName="public";
String mqttPassword="public";
String mqttClientID="WizFi3601";
String mqttAliveTime="60";
// MQTTTOPIC
String mqttPublishTopic="wizfiSensor";
String mqttSubscribeTopic="wizfiSensor";
// MQTTCON
String mqttServer="public.cloud.shiftr.io";
String mqttPort="1883";

/* WizFi360 */
#define WIZFI360_EVB_PICO
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial2(6, 7); // RX, TX
#endif

/* Baudrate */
#define SERIAL_BAUDRATE   115200
#define SERIAL2_BAUDRATE  115200

int status = WL_IDLE_STATUS;  // the Wifi radio's status

/* BME680 */
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C
float hum_score, gas_score;
float gas_reference = 250000;
float hum_reference = 40;
int   getgasreference_count = 0;

unsigned long lastMillis = 0;


void setup() {
  // initialize serial for debugging
  Serial.begin(SERIAL_BAUDRATE);
  // initialize serial for WizFi360 module
  Serial2.begin(SERIAL2_BAUDRATE);
  // initialize WizFi360 module
  WiFi.init(&Serial2);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // check if BME680 available
  //if (!bme.begin(0x76)) {
  if (!bme.begin()) {
    Serial.println("BME680 could not be found");
    // don't continue
    while (true);
  }

  // set basic settings for sensor
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println("You're connected to the network");
  printCurrentNet();
  printWifiData();

  Serial.println("Connecting to MQTT");
  connectMQTT();
}

void loop() {
  // Allocate the JSON document with 240 bytes memory
  StaticJsonDocument<240> sensor;

  // measure and publish a message roughly every 60 seconds
  if (millis() - lastMillis > 57500) {
    
    // BME680 reading
    if (! bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    // get sensor data
    // set sensor id in MQTT message - to identify multiple sensors   
    sensor["id"] = id;
    sensor["temperature"] = String(bme.temperature);
    sensor["humidity"] = String(bme.humidity);
    sensor["gas"] = String(bme.gas_resistance / 1000.0);
    sensor["aQ"] = getAirQuality();
    // output to serial
    Serial.print(String(sensor["id"]));
    Serial.print(" - ");
    Serial.print(String(sensor["temperature"]));
    Serial.print("°C - ");
    Serial.print(String(sensor["humidity"]));
    Serial.print("% - ");
    Serial.print(String(sensor["gas"]));
    Serial.print("kOhms - ");
    Serial.println(String(sensor["aQ"]));
    // prepare JSON and send via MQTT
    String buffer;
    size_t n = serializeJson(sensor, buffer);
    sendData("AT+MQTTPUB=\"" + buffer + "\"","OK",1500,false);
    // note down time for the loop
    lastMillis = millis();
  }
}

void printWifiData() {
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print("MAC address: ");
  Serial.println(buf);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to
  byte bssid[6];
  WiFi.BSSID(bssid);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[5], bssid[4], bssid[3], bssid[2], bssid[1], bssid[0]);
  Serial.print("BSSID: ");
  Serial.println(buf);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.println(rssi);
}

String getAirQuality() {
  float current_humidity = bme.readHumidity();
  if (current_humidity >= 38 && current_humidity <= 42)
    hum_score = 0.25*100; // Humidity +/-5% around optimum 
  else
  { //sub-optimal
    if (current_humidity < 38) 
      hum_score = 0.25/hum_reference*current_humidity*100;
    else
    {
      hum_score = ((-0.25/(100-hum_reference)*current_humidity)+0.416666)*100;
    }
  }
  
  //Calculate gas contribution to IAQ index
  float gas_lower_limit = 5000;   // Bad air quality limit
  float gas_upper_limit = 50000;  // Good air quality limit 
  if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit; 
  if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
  gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;
  float air_quality_score = hum_score + gas_score;
  if ((getgasreference_count++)%10==0) GetGasReference();
return CalculateIAQ(air_quality_score);
}

void GetGasReference(){
  int readings = 10;
  for (int i = 1; i <= readings; i++){
    gas_reference += bme.readGas();
  }
  gas_reference = gas_reference / readings;
}

String CalculateIAQ(float score) {
  String IAQ_text = "Air quality is ";
  score = (100 - score) * 5;
  if      (score >= 301)                  IAQ_text += "Hazardous";
  else if (score >= 201 && score <= 300 ) IAQ_text += "Very Unhealthy";
  else if (score >= 176 && score <= 200 ) IAQ_text += "Unhealthy";
  else if (score >= 151 && score <= 175 ) IAQ_text += "Unhealthy for Sensitive Groups";
  else if (score >=  51 && score <= 150 ) IAQ_text += "Moderate";
  else if (score >=  00 && score <=  50 ) IAQ_text += "Good";
  return IAQ_text;
}

void connectMQTT() {
  if (sendData("AT+MQTTSET=\"" + mqttUserName + "\",\"" + mqttPassword + "\",\"" + mqttClientID + "\"," + mqttAliveTime + "","OK",1500,false)) {
    Serial.println("MQTTSET: OK");
    if (sendData("AT+MQTTTOPIC=\"" + mqttPublishTopic + "\",\"" + mqttSubscribeTopic + "\"","OK",1500,false)) {
      Serial.println("MQTTTOPIC: OK");
      if (sendData("AT+MQTTCON=0,0,\"" + mqttServer + "\"," + mqttPort + "","0,CONNECT",2500,false)) {
        Serial.println("MQTTCON: OK");
      } else {
        Serial.println("MQTTCON: ERROR");
      }
    } else {
      Serial.println("MQTTCON: ERROR");
    }
  } else {
    Serial.println("MQTTSET: ERROR");
  }
}

boolean sendData(String command, String key, int timeout, boolean debug)
{
    String result = "";
    boolean success = false;

    // clean Serial2 buffer from previous comms
    while(Serial2.available())
      Serial2.read();

    // send the new command    
    Serial2.print(command + "\r\n");
    // and wait until sending has finished
    Serial2.flush();

    // calculate backoffTime
    long backoffTime = millis() + timeout;
    // try to get an answer  
    while(backoffTime > millis())
    {
      // if new data on serial
      while(Serial2.available())
      {
        // read the next character
        char c = Serial2.read();
        // and add to result string 
        result+=c;
      }
      // if the result string contains they keyword we are looking for, exit
      if(result.indexOf(key) >= 0) {
        while(Serial2.available())
          Serial2.read();
        delay(25);
        backoffTime = millis();
        success = true;
      } else {
        // else, keep on waiting
        success = false;       
      }     
    }

    if(debug)
    {
      if(result.indexOf(key) >= 0) {
        Serial.println(":yes:"+result);
      } else {
        Serial.println(":no:"+result);
      }
    }
    return success;
}
