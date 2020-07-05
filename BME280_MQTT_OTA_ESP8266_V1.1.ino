/*
Project:  BME280 Sensor data send over MQTT with a ESP8266 / NodeMCU
Author:   Thomas Edlinger for www.edistechlab.com
Date:     Created 19.10.2019 / updated 27.10.2019
Version:  V1.1

Changelog:
OTA implementation 
Placeholder for inTopics
 
Required libraries (Tools -> manage libraries)
 - PubSubClient by Nick O'Leary V2.7.0  
 - Adafruit BME280 Library V2.0.1
 - Adafruit Unified Sensor V1.1.2
Required Board (Tools -> Board -> Boards Manager...)
 - Board: esp8266 by ESP8266 Community V2.6.3

Wirering for the BME280 Sensor:
BME280      NodeMCU
VCC         3.3V
GND         G
SCL         D1 / GPIO5
SDA         D2 / GPIO4
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define wifi_ssid "your_SSID"
#define wifi_password "your_Password"

#define mqtt_server "your_MQTT_Server_IP"
#define mqtt_user "your_username"         
#define mqtt_password "your_password"     

#define ESPHostname "ESP8266_01"

#define humidity_topic "esp01/humidity"
#define temperature_topic "esp01/temperature"
#define pressure_topic "esp01/pressure"

#define inTopic "esp01/inTopic"
#define outTopic "esp01/outTopic"

Adafruit_BME280 bme; // I2C
float temp = 0.0;
float hum = 0.0;
float pres = 0.0;
float diff = 1.0;

unsigned long delayTime;

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void setup() {
  Serial.begin(115200);
  setup_wifi();
   ArduinoOTA.setHostname(ESPHostname);
  // ArduinoOTA.setPassword("admin");
  ArduinoOTA.begin();
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  while(!Serial);    // time to get serial running
    unsigned status;
    status = bme.begin(0x76, &Wire);   //I2C address is either 0x76 or 0x77
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
        while (1);
    }
    delayTime = 1000;
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  ArduinoOTA.handle(); 
  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    getBME280Values();
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  if ((char)payload[0] == '1') {
    //If the first Value of the payload is 1 
  } else {
    // Room for Code
  }
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return !isnan(newValue) &&
         (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}

void getBME280Values() {
    float newPres = bme.readPressure() / 100.0F;
    float newTemp = bme.readTemperature();
    float newHum = bme.readHumidity();

    if (checkBound(newTemp, temp, diff)) {
      temp = newTemp;
      Serial.print("New temperature:");
      Serial.println(String(temp).c_str());
      client.publish(temperature_topic, String(temp).c_str(), true);
    }

    if (checkBound(newHum, hum, diff)) {
      hum = newHum;
      Serial.print("New Humidity:");
      Serial.println(String(hum).c_str());
      client.publish(humidity_topic, String(hum).c_str(), true);
    }

    if (checkBound(newPres, pres, diff)) {
      pres = newPres;
      Serial.print("New Pressure:");
      Serial.println(String(pres).c_str());
      client.publish(pressure_topic, String(pres).c_str(), true);
    }  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(outTopic, ESPHostname);
      // ... and resubscribe
      client.subscribe(inTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
