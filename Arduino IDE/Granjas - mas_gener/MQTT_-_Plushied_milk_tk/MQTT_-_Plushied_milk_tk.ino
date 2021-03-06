#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>

// Temperatura sensor definations and libraries
#include "DHT.h"       // Include DHT library
#define DHT_PIN 12     // Defines pin number to which the sensor is connected
#define DHT_TYPE DHT11 // Defines the sensor type. It can be DHT11 or DHT22

// GPS definitions and libaries 
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#define RX_PIN 16         // Pinout RX of ESP32
#define TX_PIN 17         // Pinout TX of ESP32
#define REFRESH_RATE 5000 // Defined in miliseconds
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

// Level sensor definations and libraries
#define LV_PIN 36   // LV  Pin
#define ANALOG_BIT_RESOLUTION 12.0

//------------------------------------------------//

// topico raiz
#define topico "tk"

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = 
const char *WIFI_PASSWORD = 
char macAddress[18];

const char *MQTT_BROKER_IP = "iiot-upc.gleeze.com";
const int   MQTT_PORT = 1883;
const char *MQTT_USER = "iiot-upc";
const char *MQTT_PASSWORD = "";
const bool  RETAINED = true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");

  DHT dhtSensor(DHT_PIN, DHT_TYPE); // Defines the sensor dht
  dhtSensor.begin(); // Starts sensor communication
  
  mqttClient.setServer(MQTT_BROKER_IP,// Donde esta el servidor
                       MQTT_PORT); // Connect the configured mqtt broker

  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker
}


void loop() {
  checkConnections(); // We check the connection every time
  
  // Publish every 10 seconds
  static int nowTime = millis();
  static int startTime = 0;
  static int elapsedTime = 0;
  nowTime = millis();
  elapsedTime = nowTime - startTime;
  if (elapsedTime >= 10000) {
    publishSmallJson();   // Publishes a small json
    startTime = nowTime;
  }
}

// ---------------------Get data------------------//

// Temperature

int get_temperature(){

  static float temperature;
  
  
  temperature = dhtSensor.readTemperature(); // Reads the temperature, it takes
                                             // about 250 milliseconds
  Serial.println("Temperature: " + String(temperature) + "??C"); // Prints in a new line the result

  // Crear un int de 1 decimal
  temperature = round(temperature*10);
  
  return temperature;
}

// Level transmition
float get_tank_level(){
  
  static float voltage_measure;
  static float level;
  analogReadResolution(ANALOG_BIT_RESOLUTION);  
  
  voltage_measure = analogRead(LV_PIN);  // Reads the value from the pin in a 0-4095 resolution corresponding to a linear 0-3.3V LV_PIN
  
  level = 100*(voltage_measure/4095);    // return de level in porcentage of V read
  
  if (level==0){                        // Si el sensor no lee envia 50% de nivel
    return level=50;
    }
  else{
    return round(level);
    }
}


//-------------------------------------------------//

/* Additional functions */

void publishSmallJson() {
  static const String topicStr = createTopic(topico);
  static const char *topic = topicStr.c_str();

  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  doc["t"]   = get_temperature();
  doc["lv"]  = get_tank_level();

  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}



String createTopic(char *topic) {
  String topicStr = String("amptec") + "/" + topic + "/" + String(macAddress);
  return topicStr;
}

void connectToWiFiNetwork() {
  Serial.print(
      "Connecting with Wi-Fi: " +
      String(WIFI_SSID)); // Print the network which you want to connect
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".."); // Connecting effect
  }
  Serial.print("..connected!  (ip: "); // After being connected to a network,
                                       // our ESP32 should have a IP
  Serial.print(WiFi.localIP());
  Serial.println(")");
  String macAddressStr = WiFi.macAddress().c_str();
  strcpy(macAddress, macAddressStr.c_str());
}

void connectToMqttBroker() {
  Serial.print(
      "Connecting with MQTT Broker:" +
      String(MQTT_BROKER_IP));    // Print the broker which you want to connect
  mqttClient.connect(macAddress, MQTT_USER, MQTT_PASSWORD);// Using unique mac address from ESP32
  while (!mqttClient.connected()) {
    delay(500);
    Serial.print("..");             // Connecting effect
    mqttClient.connect(macAddress); // Using unique mac address from ESP32
  }
  Serial.println("..connected! (ClientID: " + String(macAddress) + ")");
}

void checkConnections() {
  if (mqttClient.connected()) {
    mqttClient.loop();
  } else { // Try to reconnect
    Serial.println("Connection has been lost with MQTT Broker");
    if (WiFi.status() != WL_CONNECTED) { // Check wifi connection
      Serial.println("Connection has been lost with Wi-Fi");
      connectToWiFiNetwork(); // Reconnect Wifi
    }
    connectToMqttBroker(); // Reconnect Server MQTT Broker
  }
}
