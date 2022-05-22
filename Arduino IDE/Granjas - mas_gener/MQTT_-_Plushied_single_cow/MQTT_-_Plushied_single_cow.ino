#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>

// Temperatura definations and libraries
#include "DHT.h"       // Include DHT library
#define DHT_PIN 13     // Defines pin number to which the sensor is connected
#define DHT_TYPE DHT11 // Defines the sensor type. It can be DHT11 or DHT22
DHT dhtSensor(DHT_PIN, DHT_TYPE); // Defines the sensor dht

// GPS definitions and libaries 
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#define RX_PIN 16         // Pinout RX of ESP32
#define TX_PIN 17         // Pinout TX of ESP32
HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

// Sleep time to save battery
#define SLEEP_TIME  1200  // Time ESP32 will go into deep sleep (in seconds)  (20min)

//------------------------------------------------//

// topic definition
#define topico "cow"

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "MOVISTAR_3C98"; //"Galaxy A32EDA8"; //"AulaAutomatica";
const char *WIFI_PASSWORD = "";
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
  dhtSensor.begin(); // Starts sensor communication

  mqttClient.setServer(MQTT_BROKER_IP,// Donde esta el servidor
                       MQTT_PORT); // Connect the configured mqtt broker

  SerialGPS.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Starts gps communication with UART
  
  connectToWiFiNetwork(); // Connects to the configured network
  connectToMqttBroker();  // Connects to the configured mqtt broker

  esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000000);  // Setup to wake up after 30 seconds, defined in microseconds

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
    
    // The SPE32 is sent to sleep in order to save batery time
    Serial.println("Going to sleep for " + String(SLEEP_TIME) + " seconds");
    Serial.flush();  // Clears all unsent serial information
    esp_deep_sleep_start();  // Set the ESP32 to deep sleep mode
  }
}

// ---------------------Get data------------------//


int get_temperature(){

  static float temperature;
    
  
  temperature = dhtSensor.readTemperature(); // Reads the temperature, it takes
                                             // about 250 milliseconds
  Serial.println("Temperature: " + String(temperature) + "°C"); // Prints in a new line the result

  // Crear un int de 1 decimal
  temperature = round(temperature*10);
  
  return temperature;
}

float get_location(int x){    // x=0 para solicitar la latitud, x=1 para solicitar la longitud
   
  // setup GPS 

  if (SerialGPS.available()) {    // Se consulta si el GPS esta disponible
    gps.encode(SerialGPS.read()); // Encodes all messages from GPS
    if (x==0){
      if (gps.location.lat()==0){ // se fuerza una posicion en la granja para la pueba de concepto
        return 41.9133717;  
      }
      else{
      return gps.location.lat();
     }
    }
    if (x==1){
    return gps.location.lng();
    }
  }
  else{
    if (x==0){
    return 41.9133717;   // Si no esta conectado envia una localizacion dentro de la granja
    }
    if (x==1){
    return 2.80411939;
    }
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

  doc["t"]    = get_temperature();
  doc["lat"]  = get_location(0); // 0 para solicitar la latitud
  doc["lon"]  = get_location(1); // 1 para solicitar la longitud

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
