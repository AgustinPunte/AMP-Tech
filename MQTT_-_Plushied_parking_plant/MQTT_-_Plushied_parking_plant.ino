#include <ArduinoJson.h>  // https://arduinojson.org/
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient
#include <WiFi.h>
#include <Wire.h>

// Distance sensor definations and libraries
#define ECHO_PIN 12 // Analog input that receives the echo signal
#define TRIG_PIN 13 // Digital output that sends the trigger signal

//------------------------------------------------//

// Topic definition
#define place "plant"

// Replace the next variables with your Wi-Fi SSID/Password
const char *WIFI_SSID = "MOVISTAR_3C98"; //"Galaxy A32EDA8"; //"AulaAutomatica";
const char *WIFI_PASSWORD = "Tf4dXaF3VFRV4cXKX93T"; //"eiue1560"; //// "ticsFcim";
char macAddress[18];

const char *MQTT_BROKER_IP = "iiot-upc.gleeze.com";
const int   MQTT_PORT = 1883;
const char *MQTT_USER = "iiot-upc";
const char *MQTT_PASSWORD = "cimupc";
const bool  RETAINED = true;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600); // Starts the serial communication
  Serial.println("\nBooting device...");
  
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

// Obtiene la distancia para saber si un camion esta parqueado o no.

bool get_park(){
    
    static long distance;
    static long duration;
    char *truck = "NaN";

    pinMode(ECHO_PIN, INPUT);  // Sets the ECHO_PIN as an Input
    pinMode(TRIG_PIN, OUTPUT); // Sets the TRIG_PIN as an Output

    digitalWrite(TRIG_PIN, LOW); // Clear the TRIG_PIN by setting it LOW
    delayMicroseconds(5);

    // Trigger the sensor by setting the TRIG_PIN to HIGH for 10 microseconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    duration = pulseIn(ECHO_PIN, HIGH); // pulseIn() returns the duration (length of the pulse) in microseconds
    Serial.println("Distance to the object: " + String(distance) + " cm");
    distance = duration * 0.034 / 2; // Returns the distance in cm

    truck = "0407-FCM";
    
    if(truck !="NaN"){
      if(distance<30){
        return 1; 
      }
      else{
       return 0;
      }
    }
}

String get_plate(){
  return "0407-FCM";
  
}

//-------------------------------------------------//

/* Additional functions */

void publishSmallJson() {
  static const String topicStr = createTopic("spot_1");
  static const char *topic = topicStr.c_str();

  StaticJsonDocument<128> doc; // Create JSON document of 128 bytes
  char buffer[128]; // Create the buffer where we will print the JSON document
                    // to publish through MQTT

  doc["spot_1"] = get_park(); // Add names and values to the JSON document
  doc["truck"]  = get_plate(); // Add names and values to the JSON document
 

  serializeJson(doc, buffer);
  mqttClient.publish(topic, buffer, RETAINED);
  Serial.println(" <= " + String(topic) + ": " + String(buffer));
}

String createTopic(char *topic) {
  String topicStr = String(macAddress) + "/" + topic;
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
