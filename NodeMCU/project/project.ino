#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

SoftwareSerial uart(D7, D8);

const char* ssid = "hpppp2g";
const char* password = "kizumonogatari0885801586";
const char* mqtt_server = "broker.netpie.io";
const int mqtt_port = 1883;
const char* mqtt_Client = "cfb7ca5d-f6f2-4107-95a6-52ee48d50280";
const char* mqtt_username = "w19WZEKb2Y3xxtAeouWWebUec3WX9Lb9";
const char* mqtt_password = "K#B)I-w9ykoIK8poOJz_IjqZGK6F~$JV";

WiFiClient espClient;
PubSubClient client(espClient);
char msg[200];
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("@msg/led");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(5000);
    }
  }
}
void callback(char* topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    String message;
    for (int i = 0; i < length; i++) {
        message = message + (char)payload[i];
    }
    Serial.println(message);
    
}

void setup() {

  Serial.begin(9600);
  uart.begin(9600);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}


void loop() {



  if (uart.available()) {

    String s = "";
    char c = 0;
    while (1) {
      if (uart.available()) {
        c = uart.read();
        if (c == '\n')
          break;
        s += c;
      }
    }


    Serial.println(s);

    String data = "{\"data\": {\"humidity\":";
    int i = 0, size = s.length();
    while (i < size) {
      if (s[i] == '|')
        break;
      data += s[i];
      i++;
    }
    i++;
    data += ", \"temperature\":";
    while (i < size) {
      if (s[i] == '|')
        break;
      data += s[i];
      i++;
    }
    i++;
    data += ", \"dustdensity\":";
    while (i < size) {
      if (s[i] == '|')
        break;
      data += s[i];
      i++;
    }
    i++;
    data += ", \"heatindex\":";
    while (i < size) {
      if (s[i] == '|')
        break;
      data += s[i];
      i++;
    }
   i++;
    data += ", \"gas\":";
    while (i < size) {
      if (s[i] == '|')
        break;
      data += s[i];
      i++;
    }


  data += "}}";
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    data.toCharArray(msg, (data.length()+1));
    Serial.println(msg);
    client.publish("@shadow/data/update", msg);
    
  }
  
}
