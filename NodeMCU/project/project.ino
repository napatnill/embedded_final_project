#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

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
bool initialized = 0;
char msg[200];
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection…");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("connected");
      client.subscribe("@private/#");
      client.subscribe("@msg/mode");
      client.subscribe("@msg/max_dustdensity");
      client.subscribe("@msg/max_temperature");
      client.subscribe("@msg/max_humidity");
      client.subscribe("@msg/max_gas");
      client.subscribe("@msg/min_dustdensity");
      client.subscribe("@msg/min_gas");
      client.subscribe("@msg/min_humidity");
      client.subscribe("@msg/min_temperature");





    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      delay(10000);
    }
  }
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
  // client.publish("@shadow/data/get", "");
}

const unsigned long RESPONSE_TIMEOUT = 10000;  // Timeout value in milliseconds
const unsigned int MAX_RETRY_COUNT = 3;        // Maximum number of retries

bool waitingForResponse = false;
bool responseReceived = false;
bool recp = false;
unsigned long responseTimer = 0;
unsigned int retryCount = 0;

float mn_du;
float mn_gas;
float mn_hu;
float mn_tem;
int mode;
float mx_du;
float mx_gas;
float mx_hu;
float mx_tem;


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message;


  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.println(message);



  if (String(topic) == "@private/shadow/data/get/response") {
    StaticJsonDocument<512> doc;
    deserializeJson(doc, payload, length);
    responseReceived = true;

    mn_du = doc["data"]["mn_du"];
    mn_gas = doc["data"]["mn_gas"];
    mn_hu = doc["data"]["mn_hu"];
    mn_tem = doc["data"]["mn_tem"];
    mode = doc["data"]["mode"];
    mx_du = doc["data"]["mx_du"];
    mx_gas = doc["data"]["mx_gas"];
    mx_hu = doc["data"]["mx_hu"];
    mx_tem = doc["data"]["mx_tem"];


    Serial.println(mn_du);
    Serial.println(mn_gas);
    Serial.println(mn_hu);
    Serial.println(mn_tem);
    Serial.println(mode);
    Serial.println(mx_du);
    Serial.println(mx_gas);
    Serial.println(mx_hu);
    Serial.println(mx_tem);
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  if (!recp) {
    initia();
    delay(100);
  } else {
    comu();
  }

  //
}


void initia() {
  if (initialized == 0) {
    if (!waitingForResponse) {
      Serial.println("Requesting shadow data");
      client.publish("@shadow/data/get", "", 1);
      waitingForResponse = true;
      responseTimer = millis();  // Start a timer to track the response time
      retryCount = 0;            // Reset the retry count
    } else {
      // Check if a response has been received or timeout has occurred
      if (responseReceived || (millis() - responseTimer) >= RESPONSE_TIMEOUT) {
        if (responseReceived) {
          Serial.println("Shadow data received");
          recp = true;

        } else {
          Serial.println("Shadow data request timeout");
          retryCount++;

          if (retryCount <= MAX_RETRY_COUNT) {
            Serial.println("Retrying...");
            waitingForResponse = false;
            responseReceived = false;
            responseTimer = 0;
          } else {
            Serial.println("Maximum retry count reached");
            waitingForResponse = false;
            responseReceived = false;
            responseTimer = 0;
            initialized = 1;  // Set initialized to 1 to proceed with other logic
          }
        }
      }
    }
  }
}
void comu() {
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
    if (s.length() >= 32) {
      Serial.println("bad receive");
      return;
    }

    String data = "{\"data\": {\"hu\":";
    int i = 0, size = s.length();
    while (i < size) {
      if (s[i] == '|')
        break;
      data += s[i];
      i++;
    }
    i++;
    data += ", \"tem\":";
    while (i < size) {
      if (s[i] == '|')
        break;
      data += s[i];
      i++;
    }
    i++;
    data += ", \"du\":";
    while (i < size) {
      if (s[i] == '|')
        break;
      data += s[i];
      i++;
    }
    i++;
    data += ", \"hi\":";
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


    data.toCharArray(msg, (data.length() + 1));
    Serial.println(msg);

    if (!client.connected()) {
      reconnect();
    }

    client.loop();
    client.publish("@shadow/data/update", msg);
    // delay(5000);
    //  client.loop();
    //   client.publish("@shadow/data/get", "");
  }
}
