#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include "Definition.h"

WiFiClient espClient;
PubSubClient client;

String a;

const byte numChars = 150;
char receivedChars[numChars];
boolean newData = false;
char data;

void setup(){
  Serial.begin(9600);
  
  setup_wifi();
  client.setClient(espClient);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Serial.println("");
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    if (client.connect("EspLiam")) {
      Serial.println(F("connected"));
      client.subscribe(TOPIC_LIAM_COMMAND);
      client.publish(TOPIC_LIAM_CONSOLE, String("Starting EspLiam").c_str(), true);

    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
 
  char input[length];

  for (int i = 0; i < length; i++) {
    input[i]=payload[i];
  }

  client.publish(TOPIC_ESP_CONSOLE, "Received data on topic: ", true);
  client.publish(TOPIC_ESP_CONSOLE, topic, true);
  client.publish(TOPIC_ESP_CONSOLE, input, true);

  /* Send all incoming json to liam */
  Serial.println((char*)input);
}

void loop(){

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  receiveJsonFromSerial();
  publishJsonFromLiam();
}

void receiveJsonFromSerial() {
    static boolean recvInProgress = false;
    static byte ndx = 1;
    char startMarker = '{';
    char endMarker = '}';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
        if (recvInProgress == true) {
            if (rc != endMarker) {

                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '}'; // Append end marker
                ndx++;
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 1;
                newData = true;
            }
        }
        else if (rc == startMarker) {
            recvInProgress = true;
            receivedChars[0] = '{';
        }
    }
}

void publishJsonFromLiam() {
  
    if (newData == true) {
        // Max number chars to receive is 108
        client.publish(TOPIC_LIAM_CONSOLE, String(receivedChars).c_str(), true);
        newData = false;
    } 
}
