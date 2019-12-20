#include "EspMQTTClient.h"

EspMQTTClient client(
  "BOTCONTROL",
  "newcaprica",
  "172.16.0.137",       // MQTT Broker server ip
  "theo",               // Can be omitted if not needed
  "plopkoeken",         // Can be omitted if not needed
  "ESP32",              // Client name that uniquely identify your device
  1883                  // MQTT Port: default = 1883
);

void setup() {
  Serial.begin(115200);
}

void onConnectionEstablished() {}

void loop() {
  client.loop();
  if(client.isMqttConnected()) {
    client.publish("warmste/week", "Hi!");
    Serial.print("Message sent.\n");
  }
  delay(1000);
}
