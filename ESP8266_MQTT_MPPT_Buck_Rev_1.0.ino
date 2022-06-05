/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ESP8266 MQTT MPPT Buck Converter [Rev. 1.0]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   This code is under MIT License
   Copyright (c) 2022 Sayantan Sinha
*/
#if !defined(ESP8266)     //These define's must be placed at the beginning before #include "ESP8266TimerInterrupt.h"
#error This code is designed to run on ESP8266 and ESP8266-based boards! Please check your Tools->Board setting.
#endif

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define LED_PIN 2                               // LED at GPIO 2
#define WLAN_SSID       "<WiFi SSID>"           // WiFi Access Point
#define WLAN_PASS       "<password>"
#define AIO_SERVER      "io.adafruit.com"       // Adafruit.io Setup
#define AIO_SERVERPORT  1883                    // use 8883 for SSL
#define AIO_USERNAME    "<aio user name>"
#define AIO_KEY         "<aio key>"

WiFiClient client;  // Create an ESP8266 WiFiClient class to connect to the MQTT server.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);  // Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Publish vpvPublish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pv-voltage");
Adafruit_MQTT_Publish ipvPublish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pv-current");
Adafruit_MQTT_Publish voPublish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bat-voltage");
Adafruit_MQTT_Publish ioPublish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bat-current");
Adafruit_MQTT_Publish effPublish = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/efficiency");

float vpv = 18.45;           // PV voltage
float ipv = 2.25;            // PV current
float vo = 14.15;            // Output/Battery voltage
float io = 2.552;            // Output/Battery current
float eff = 95.59;           // Efficiency

bool MQTT_connect();

void setup()
{
  Serial.begin(9600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {  // Retry every 500 ms
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
  digitalWrite(LED_PIN, HIGH);

  MQTT_connect();

  eff = (vo * io * 100) / (vpv * ipv);
  vpvPublish.publish(vpv, sizeof(vpv));
  delay(800);
  ipvPublish.publish(ipv, sizeof(ipv));
  delay(800);
  voPublish.publish(vo, sizeof(vo));
  delay(800);
  ioPublish.publish(io, sizeof(io));
  delay(800);
  effPublish.publish(eff, sizeof(eff));
  delay(800);
}

void loop()
{
  while (Serial.available()) {
    vpv = Serial.parseFloat();
    ipv = Serial.parseFloat();
    vo = Serial.parseFloat();
    io = Serial.parseFloat();
    Serial.flush();
    eff = (vo * io * 100) / (vpv * ipv);
    vpvPublish.publish(vpv, sizeof(vpv));
    delay(800);
    ipvPublish.publish(ipv, sizeof(ipv));
    delay(800);
    voPublish.publish(vo, sizeof(vo));
    delay(800);
    ioPublish.publish(io, sizeof(io));
    delay(800);
    if (!effPublish.publish(eff, sizeof(eff))) {              // Publish the current efficiency
      mqtt.disconnect();                                      // If publishing fails, reset the MQTT Connection
      delay(10);
      if (!MQTT_connect()) {
        while (WiFi.status() != WL_CONNECTED) {               // Retry every 500 ms
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          delay(500);
        }
        digitalWrite(LED_PIN, HIGH);
        MQTT_connect();
      }
    }
  }
}


bool MQTT_connect()
{
  int8_t ret;
  if (mqtt.connected()) {                    // Stop if already connected.
    digitalWrite(LED_PIN, HIGH);
    return true;
  }
  digitalWrite(LED_PIN, LOW);
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {     // connect will return 0 for connected
    mqtt.disconnect();
    delay(5000);                            // Retry after 5 seconds
    retries--;
    if (retries == 0) {
      return false;
    }
  }
  digitalWrite(LED_PIN, HIGH);
  return true;
}
