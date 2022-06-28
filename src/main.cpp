#include <Arduino.h>
#include <WiFi.h>
#include <M5Stack.h>

int val_capteur = 0;
int last_val_capteur = 0;
long timer_1 = 0;
long timer_2 = 0;
long timer_3 = 0;
bool send = 0;
int counter = 0;
bool internet_state = 0;
bool mqtt_state = 0;

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include "Adafruit_TCS34725.h"

#define WIFI_SSID "LucasR"
#define WIFI_PASSWORD "lucasromary3"

#define MQTT_HOST IPAddress(192, 168, 213, 60)
//#define MQTT_HOST "mqtt.ci-ciad.utbm.fr"
#define MQTT_PORT 1883

// a MODIFIER
#define MQTT_TOPIC "M5Stack/color/counter"

#define commonAnode true // set to false if using a common cathode LED.  //如果使用普通阴极LED，则设置为false

byte gammatable[256]; // our RGB -> eye-recognized gamma color

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
int status = WL_IDLE_STATUS;

float r, g, b, last_r = 0;

void connectToWifi()
{
  Serial.print("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  /*
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  */
  internet_state = 1;
  Serial.println("");
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    internet_state = 0;
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  mqtt_state = 1;
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString("Internet : ", 0, 60, 4);
  M5.Lcd.drawString(WiFi.localIP().toString(), 200, 60, 4);
  M5.Lcd.drawString("MQTT : ", 0, 90, 4);
  M5.Lcd.drawString(String(mqtt_state), 200, 90, 4);
  M5.Lcd.drawString("Compteur : ", 0, 120, 4);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  mqtt_state = 0;
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString("Internet : ", 0, 60, 4);
  M5.Lcd.drawString(WiFi.localIP().toString(), 200, 60, 4);
  M5.Lcd.drawString("MQTT : ", 0, 90, 4);
  M5.Lcd.drawString(String(mqtt_state), 200, 90, 4);
  M5.Lcd.drawString("Compteur : ", 0, 120, 4);

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  /*
  String messageTemp;

  for (int i = 0; i < len; i++)
  {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  Serial.print("  message: ");
  Serial.println(messageTemp);
  Serial.println();

  if (String(topic) == MQTT_TOPIC && messageTemp == "1")
  {
    relay_on = 1;
    Serial.println("ON");
  }
  if (String(topic) == MQTT_TOPIC && messageTemp == "0")
  {
    relay_on = 0;
    Serial.println("Off");
  }
  */
}

void onMqttPublish(uint16_t packetId)
{

  // Serial.println("Publish acknowledged.");
}

static uint16_t color16(uint16_t r, uint16_t g, uint16_t b)
{
  uint16_t _color;
  _color = (uint16_t)(r & 0xF8) << 8;
  _color |= (uint16_t)(g & 0xFC) << 3;
  _color |= (uint16_t)(b & 0xF8) >> 3;
  return _color;
}

void setup()
{
  M5.begin();       // Init M5Core.  初始化 M5Core
  M5.Power.begin(); // Init power  初始化电源模块
  Serial.begin(115200);
  pinMode(22, INPUT_PULLUP);

  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString("Internet : ", 0, 60, 4);
  M5.Lcd.drawString(String(internet_state), 200, 60, 4);
  M5.Lcd.drawString("MQTT : ", 0, 90, 4);
  M5.Lcd.drawString(String(mqtt_state), 200, 90, 4);
  M5.Lcd.drawString("Couleur : ", 0, 120, 4);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString("Internet : ", 0, 60, 4);
  M5.Lcd.drawString(WiFi.localIP().toString(), 200, 60, 4);
  M5.Lcd.drawString("MQTT : ", 0, 90, 4);
  M5.Lcd.drawString(String(mqtt_state), 200, 90, 4);
  M5.Lcd.drawString("Couleur : ", 0, 120, 4);

  while (!tcs.begin())
  { //如果color unit未能初始化
    Serial.println("No TCS34725 found ... check your connections");
    M5.Lcd.drawString("No Found sensor.", 200, 120, 4);
    delay(1000);
  }
  tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_154MS); // Sets the integration time for the TC34725.  设置TC34725的集成时间
  tcs.setGain(TCS34725_GAIN_4X);                          // Adjusts the gain on the TCS34725.  调整TCS34725上的增益
}

void loop()
{
  uint16_t clear, red, green, blue;
  uint32_t sum = clear;

  tcs.getRawData(&red, &green, &blue, &clear); // Reads the raw red, green, blue and clear channel values.  读取原始的红、绿、蓝和清晰的通道值

  r = red;
  r /= sum;
  g = green;
  g /= sum;
  b = blue;
  b /= sum;
  r *= 256;
  g *= 256;
  b *= 256;

  timer_1 = millis();

  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextDatum(CC_DATUM);
  M5.Lcd.drawString("Internet : ", 0, 60, 4);
  M5.Lcd.drawString(WiFi.localIP().toString(), 200, 60, 4);
  M5.Lcd.drawString("MQTT : ", 0, 90, 4);
  M5.Lcd.drawString(String(mqtt_state), 200, 90, 4);
  M5.Lcd.drawString("Couleur : ", 0, 120, 4);
  M5.Lcd.drawString(String(r), 200, 120, 4);
  M5.Lcd.drawString("Compteur : ", 0, 150, 4);
  M5.Lcd.drawString(String(counter), 200, 150, 4);
  delay(250);

  if (r > 120 && last_r < 120 && timer_1 - timer_2 > 1000)
  {
    counter++;
    mqttClient.publish((MQTT_TOPIC), 0, false, String(counter).c_str());
    timer_2 = millis();
  }
}