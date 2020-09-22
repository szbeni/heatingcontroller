/**
    Required libraries:
      - Adafruit BME280 Library
      - Adafruit Unified Sensor
      - PubSubClient
**/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define MQTT_TOPIC_HUMIDITY "caravan/heatingcontrol/sensor/humidity"
#define MQTT_TOPIC_TEMPERATURE "caravan/heatingcontrol/sensor/temperature"
#define MQTT_TOPIC_PRESSURE "caravan/heatingcontrol/sensor/pressure"
#define MQTT_TOPIC_ALTITUDE "caravan/heatingcontrol/sensor/altitude"
#define MQTT_TOPIC_INPUT  "caravan/heatingcontrol/input"
#define MQTT_TOPIC_STATE "caravan/heatingcontrol/status"
#define MQTT_TOPIC_COMMAND "caravan/heatingcontrol/cmd/#"

#define MQTT_PUBLISH_INPUT_DELAY 10000
#define MQTT_PUBLISH_SENSOR_DELAY 10000
#define MQTT_CLIENT_ID "heatingcontrol"

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_ADDRESS 0x76

#define OUTPUT_ON_SELECT    D4
#define OUTPUT_OFF          D3
#define BUTTON_PRESS_DELAY  50


#define INPUT_CHECK_DELAY 500
#define INPUT_NUM           5
#define INPUT_GAS_AUTO_FAN  D5
#define INPUT_GAS_SLOW_FAN  D6
#define INPUT_FAN           D7
#define INPUT_ELEC_AUTO_FAN D8
#define INPUT_ELEC_SLOW_FAN D0

const char *WIFI_SSID = "ABWifi";
const char *WIFI_PASSWORD = "Secret_12345";

const char *MQTT_SERVER = "192.168.3.1";
const char *MQTT_USER = "caravan"; // NULL for no authentication
const char *MQTT_PASSWORD = "Fafafa123$"; // NULL for no authentication

float humidity;
float temperature;
float pressure;
float altitude;
long lastSensorMsgTime = 0;
long lastInputMsgTime = 0;
long lastInputCheckTime = 0;


int input[5];
int input_prev[5];

Adafruit_BME280 bme;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup() {
  pinMode(OUTPUT_ON_SELECT, OUTPUT);
  pinMode(OUTPUT_OFF, OUTPUT);
  
  pinMode(INPUT_GAS_AUTO_FAN, INPUT_PULLUP);
  pinMode(INPUT_GAS_SLOW_FAN, INPUT);
  pinMode(INPUT_FAN, INPUT);
  pinMode(INPUT_ELEC_SLOW_FAN, INPUT);
  pinMode(INPUT_ELEC_AUTO_FAN, INPUT);
  
  Serial.begin(115200);
  while (! Serial);

  if (!bme.begin(BME280_ADDRESS)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring or BME-280 address!");
    while (1);
  }

  // Use force mode so that the sensor returns to sleep mode when the measurement is finished
  //bme.setSampling(Adafruit_BME280::MODE_FORCED,
  //                Adafruit_BME280::SAMPLING_X1, // temperature
  //Adafruit_BME280::SAMPLING_X1, // pressure
  //                Adafruit_BME280::SAMPLING_X1, // humidity
  //                Adafruit_BME280::FILTER_OFF);
  //  

  setupWifi();
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqttOnMessage);
}

void loop() {
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();
  sensorLoop();
  heatingControlLoop();

}

void sensorLoop() {
  long now = millis();
  // Sensor readings
  if (now - lastSensorMsgTime > MQTT_PUBLISH_SENSOR_DELAY) {
    lastSensorMsgTime = now;

    // Reading BME280 sensor data
    //bme.takeForcedMeasurement(); // has no effect in normal mode
    humidity = bme.readHumidity();
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    
    if (isnan(humidity) || isnan(temperature)|| isnan(pressure)) {
      Serial.println("BME280 reading issues");
      return;
    }

    // Publishing sensor data
    mqttPublish(MQTT_TOPIC_TEMPERATURE, temperature);
    mqttPublish(MQTT_TOPIC_HUMIDITY, humidity);
    mqttPublish(MQTT_TOPIC_PRESSURE, pressure);
    mqttPublish(MQTT_TOPIC_ALTITUDE, altitude);
  }

}

int test;
int test_prev;

void publishInputs()
{
  lastInputMsgTime = millis();
  String msg = "";
  for(int i =0; i<INPUT_NUM;i++) 
  {
    if(input[i] == HIGH)
      msg += "0";
    else 
      msg += "1";
  }
  mqttClient.publish(MQTT_TOPIC_INPUT, msg.c_str(), true);
}

void heatingControlLoop() {
  long now = millis();
  
  if (now - lastInputMsgTime > MQTT_PUBLISH_INPUT_DELAY) {
    publishInputs();
  }
  
  if (now - lastInputCheckTime > INPUT_CHECK_DELAY) {
    lastInputCheckTime = now;
  }
  else {
    return;
  }

  input[0] = digitalRead(INPUT_GAS_AUTO_FAN);
  input[1] = digitalRead(INPUT_GAS_SLOW_FAN);
  input[2] = digitalRead(INPUT_FAN);
  input[3] = digitalRead(INPUT_ELEC_AUTO_FAN);
  input[4] = digitalRead(INPUT_ELEC_SLOW_FAN);
  
  int changed = 0;
  for(int i=0;i<INPUT_NUM;i++){
    if (input_prev[i] != input[i]) 
    {
      input_prev[i] = input[i];
      changed = 1;
    }
  }
  if (changed == 1)
  {
    publishInputs();
  }
}

void setupWifi() {
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATE, 1, true, "disconnected", false)) {
      Serial.println("connected");

      // Once connected, publish an announcement...
      mqttClient.publish(MQTT_TOPIC_STATE, "connected", true);
      mqttClient.subscribe(MQTT_TOPIC_COMMAND);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void mqttPublish(char *topic, float payload) {
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(payload);

  mqttClient.publish(topic, String(payload).c_str(), true);
}


void mqttOnMessage(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();


  if (strcmp(topic,"button")>0)
  {
    if ((char)payload[0] == '0') {  
      digitalWrite(OUTPUT_ON_SELECT, HIGH);
      delay(BUTTON_PRESS_DELAY);
      digitalWrite(OUTPUT_ON_SELECT, LOW);
    }
    else if ((char)payload[0] == '1') {  
      digitalWrite(OUTPUT_OFF, HIGH);
      delay(BUTTON_PRESS_DELAY);
      digitalWrite(OUTPUT_OFF, LOW);
    }
  }
}
