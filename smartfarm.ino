#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "time.h"
#include <ESP32Servo.h>

// ==== Cảm biến ====
#define DHTPIN 4
#define DHTTYPE DHT11
#define SOIL_A0 34
#define LDR_A0 35
#define MQ135_A0 32
#define RAIN_D0 33   // Cảm biến mưa (D0)

// ==== Relay ====
#define RELAY_PUMP 25  // Bơm nước (IN1)
#define RELAY_FAN1 26  // Quạt 1 (IN2)
#define RELAY_FAN2 27  // Quạt 2 (IN3)

// ==== Servo ====
#define SERVO_PIN 14
Servo myServo;
int posCenter = 90;
int offset = 120;
bool lastRainState = HIGH; // HIGH = không mưa

// ==== UART Gửi Lệnh Đèn ====
HardwareSerial ledSerial(2); 
#define TX2_PIN 17
#define RX2_PIN 16

// ==== WiFi & MQTT ====
const char* ssid = "Tony";
const char* password = "hoilamchi";
const char* mqtt_server = "13.239.43.66";

WiFiClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== NTP ====
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;

// ==== Manual Mode ====
unsigned long lastManualTime = 0;
const unsigned long MANUAL_TIMEOUT = 10000;

// ==== MQTT Callback ====
void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) message += (char)payload[i];
  message.trim();

  Serial.print("MQTT Command -> ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);

  if (String(topic) == "farm/relay/pump") {
    lastManualTime = millis();
    if (message == "1") {
      digitalWrite(RELAY_PUMP, LOW);
      Serial.println("Pump ON (manual)");
    } else {
      digitalWrite(RELAY_PUMP, HIGH);
      Serial.println("Pump OFF (manual)");
    }
  }

  if (String(topic) == "farm/relay/fan") {
    lastManualTime = millis();
    if (message == "1") {
      digitalWrite(RELAY_FAN1, LOW);
      digitalWrite(RELAY_FAN2, LOW);
      Serial.println("Fan ON (manual)");
    } else {
      digitalWrite(RELAY_FAN1, HIGH);
      digitalWrite(RELAY_FAN2, HIGH);
      Serial.println("Fan OFF (manual)");
    }
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  lcd.init();
  lcd.backlight();

  pinMode(SOIL_A0, INPUT);
  pinMode(LDR_A0, INPUT);
  pinMode(MQ135_A0, INPUT);
  pinMode(RAIN_D0, INPUT_PULLUP);

  pinMode(RELAY_PUMP, OUTPUT);
  pinMode(RELAY_FAN1, OUTPUT);
  pinMode(RELAY_FAN2, OUTPUT);
  digitalWrite(RELAY_PUMP, HIGH);
  digitalWrite(RELAY_FAN1, HIGH);
  digitalWrite(RELAY_FAN2, HIGH);

  // Servo setup
  myServo.attach(SERVO_PIN);
  myServo.write(posCenter);

  // UART cho LED
  ledSerial.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);

  Serial.println("Hệ thống sẵn sàng!");
  Serial.println("Chờ tín hiệu từ cảm biến mưa...");

  // WiFi
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  lcd.clear();
  lcd.print("WiFi Connected");

  // MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Connecting MQTT...");
    lcd.setCursor(0, 1);
    lcd.print("MQTT connecting ");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected MQTT");
      lcd.clear();
      lcd.print("MQTT Connected");
      client.subscribe("farm/relay/pump");
      client.subscribe("farm/relay/fan");
    } else {
      delay(2000);
    }
  }
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return;
  char timeString[9];
  strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);
  lcd.setCursor(0, 0);
  lcd.print(timeString);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  int soilAnalog = analogRead(SOIL_A0);
  int ldrValue = analogRead(LDR_A0);
  int gasAnalog = analogRead(MQ135_A0);
  int currentRainState = digitalRead(RAIN_D0);

  float soilPercent = map(soilAnalog, 0, 4095, 100, 0);
  float lightPercent = map(ldrValue, 0, 4095, 100, 0);
  float gasLevel = map(gasAnalog, 0, 4095, 0, 100);  // CO₂ nồng độ %

  // === Điều khiển LED qua Arduino ===
  if (lightPercent < 20) {
    ledSerial.println("ON");
  } else {
    ledSerial.println("OFF");
  }

  // Servo phản ứng với mưa
  if (currentRainState == LOW && lastRainState == HIGH) {
    Serial.println("🌧 Bat dau co mua!");
    myServo.write(posCenter - offset);
    delay(300);
    myServo.write(posCenter);
  }
  if (currentRainState == HIGH && lastRainState == LOW) {
    Serial.println("☀ Het mua!");
    myServo.write(posCenter + offset);
    delay(300);
    myServo.write(posCenter);
  }
  lastRainState = currentRainState;

  // === AUTO MODE ===
  bool manualActive = (millis() - lastManualTime < MANUAL_TIMEOUT);
  if (!manualActive) {
    if (soilPercent < 40)
      digitalWrite(RELAY_PUMP, LOW);
    else if (soilPercent >= 50)
      digitalWrite(RELAY_PUMP, HIGH);

    if (t > 31) {
      digitalWrite(RELAY_FAN1, LOW);
      digitalWrite(RELAY_FAN2, LOW);
    } else if (t < 31) {
      digitalWrite(RELAY_FAN1, HIGH);
      digitalWrite(RELAY_FAN2, HIGH);
    }
  }

  // Publish dữ liệu
  char msg[10];
  sprintf(msg, "%.1f", h); client.publish("farm/hum", msg);
  sprintf(msg, "%.1f", t); client.publish("farm/temp", msg);
  sprintf(msg, "%.1f", soilPercent); client.publish("farm/soil", msg);
  sprintf(msg, "%.1f", lightPercent); client.publish("farm/light", msg);
  sprintf(msg, "%.1f", gasLevel); client.publish("farm/gas", msg);

  // === LCD hiển thị ===
  lcd.clear();
  printLocalTime();
  lcd.setCursor(0, 1);
  lcd.printf("T:%.1f H:%.1f", t, h);
  delay(1500);

  lcd.clear();
  lcd.printf("Soil:%.0f%% L:%.0f%%", soilPercent, lightPercent);
  delay(1500);

  lcd.clear();
  lcd.printf("CO2: %.0f%%", gasLevel);  // <-- HIỂN THỊ NỒNG ĐỘ CO2
  delay(1500);
}
