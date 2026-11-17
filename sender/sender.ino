#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "BMI088.h"

#define RADIO_CHANNEL 4
#define RADIO_RATE WIFI_PHY_RATE_6M
#define I2C_HZ 400000
#define ACCEL_EVERY 4

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct __attribute__((packed)) Packet {
  int32_t angle;
  uint16_t buttons;
  uint16_t seq;
  uint8_t brake;
  uint8_t throttle;
};

struct __attribute__((packed)) Cmd {
  char cmd;
  uint8_t id;
  uint16_t magic;
  float value;
};
const uint16_t CMD_MAGIC = 0xC0DE;

Packet pkt = {0, 0, 0, 0, 0};
volatile char pendingCmd = 0;
volatile float pendingVal = 0;
volatile uint32_t failCount = 0;
uint8_t lastCmdId = 0;

BMI088 bmi088(BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS);

const float RAD2DEG = 57.2958f;
const float TAU = 20.0f;
const float ACC_SIGN = -1.0f;
const float KI = 1.0f / (4.0f * TAU * TAU);
const float MAX_ANGLE = 1000.0f;
const unsigned long SEND_US = 1000;
const unsigned long PRINT_MS = 33;
const bool DEBUG_PRINT = false;

const int ACCEL_TOGGLE_PIN = 4;
const bool HARDWARE_TOGGLE_ENABLE = false;

const int BRAKE_PIN = 34;
const int THROTTLE_PIN = 33;
const bool THROTTLE_ACTIVE_HIGH = true;
const bool BRAKE_ACTIVE_HIGH = false;

bool useAccel = true;
bool invert = false;
float scale = 1.0f;
float deadband = 1.5f;
bool lastTrust = false;

float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

float angle = 0;
float angleOffset = 0;
float gyroBias = 0;
float stillTime = 0;
float rateLp = 0;
unsigned long lastUs = 0;
unsigned long lastSendUs = 0;
unsigned long lastPrint = 0;
uint8_t accelCount = 0;

bool lastPinState = HIGH;
unsigned long lastPinDebounce = 0;

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) failCount++;
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(Cmd)) return;
  Cmd c;
  memcpy(&c, data, sizeof(Cmd));
  if (c.magic != CMD_MAGIC) return;
  if (c.id == lastCmdId) return;
  lastCmdId = c.id;
  pendingVal = c.value;
  pendingCmd = c.cmd;
}

void calibrateGyro() {
  const int WINDOWS = 5;
  const int PER = 300;
  for (int attempt = 0; attempt < 10; attempt++) {
    Serial.println("Calibrating, keep still");
    float total = 0, wmin = 1e9f, wmax = -1e9f;
    for (int w = 0; w < WINDOWS; w++) {
      float sum = 0;
      for (int i = 0; i < PER; i++) {
        bmi088.getGyroscope(&gx, &gy, &gz);
        sum += gz;
        delay(2);
      }
      float m = sum / PER;
      total += m;
      if (m < wmin) wmin = m;
      if (m > wmax) wmax = m;
    }
    gyroBias = total / WINDOWS;
    if (wmax - wmin < 1.5f) break;
    Serial.println("Moved during calibration, retrying");
  }

  if (useAccel) {
    float sx = 0, sy = 0;
    for (int i = 0; i < 10; i++) {
      bmi088.getAcceleration(&ax, &ay, &az);
      sx += ax;
      sy += ay;
      delay(10);
    }
    angle = ACC_SIGN * atan2f(sy, sx) * RAD2DEG;
  } else {
    angle = 0;
  }

  angleOffset = angle;
  stillTime = 0;
  rateLp = 0;
  lastUs = micros();
  Serial.print("Gyro bias: ");
  Serial.println(gyroBias, 3);
}

void setupRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_wifi_set_ps(WIFI_PS_NONE);

  wifi_country_t country;
  memset(&country, 0, sizeof(country));
  strcpy(country.cc, "DK");
  country.schan = 1;
  country.nchan = 13;
  country.max_tx_power = 20;
  country.policy = WIFI_COUNTRY_POLICY_MANUAL;
  esp_wifi_set_country(&country);

  esp_wifi_set_max_tx_power(84);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  uint8_t primary;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.print("Channel: ");
  Serial.println(primary);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);
  esp_err_t r = esp_wifi_config_espnow_rate(WIFI_IF_STA, RADIO_RATE);
  Serial.println(r == ESP_OK ? "Rate ok" : "Rate failed");

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  Serial.println(esp_now_add_peer(&peer) == ESP_OK ? "Peer ok" : "Peer failed");
}

void applyCommand(char c, float v) {
  switch (c) {
    case 'z':
      angleOffset = angle;
      Serial.println("Zeroed");
      break;
    case 'c':
      calibrateGyro();
      break;
    case 'a':
      useAccel = !useAccel;
      Serial.println(useAccel ? "Mode: Accel + Gyro Fusion" : "Mode: Gyro Only");
      break;
    case 'i':
      invert = !invert;
      Serial.println(invert ? "Inverted" : "Not inverted");
      break;
    case 's':
      scale = constrain(v, 0.5f, 2.0f);
      break;
    case 'd':
      deadband = constrain(v, 0.0f, 20.0f);
      break;
  }
}

void handleInputs() {
  char rc = pendingCmd;
  if (rc) {
    float rv = pendingVal;
    pendingCmd = 0;
    applyCommand(rc, rv);
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') continue;
    applyCommand(c, 0);
  }

  if (HARDWARE_TOGGLE_ENABLE && ACCEL_TOGGLE_PIN >= 0) {
    bool currentPinState = digitalRead(ACCEL_TOGGLE_PIN);
    if (currentPinState != lastPinState && (millis() - lastPinDebounce) > 50) {
      lastPinDebounce = millis();
      lastPinState = currentPinState;
      if (currentPinState == LOW) {
        useAccel = !useAccel;
        Serial.println(useAccel ? "Hardware Toggle: Accel + Gyro" : "Hardware Toggle: Gyro Only");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (HARDWARE_TOGGLE_ENABLE && ACCEL_TOGGLE_PIN >= 0) {
    pinMode(ACCEL_TOGGLE_PIN, INPUT_PULLUP);
    lastPinState = digitalRead(ACCEL_TOGGLE_PIN);
  }

  pinMode(BRAKE_PIN, INPUT);
  pinMode(THROTTLE_PIN, THROTTLE_ACTIVE_HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);

  Wire.begin(21, 22);
  Wire.setClock(I2C_HZ);

  while (!bmi088.isConnection()) {
    Serial.println("BMI088 is not connected");
    delay(1000);
  }
  bmi088.initialize();
  delay(100);

  calibrateGyro();
  setupRadio();
  Serial.println("Commands: z = zero, c = recalibrate, a = toggle accel/gyro mode");
}

void loop() {
  handleInputs();

  bmi088.getGyroscope(&gx, &gy, &gz);

  if (useAccel && ++accelCount >= ACCEL_EVERY) {
    accelCount = 0;
    bmi088.getAcceleration(&ax, &ay, &az);
  }

  unsigned long now = micros();
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;
  if (dt > 0.05f) dt = 0.05f;

  float rate = gz - gyroBias;

  rateLp += (rate - rateLp) * (dt / (0.25f + dt));
  if (fabsf(rateLp) < 0.7f) {
    stillTime += dt;
    if (stillTime > 1.0f) {
      gyroBias += (gz - gyroBias) * (dt / 20.0f);
    }
  } else {
    stillTime = 0;
  }

  float predicted = angle + rate * dt;

  bool trust = false;
  if (useAccel) {
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    float xy = sqrtf(ax * ax + ay * ay);
    trust = fabsf(mag - 1000.0f) < 80.0f &&
            xy > 250.0f &&
            fabsf(rate) < 20.0f;
  }

  if (trust) {
    float accAngle = ACC_SIGN * atan2f(ay, ax) * RAD2DEG;
    float diff = accAngle - predicted;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    if (fabsf(rate) < 5.0f && fabsf(diff) < 10.0f) {
      gyroBias -= KI * diff * dt;
    }
    angle = predicted + (dt / (TAU + dt)) * diff;
  } else {
    angle = predicted;
  }

  lastTrust = trust;

  float steer = (angle - angleOffset) * scale;
  if (invert) steer = -steer;
  if (steer > deadband) steer -= deadband;
  else if (steer < -deadband) steer += deadband;
  else steer = 0;
  if (steer > MAX_ANGLE) steer = MAX_ANGLE;
  if (steer < -MAX_ANGLE) steer = -MAX_ANGLE;

  if (now - lastSendUs >= SEND_US) {
    lastSendUs += SEND_US;
    if (now - lastSendUs >= SEND_US) lastSendUs = now;
    bool brakePressed = (digitalRead(BRAKE_PIN) == HIGH) == BRAKE_ACTIVE_HIGH;
    pkt.brake = brakePressed ? 255 : 0;
    bool thrPressed = (digitalRead(THROTTLE_PIN) == HIGH) == THROTTLE_ACTIVE_HIGH;
    pkt.throttle = thrPressed ? 255 : 0;
    pkt.angle = (int32_t)roundf(steer * 100.0f);
    pkt.buttons = (useAccel ? 0x4000 : 0) | (lastTrust ? 0x8000 : 0) | (invert ? 0x2000 : 0);
    pkt.seq++;
    if (esp_now_send(broadcastMac, (uint8_t *)&pkt, sizeof(pkt)) != ESP_OK) failCount++;
  }

  if (DEBUG_PRINT && millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    Serial.print("steer ");
    Serial.print(steer, 2);
    Serial.print("\tmode ");
    Serial.print(useAccel ? "ACC+GYRO" : "GYRO_ONLY");
    Serial.print("\tbias ");
    Serial.print(gyroBias, 3);
    Serial.print("\ttrust ");
    Serial.print(trust ? 1 : 0);
    Serial.print("\tbrake ");
    Serial.print(pkt.brake);
    Serial.print("\tthr ");
    Serial.print(pkt.throttle);
    Serial.print("\tfailed ");
    Serial.println(failCount);
  }
}