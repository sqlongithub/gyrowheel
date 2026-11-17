#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define RADIO_CHANNEL 4
#define RADIO_RATE WIFI_PHY_RATE_6M

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

const bool DEBUG_TEXT = false;

uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Packet latest = {0, 0, 0, 0, 0};
volatile bool fresh = false;
unsigned long lastDebug = 0;
uint8_t cmdId = 0;

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(Packet)) {
    portENTER_CRITICAL(&mux);
    memcpy(&latest, data, sizeof(Packet));
    fresh = true;
    portEXIT_CRITICAL(&mux);
  }
}

void sendCmd(char cmd, float value) {
  Cmd c;
  c.cmd = cmd;
  c.id = ++cmdId;
  c.magic = CMD_MAGIC;
  c.value = value;
  for (int i = 0; i < 3; i++) {
    esp_now_send(broadcastMac, (uint8_t *)&c, sizeof(c));
    delay(2);
  }
}

void pollSerialCommands() {
  static char buf[24];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) {
        buf[n] = 0;
        sendCmd(buf[0], (float)atof(buf + 1));
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}

void setup() {
  Serial.begin(921600);
  delay(500);
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

  if (esp_now_init() != ESP_OK) {
    while (true) {
      Serial.println("# ESP-NOW init failed");
      delay(1000);
    }
  }
  esp_now_register_recv_cb(onReceive);
  esp_wifi_config_espnow_rate(WIFI_IF_STA, RADIO_RATE);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);
  peer.channel = 0;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  pollSerialCommands();

  if (!fresh) return;

  Packet p;
  portENTER_CRITICAL(&mux);
  p = latest;
  fresh = false;
  portEXIT_CRITICAL(&mux);

  char line[48];
  int n;
  if (DEBUG_TEXT) {
    if (millis() - lastDebug < 33) return;
    lastDebug = millis();
    n = snprintf(line, sizeof(line), "angle %.2f seq %u\n", p.angle / 100.0f, (unsigned)p.seq);
  } else {
    n = snprintf(line, sizeof(line), "%ld,%u,%u,%u\n", (long)p.angle, (unsigned)p.buttons, (unsigned)p.brake, (unsigned)p.throttle);
  }
  Serial.write((uint8_t *)line, n);
}