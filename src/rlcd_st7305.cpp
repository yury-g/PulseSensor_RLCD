#include "rlcd_st7305.h"

#include <Arduino.h>
#include <cstring>
#include <esp_heap_caps.h>

RlcdSt7305::RlcdSt7305() : Adafruit_GFX(WIDTH, HEIGHT), spi_(HSPI) {
}

bool RlcdSt7305::begin() {
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_RST, HIGH);

  fb_ = static_cast<uint8_t*>(heap_caps_malloc(FB_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (fb_ == nullptr) {
    fb_ = static_cast<uint8_t*>(malloc(FB_SIZE));
  }
  if (fb_ == nullptr) {
    ready_ = false;
    return false;
  }

  clear(RLCD_WHITE);
  spi_.begin(PIN_SCK, -1, PIN_MOSI, -1);
  spi_.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  initPanel();
  display();
  ready_ = true;
  return true;
}

bool RlcdSt7305::ready() const {
  return ready_;
}

void RlcdSt7305::clear(uint16_t color) {
  if (fb_ == nullptr) return;
  memset(fb_, color == RLCD_BLACK ? 0x00 : 0xFF, FB_SIZE);
}

void RlcdSt7305::display() {
  if (fb_ == nullptr) return;
  const uint8_t winX[] = {0x12, 0x2A};
  const uint8_t winY[] = {0x00, 0xC7};
  sendCommandData(0x2A, winX, sizeof(winX));
  sendCommandData(0x2B, winY, sizeof(winY));
  sendCommand(0x2C);
  sendData(fb_, FB_SIZE);
}

void RlcdSt7305::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if (fb_ == nullptr || x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;

  int invY = HEIGHT - 1 - y;
  int index = (x / 2) * (HEIGHT / 4) + (invY / 4);
  int bit = 7 - ((invY % 4) * 2 + (x % 2));
  uint8_t mask = 1 << bit;

  if (color == RLCD_BLACK) {
    fb_[index] &= ~mask;
  } else {
    fb_[index] |= mask;
  }
}

void RlcdSt7305::sendCommand(uint8_t command) {
  digitalWrite(PIN_DC, LOW);
  digitalWrite(PIN_CS, LOW);
  spi_.transfer(command);
  digitalWrite(PIN_CS, HIGH);
}

void RlcdSt7305::sendData(const uint8_t* data, size_t length) {
  digitalWrite(PIN_DC, HIGH);
  digitalWrite(PIN_CS, LOW);
  spi_.transferBytes(const_cast<uint8_t*>(data), nullptr, length);
  digitalWrite(PIN_CS, HIGH);
}

void RlcdSt7305::sendCommandData(uint8_t command, const uint8_t* data, size_t length) {
  digitalWrite(PIN_DC, LOW);
  digitalWrite(PIN_CS, LOW);
  spi_.transfer(command);
  if (length > 0) {
    digitalWrite(PIN_DC, HIGH);
    spi_.transferBytes(const_cast<uint8_t*>(data), nullptr, length);
  }
  digitalWrite(PIN_CS, HIGH);
}

void RlcdSt7305::resetPanel() {
  digitalWrite(PIN_RST, HIGH);
  delay(50);
  digitalWrite(PIN_RST, LOW);
  delay(20);
  digitalWrite(PIN_RST, HIGH);
  delay(50);
}

void RlcdSt7305::initPanel() {
  resetPanel();

  const uint8_t d6[] = {0x17, 0x02};
  const uint8_t d1[] = {0x01};
  const uint8_t c0[] = {0x11, 0x04};
  const uint8_t c1[] = {0x69, 0x69, 0x69, 0x69};
  const uint8_t c2[] = {0x19, 0x19, 0x19, 0x19};
  const uint8_t c4[] = {0x4B, 0x4B, 0x4B, 0x4B};
  const uint8_t d8[] = {0x80, 0xE9};
  const uint8_t b2[] = {0x02};
  const uint8_t b3[] = {0xE5, 0xF6, 0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45};
  const uint8_t b4[] = {0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76, 0x45};
  const uint8_t gTiming[] = {0x32, 0x03, 0x1F};
  const uint8_t b7[] = {0x13};
  const uint8_t b0[] = {0x64};
  const uint8_t c9[] = {0x00};
  const uint8_t m36[] = {0x48};
  const uint8_t m3a[] = {0x11};
  const uint8_t b9[] = {0x20};
  const uint8_t b8[] = {0x29};
  const uint8_t winX[] = {0x12, 0x2A};
  const uint8_t winY[] = {0x00, 0xC7};
  const uint8_t m35[] = {0x00};
  const uint8_t d0[] = {0xFF};

  sendCommandData(0xD6, d6, sizeof(d6));
  sendCommandData(0xD1, d1, sizeof(d1));
  sendCommandData(0xC0, c0, sizeof(c0));
  sendCommandData(0xC1, c1, sizeof(c1));
  sendCommandData(0xC2, c2, sizeof(c2));
  sendCommandData(0xC4, c4, sizeof(c4));
  sendCommandData(0xC5, c2, sizeof(c2));
  sendCommandData(0xD8, d8, sizeof(d8));
  sendCommandData(0xB2, b2, sizeof(b2));
  sendCommandData(0xB3, b3, sizeof(b3));
  sendCommandData(0xB4, b4, sizeof(b4));
  sendCommandData(0x62, gTiming, sizeof(gTiming));
  sendCommandData(0xB7, b7, sizeof(b7));
  sendCommandData(0xB0, b0, sizeof(b0));
  sendCommand(0x11);
  delay(120);
  sendCommandData(0xC9, c9, sizeof(c9));
  sendCommandData(0x36, m36, sizeof(m36));
  sendCommandData(0x3A, m3a, sizeof(m3a));
  sendCommandData(0xB9, b9, sizeof(b9));
  sendCommandData(0xB8, b8, sizeof(b8));
  sendCommand(0x21);
  sendCommandData(0x2A, winX, sizeof(winX));
  sendCommandData(0x2B, winY, sizeof(winY));
  sendCommandData(0x35, m35, sizeof(m35));
  sendCommandData(0xD0, d0, sizeof(d0));
  sendCommand(0x38);
  sendCommand(0x29);
}
