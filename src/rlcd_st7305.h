#pragma once

#include <Adafruit_GFX.h>
#include <stddef.h>
#include <stdint.h>
#include <SPI.h>

static constexpr uint16_t RLCD_WHITE = 0;
static constexpr uint16_t RLCD_BLACK = 1;

class RlcdSt7305 : public Adafruit_GFX {
public:
  static constexpr int16_t WIDTH = 400;
  static constexpr int16_t HEIGHT = 300;
  static constexpr size_t FB_SIZE = 15000;

  RlcdSt7305();

  bool begin();
  bool ready() const;
  void clear(uint16_t color);
  void display();
  void drawPixel(int16_t x, int16_t y, uint16_t color) override;

private:
  static constexpr int PIN_SCK = 11;
  static constexpr int PIN_MOSI = 12;
  static constexpr int PIN_DC = 5;
  static constexpr int PIN_CS = 40;
  static constexpr int PIN_RST = 41;
  static constexpr uint32_t SPI_HZ = 24000000;

  SPIClass spi_;
  uint8_t* fb_ = nullptr;
  bool ready_ = false;

  void sendCommand(uint8_t command);
  void sendData(const uint8_t* data, size_t length);
  void sendCommandData(uint8_t command, const uint8_t* data, size_t length);
  void resetPanel();
  void initPanel();
};
