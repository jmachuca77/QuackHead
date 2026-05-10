#pragma once

#include <Arduino.h>

namespace qh4 {
// Teensy 4.1 net labels from the QuackHead4 schematic.
constexpr uint8_t kSerial1RxPin = 0;   // RX1  -> RS485 channel 1 RO
constexpr uint8_t kSerial1TxPin = 1;   // TX1  -> RS485 channel 1 DI
constexpr uint8_t kDinAudioPin = 7;    // DIN  -> MAX98357 I2S input
constexpr uint8_t kSerial3TxPin = 14;  // TX3  -> RS485 channel 2 DI
constexpr uint8_t kSerial3RxPin = 15;  // RX3  -> RS485 channel 2 RO
constexpr uint8_t kSerial4RxPin = 16;  // RX4
constexpr uint8_t kSerial4TxPin = 17;  // TX4
constexpr uint8_t kLrclkPin = 20;      // LRCLK
constexpr uint8_t kBclkPin = 21;       // BCLK
constexpr uint8_t kSerial6TxPin = 24;  // TX6
constexpr uint8_t kSerial6RxPin = 25;  // RX6
constexpr uint8_t kMosiPin = 11;
constexpr uint8_t kSckPin = 13;
constexpr uint8_t kSerial7RxPin = 28;  // RX7  -> TTL bridge 1 RX
constexpr uint8_t kSerial7TxPin = 29;  // TX7  -> TTL bridge 1 TX
constexpr uint8_t kTXEN = 32;          // TXEN  -> TTL bridge 1 direction control
constexpr uint8_t kTXEN2 = 33;         // TXEN2 -> TTL bridge 2 direction control
constexpr uint8_t kSerial8RxPin = 34;  // RX8  -> TTL bridge 2 RX
constexpr uint8_t kSerial8TxPin = 35;  // TX8  -> TTL bridge 2 TX
constexpr uint8_t kEyeLeftPin = 36;    // shared with LEFTEYE SPI CS
constexpr uint8_t kEyeRightPin = 37;   // shared with RIGHTEYE SPI CS
constexpr uint8_t kTorchLedPin = 38;   // LED
constexpr uint8_t kLcdDcPin = 39;      // LCD_DC
constexpr uint8_t kLcdRstPin = 40;     // LCD_RST
constexpr uint8_t kLcdBlPin = 41;      // LCD_BL

constexpr uint32_t kDebugBaud = 115200;
constexpr uint32_t kRs485TestBaud = 4000000;
}  // namespace qh4
