// Teensy half-duplex serial port handler for DYNAMIXEL Protocol 2.0 bus.
// Wraps HardwareSerial + TXEN direction pin.
#pragma once

#include "internal/port_handler.h"
#include <Arduino.h>

namespace DYNAMIXEL {

class TeensyHalfDuplexPort : public DXLPortHandler {
 public:
  TeensyHalfDuplexPort(HardwareSerial& serial, uint8_t txen_pin)
      : serial_(serial), txen_pin_(txen_pin) {}

  void begin(uint32_t baud) {
    pinMode(txen_pin_, OUTPUT);
    digitalWrite(txen_pin_, LOW);  // RX mode
    serial_.begin(baud);
    setOpenState(true);
  }

  void begin() override {
    setOpenState(true);
  }

  void end() override {
    serial_.end();
    setOpenState(false);
  }

  int available() override {
    return serial_.available();
  }

  int read() override {
    return serial_.read();
  }

  size_t write(uint8_t b) override {
    setTxMode();
    size_t n = serial_.write(b);
    serial_.flush();
    setRxMode();
    return n;
  }

  size_t write(uint8_t* buf, size_t len) override {
    setTxMode();
    size_t n = serial_.write(buf, len);
    serial_.flush();
    setRxMode();
    return n;
  }

 private:
  void setTxMode() {
    digitalWrite(txen_pin_, HIGH);
    delayMicroseconds(2);
  }

  void setRxMode() {
    delayMicroseconds(2);
    digitalWrite(txen_pin_, LOW);
  }

  HardwareSerial& serial_;
  uint8_t txen_pin_;
};

}  // namespace DYNAMIXEL
