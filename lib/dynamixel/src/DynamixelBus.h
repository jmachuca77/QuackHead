// QuackHead Dynamixel Bus - high-level API wrapping the ROBOTIS protocol library.
// Provides the same interface style as PuddleDuck's DynamixelBus.
#pragma once

#include "DynamixelInternal.h"
#include "TeensyHalfDuplexPort.h"

namespace DYNAMIXEL {

class DynamixelBus {
 public:
  DynamixelBus(HardwareSerial& serial, uint8_t txen_pin)
      : port_(serial, txen_pin), internal_(nullptr) {}

  void begin(uint32_t baud) {
    port_.begin(baud);
    internal_ = new DynamixelInternal(&port_);
    internal_->setPortProtocolVersionUsingIndex(2);
  }

  bool hasError() {
    return internal_ && internal_->getLastLibErrCode() != DXL_LIB_OK;
  }

  void clearError() {
    if (internal_) internal_->clearError();
  }

  bool ping(uint8_t id) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->ping(id);
  }

  bool torqueOn(uint8_t id) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->torqueOn(id);
  }

  bool torqueOff(uint8_t id) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->torqueOff(id);
  }

  bool ledOn(uint8_t id) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->ledOn(id);
  }

  bool ledOff(uint8_t id) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->ledOff(id);
  }

  bool setOperatingMode(uint8_t id, DxlOperatingMode mode) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setOperatingMode(id, (uint8_t)mode);
  }

  bool setReturnDelayTime(uint8_t id, uint8_t ms) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setReturnDelayTime(id, ms);
  }

  bool setPositionPGain(uint8_t id, int32_t value) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setPositionPGain(id, value);
  }

  bool setPositionIGain(uint8_t id, int32_t value) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setPositionIGain(id, value);
  }

  bool setPositionDGain(uint8_t id, int32_t value) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setPositionDGain(id, value);
  }

  int32_t getPositionPGain(uint8_t id) {
    if (!internal_) return 0;
    internal_->clearError();
    return internal_->getPositionPGain(id);
  }

  int32_t getPositionIGain(uint8_t id) {
    if (!internal_) return 0;
    internal_->clearError();
    return internal_->getPositionIGain(id);
  }

  int32_t getPositionDGain(uint8_t id) {
    if (!internal_) return 0;
    internal_->clearError();
    return internal_->getPositionDGain(id);
  }

  bool setProfileAcceleration(uint8_t id, uint32_t value) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setProfileAcceleration(id, value);
  }

  bool setProfileVelocity(uint8_t id, uint32_t value) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setProfileVelocity(id, value);
  }

  bool setGoalPosition(uint8_t id, float value, ParamUnit unit = UNIT_DEGREE) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setGoalPosition(id, value, unit);
  }

  float getPresentPosition(uint8_t id, ParamUnit unit = UNIT_DEGREE) {
    if (!internal_) return 0.0f;
    internal_->clearError();
    return internal_->getPresentPosition(id, unit);
  }

  bool setGoalVelocity(uint8_t id, float value, ParamUnit unit = UNIT_RPM) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setGoalVelocity(id, value, unit);
  }

  bool setGoalCurrent(uint8_t id, float value, ParamUnit unit = UNIT_MILLI_AMPERE) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->setGoalCurrent(id, value, unit);
  }

  // Raw write for bulk operations (goal position as raw ticks)
  bool writeRaw(uint8_t id, uint16_t addr, const uint8_t* data, uint16_t len) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->write(id, addr, data, len, 10);
  }

  // Write with no response expected (broadcast-style)
  bool writeNoResp(uint8_t id, uint16_t addr, const uint8_t* data, uint16_t len) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->writeNoResp(id, addr, data, len);
  }

  // Sync write support
  bool syncWrite(InfoSyncWriteInst_t* p_info) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->syncWrite(p_info);
  }

  // Bulk write support
  bool bulkWrite(InfoBulkWriteInst_t* p_info) {
    if (!internal_) return false;
    internal_->clearError();
    return internal_->bulkWrite(p_info);
  }

  DynamixelInternal* getInternal() { return internal_; }

 private:
  TeensyHalfDuplexPort port_;
  DynamixelInternal* internal_;
};

}  // namespace DYNAMIXEL
