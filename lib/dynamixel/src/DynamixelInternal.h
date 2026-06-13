// DynamixelInternal for QuackHead - adapted from PuddleDuck's ROBOTIS library.
// Removes pd::Bus dependency, uses DXLPortHandler directly.
#pragma once

#include "internal/master.h"
#include "internal/actuator.h"
#include "internal/config.h"

namespace DYNAMIXEL {

enum ParamUnit {
  UNIT_RAW = 0,
  UNIT_PERCENT,
  UNIT_RPM,
  UNIT_DEGREE,
  UNIT_MILLI_AMPERE
};

enum DxlOperatingMode {
  OP_CURRENT = 0,
  OP_VELOCITY = 1,
  OP_POSITION = 3,
  OP_EXTENDED_POSITION = 4,
  OP_CURRENT_BASED_POSITION = 5,
  OP_PWM = 16,
};

enum DxlFunctions {
  SET_ID,
  SET_BAUD_RATE,
  SET_PROTOCOL,
  SET_POSITION,
  GET_POSITION,
  SET_VELOCITY,
  GET_VELOCITY,
  SET_PWM,
  GET_PWM,
  SET_CURRENT,
  GET_CURRENT,
  LAST_DUMMY_FUNC = 0xFF
};

enum D2ALibErrorCode {
  D2A_LIB_ERROR_NULLPTR_PORT_HANDLER = 0x0040,
  D2A_LIB_ERROR_NOT_SUPPORT_FUNCTION,
  D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER
};

class DynamixelInternal : public Master {
 public:
  DynamixelInternal(DXLPortHandler* port, uint16_t packet_buf_size = DEFAULT_DXL_BUF_LENGTH);

  void clearError();

  using Master::ping;
  bool ping(uint8_t id = DXL_BROADCAST_ID);
  bool scan();

  bool setModelNumber(uint8_t id, uint16_t model_number);
  uint16_t getModelNumber(uint8_t id);

  bool setID(uint8_t id, uint8_t new_id);
  bool setProtocol(uint8_t id, uint8_t version);
  bool setBaudrate(uint8_t id, uint32_t baudrate);

  bool torqueOn(uint8_t id);
  bool torqueOff(uint8_t id);
  bool ledOn(uint8_t id);
  bool ledOff(uint8_t id);

  bool setOperatingMode(uint8_t id, uint8_t mode);
  bool setReturnDelayTime(uint8_t id, uint8_t ms);

  bool setHomingRawOffset(uint8_t id, int32_t value);
  int32_t getHomingRawOffset(uint8_t id);

  int32_t getPositionPGain(uint8_t id);
  bool setPositionPGain(uint8_t id, int32_t value);
  int32_t getPositionIGain(uint8_t id);
  bool setPositionIGain(uint8_t id, int32_t value);
  int32_t getPositionDGain(uint8_t id);
  bool setPositionDGain(uint8_t id, int32_t value);

  bool setProfileAcceleration(uint8_t id, uint32_t value);
  bool setProfileVelocity(uint8_t id, uint32_t value);

  float getMinPositionLimit(uint8_t id, ParamUnit unit = UNIT_DEGREE);
  float getMaxPositionLimit(uint8_t id, ParamUnit unit = UNIT_DEGREE);

  bool setGoalPosition(uint8_t id, float value, ParamUnit unit = UNIT_DEGREE);
  float getPresentPosition(uint8_t id, ParamUnit unit = UNIT_DEGREE);

  bool setGoalVelocity(uint8_t id, float value, ParamUnit unit = UNIT_RPM);
  float getPresentVelocity(uint8_t id, ParamUnit unit = UNIT_RPM);

  bool setGoalCurrent(uint8_t id, float value, ParamUnit unit = UNIT_MILLI_AMPERE);
  float getPresentCurrent(uint8_t id, ParamUnit unit = UNIT_MILLI_AMPERE);

  bool setGoalPWM(uint8_t id, float value, ParamUnit unit = UNIT_RAW);
  float getPresentPWM(uint8_t id, ParamUnit unit = UNIT_RAW);

  float getTorqueConstant(uint8_t id);
  float getTorqueNm(uint8_t id, float current_mA);
  int32_t getCurrentLimit(uint8_t id);

  bool getTorqueEnableStat(uint8_t id);

  int32_t readControlTableItem(uint8_t item_idx, uint8_t id, uint32_t timeout = 100);
  bool writeControlTableItem(uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout = 100);

  // Sync/Bulk operations (forwarded from Master)
  using Master::syncRead;
  using Master::syncWrite;
  using Master::bulkRead;
  using Master::bulkWrite;

 private:
  uint8_t model_number_idx_[254];
  uint8_t model_number_idx_last_index_;

  uint8_t getModelNumberIndex(uint16_t model_num);
  uint16_t getModelNumberFromTable(uint8_t id);

  bool setTorqueEnable(uint8_t id, bool enable);
  bool setLedState(uint8_t id, bool state);

  float readForRangeDependencyFunc(uint8_t func_idx, uint8_t id, ParamUnit unit);
  bool writeForRangeDependencyFunc(uint8_t func_idx, uint8_t id, float value, ParamUnit unit);

  int32_t readControlTableItem(uint16_t model_num, uint8_t item_idx, uint8_t id, uint32_t timeout = 100);
  bool writeControlTableItem(uint16_t model_num, uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout = 100);

  bool convertForRangeRead(uint8_t func_idx, uint8_t id, int32_t in_data, float &out_data, ParamUnit unit);
  bool convertForRangeWrite(uint8_t func_idx, uint8_t id, float in_data, int32_t &out_data, ParamUnit unit);
};

}  // namespace DYNAMIXEL
