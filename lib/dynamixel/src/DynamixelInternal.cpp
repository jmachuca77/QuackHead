/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "DynamixelInternal.h"
#include <Arduino.h>
#include <math.h>

#ifndef PROGMEM
#define PROGMEM
#endif

// Stub out PuddleDuck logging macros
#define PDLOG_ERROR(...) do {} while(0)
#define PDLOG_INFO(...) do {} while(0)
#define PDLOG_ERROR_THROTTLED(...) do {} while(0)

namespace DYNAMIXEL {

static const uint16_t model_number_table[] PROGMEM = {
    AX12A, AX12W, AX18A,
    
    RX10, RX24F, RX28, RX64,
    
    DX113, DX116, DX117,
    
    EX106,

    MX12W,  MX28,   MX64,    MX106,
    MX28_2, MX64_2, MX106_2,
    
    XL320,
    XL330_M288,
    XL330_M077,
    XC330_M181,
    XC330_M288,    
    XC330_T181,
    XC330_T288,    
    XL430_W250,
    XXL430_W250,
    XC430_W150,  XC430_W240,
    XXC430_W250,
    XM430_W210,  XM430_W350,
    XM540_W150,  XM540_W270, 
    XH430_V210,  XH430_V350, XH430_W210, XH430_W350,
    XH540_V150,  XH540_V270, XH540_W150, XH540_W270,
    XD430_T210,  XD430_T350,
    XD540_T150,  XD540_T270,
    XW430_T200,  XW430_T333,
    XW540_T140,  XW540_T260,

    PRO_L42_10_S300_R,   
    PRO_L54_30_S400_R,   PRO_L54_30_S500_R,   PRO_L54_50_S290_R,   PRO_L54_50_S500_R,
    PRO_M42_10_S260_R,   PRO_M42_10_S260_RA,
    PRO_M54_40_S250_R,   PRO_M54_40_S250_RA,  PRO_M54_60_S250_R,   PRO_M54_60_S250_RA,
    PRO_H42_20_S300_R,   PRO_H42_20_S300_RA,
    PRO_H54_100_S500_R,  PRO_H54_100_S500_RA, PRO_H54_200_S500_R,  PRO_H54_200_S500_RA,

    PRO_M42P_010_S260_R, 
    PRO_M54P_040_S250_R, PRO_M54P_060_S250_R,
    PRO_H42P_020_S300_R, 
    PRO_H54P_100_S500_R, PRO_H54P_200_S500_R
};

static const uint8_t model_number_table_count = sizeof(model_number_table)/sizeof(model_number_table[0]);

} //namespace DYNAMIXEL

using namespace DYNAMIXEL;

typedef struct ModelDependencyFuncItemAndRangeInfo{
    uint8_t func_idx; //enum Functions
    uint8_t item_idx; //enum ControlTableItem
    uint8_t unit_type; //enum ParamUnit
    int32_t min_value;
    int32_t max_value;
    float unit_value;
} ModelDependencyFuncItemAndRangeInfo_t;

typedef struct ItemAndRangeInfo{
    uint8_t item_idx; //enum ControlTableItem
    uint8_t unit_type; //enum ParamUnit
    int32_t min_value;
    int32_t max_value;
    float unit_value;
} ItemAndRangeInfo_t;

static ItemAndRangeInfo_t getModelDependencyFuncInfo(uint16_t model_num, uint8_t func_num);
static float f_map(float x, float in_min, float in_max, float out_min, float out_max);
static bool checkAndconvertWriteData(float in_data, int32_t &out_data, ParamUnit unit, ItemAndRangeInfo_t &item_info);
static bool checkAndconvertReadData(int32_t in_data, float &out_data, ParamUnit unit, ItemAndRangeInfo_t &item_info);

DynamixelInternal::DynamixelInternal(DXLPortHandler* port, uint16_t packet_buf_size) :
    Master(2.0, packet_buf_size), model_number_idx_last_index_(0)
{
    setPort(port);
    memset(&model_number_idx_, 0xff, sizeof(model_number_idx_));
}

void
DynamixelInternal::clearError() {
    setLastLibErrCode(DXL_LIB_OK);
}

bool
DynamixelInternal::scan() {
    return ping();
}

bool
DynamixelInternal::ping(uint8_t id) {
    bool ret = false;
    if (id != DXL_BROADCAST_ID) {
        InfoFromPing_t recv_info;
    #ifdef ARDUINO
        constexpr uint32_t timeout = 10;
    #else
        constexpr uint32_t timeout = 100;
    #endif
        if (Master::ping(id, &recv_info, 1, timeout) > 0) {
            if (recv_info.id == id) {
                if (getPortProtocolVersion() == 1.0) {
                    recv_info.model_number = getModelNumber(id);
                }
                ret = setModelNumber(id, recv_info.model_number);
            }
        }
    } else {
        uint8_t recv_ids[254];
        uint8_t recv_cnt = Master::ping(DXL_BROADCAST_ID, recv_ids, sizeof(recv_ids), 3*253);
        if (recv_cnt > 0) {
            for (uint8_t i=0; i<recv_cnt; i++) {
                (void)setModelNumber(recv_ids[i], getModelNumber(id));
            }
            ret = true;
        }
    }
    return ret;  
}

bool 
DynamixelInternal::setModelNumber(uint8_t id, uint16_t model_number) {
    bool ret = false;

    if (id <= 253) {
        model_number_idx_[id] = getModelNumberIndex(model_number);
        ret = (model_number_idx_[id] != 0xFF) ? true : false;
        if (ret == false) {
            setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
        }
    } else {
        setLastLibErrCode(DXL_LIB_ERROR_INVALID_ID);
    }
    return ret;
}

uint16_t
DynamixelInternal::getModelNumber(uint8_t id) {
    uint16_t model_num = 0xFFFF;

    (void)read(id, COMMON_MODEL_NUMBER_ADDR, COMMON_MODEL_NUMBER_ADDR_LENGTH,
                (uint8_t*)&model_num, sizeof(model_num), 20);
    return model_num;
}

bool
DynamixelInternal::setID(uint8_t id, uint8_t new_id) {
    return writeControlTableItem(ControlTableItem::ID, id, new_id);
}

bool
DynamixelInternal::setProtocol(uint8_t id, uint8_t version) {
    if (version >= 1 && version <= 2) {
        return writeControlTableItem(ControlTableItem::PROTOCOL_VERSION, id, version);
    }
    setLastLibErrCode(DXL_LIB_ERROR_INVALID_PROTOCOL_VERSION);
    return false;
}

//TODO: Simplify the code by grouping model numbers.
bool
DynamixelInternal::setBaudrate(uint8_t id, uint32_t baudrate) {
    uint16_t model_num = getModelNumberFromTable(id);
    uint8_t baud_idx = 0;

    switch(model_num) {
        case AX12A:
        case AX12W:
        case AX18A:
        case DX113:
        case DX116:
        case DX117:
        case RX10:
        case RX24F:
        case RX28:
        case RX64:
        case EX106:    
        case MX12W:
        case MX28:
        case MX64:
        case MX106:
            switch(baudrate) {
                case 9600:
                    baud_idx = 207;
                    break;
                case 57600:
                    baud_idx = 34;
                    break;
                case 115200:
                    baud_idx = 16;
                    break;
                case 1000000:
                    baud_idx = 1;
                    break;
                default:
                    return false;                    
            }
            break;

        case XL320:
            switch(baudrate) {
                case 9600:
                    baud_idx = 0;
                    break;
                case 57600:
                    baud_idx = 1;
                    break;
                case 115200:
                    baud_idx = 2;
                    break;
                case 1000000:
                    baud_idx = 3;
                    break;
                default:
                    return false;                    
            }    
            break;
        case XC330_M288:
        case XC330_M181:
        case XC330_T288:
        case XC330_T181:
        case XL330_M288:
        case XL330_M077:
            switch(baudrate) {
                case 9600:
                    baud_idx = 0;
                    break;
                case 57600:
                    baud_idx = 1;
                    break;
                case 115200:
                    baud_idx = 2;
                    break;
                case 1000000:
                    baud_idx = 3;
                    break;
                case 2000000:
                    baud_idx = 4;
                    break;
                case 3000000:
                    baud_idx = 5;
                    break;
                case 4000000:
                    baud_idx = 6;
                    break;
                default:
                    return false;                    
            }    
            break;

        case MX28_2:
        case MX64_2:
        case MX106_2:
        case XC430_W150:
        case XC430_W240:
        case XXC430_W250:
        case XL430_W250:
        case XXL430_W250:
        case XM430_W210:
        case XM430_W350:
        case XH430_V210:
        case XH430_V350:
        case XH430_W210:
        case XH430_W350:
        case XD430_T210:
        case XD430_T350:
        case XM540_W150:
        case XM540_W270:
        case XH540_W150:
        case XH540_W270:
        case XH540_V150:
        case XH540_V270:
        case XD540_T150:
        case XD540_T270:
        case XW430_T200:
        case XW430_T333:
        case XW540_T140:
        case XW540_T260:    
            switch(baudrate) {
                case 9600:
                    baud_idx = 0;
                    break;
                case 57600:
                    baud_idx = 1;
                    break;
                case 115200:
                    baud_idx = 2;
                    break;
                case 1000000:
                    baud_idx = 3;
                    break;
                case 2000000:
                    baud_idx = 4;
                    break;
                case 3000000:
                    baud_idx = 5;
                    break;
                case 4000000:
                    baud_idx = 6;
                    break;
                case 4500000:
                    baud_idx = 7;
                    break;    
                default:
                    return false;          
            }
            break;

        // case PRO_L42_10_S300_R:
        // case PRO_L54_30_S400_R:
        // case PRO_L54_30_S500_R:
        // case PRO_L54_50_S290_R:
        // case PRO_L54_50_S500_R:
        case PRO_M42_10_S260_R:
        case PRO_M54_40_S250_R:
        case PRO_M54_60_S250_R:
        case PRO_H42_20_S300_R:
        case PRO_H54_100_S500_R:
        case PRO_H54_200_S500_R:
            switch(baudrate) {
                case 9600:
                    baud_idx = 0;
                    break;
                case 57600:
                    baud_idx = 1;
                    break;
                case 115200:
                    baud_idx = 2;
                    break;
                case 1000000:
                    baud_idx = 3;
                    break;
                case 2000000:
                    baud_idx = 4;
                    break;
                case 3000000:
                    baud_idx = 5;
                    break;
                case 4000000:
                    baud_idx = 6;
                    break;
                case 4500000:
                    baud_idx = 7;
                    break;
                case 10500000:
                    baud_idx = 8;
                    break;
                default:
                    return false;          
            }
            break;

        case PRO_M42_10_S260_RA:
        case PRO_M54_40_S250_RA:
        case PRO_M54_60_S250_RA:
        case PRO_H42_20_S300_RA:
        case PRO_H54_100_S500_RA:
        case PRO_H54_200_S500_RA:
        case PRO_H42P_020_S300_R:
        case PRO_H54P_100_S500_R:
        case PRO_H54P_200_S500_R:
        case PRO_M42P_010_S260_R:
        case PRO_M54P_040_S250_R:
        case PRO_M54P_060_S250_R:
            switch(baudrate) {
                case 9600:
                    baud_idx = 0;
                    break;
                case 57600:
                    baud_idx = 1;
                    break;
                case 115200:
                    baud_idx = 2;
                    break;
                case 1000000:
                    baud_idx = 3;
                    break;
                case 2000000:
                    baud_idx = 4;
                    break;
                case 3000000:
                    baud_idx = 5;
                    break;
                case 4000000:
                    baud_idx = 6;
                    break;
                case 4500000:
                    baud_idx = 7;
                    break;
                case 6000000:
                    baud_idx = 8;
                    break;          
                case 10500000:
                    baud_idx = 9;
                    break;          
                default:
                    return false;          
            }                
            break;
        default:
            return false;
    }
    return writeControlTableItem(ControlTableItem::BAUD_RATE, id, baud_idx);
}

/* Commands for Slave */
bool
DynamixelInternal::torqueOn(uint8_t id) {
    return setTorqueEnable(id, true);
}

bool
DynamixelInternal::torqueOff(uint8_t id) {
    return setTorqueEnable(id, false);
}

bool
DynamixelInternal::setTorqueEnable(uint8_t id, bool enable) {
    uint16_t model_num = getModelNumberFromTable(id);
    auto res = writeControlTableItem(ControlTableItem::TORQUE_ENABLE, id, enable);
    switch (model_num) {
        case XL330_M288:
        case XL330_M077:
        case XC330_M288:
        case XC330_M181:    
        case XC330_T181:
        case XC330_T288:
            // At least the XC330 needs a delay after changing the torque
            // state or immediate commands may be missed.
            delayMicroseconds(1000);
            break;
    }
    return res;
}

bool
DynamixelInternal::ledOn(uint8_t id) {
    return setLedState(id, true);
}

bool
DynamixelInternal::ledOff(uint8_t id) {
    return setLedState(id, false);
}

bool
DynamixelInternal::setLedState(uint8_t id, bool state) {
    bool ret = false;
    uint16_t model_num = getModelNumberFromTable(id);

    switch(model_num) {
        // case PRO_L42_10_S300_R:
        // case PRO_L54_30_S400_R:
        // case PRO_L54_30_S500_R:
        // case PRO_L54_50_S290_R:
        // case PRO_L54_50_S500_R:
        case PRO_M42_10_S260_R:
        case PRO_M54_40_S250_R:
        case PRO_M54_60_S250_R:
        case PRO_H42_20_S300_R:
        case PRO_H54_100_S500_R:
        case PRO_H54_200_S500_R:
        case PRO_M42_10_S260_RA:
        case PRO_M54_40_S250_RA:
        case PRO_M54_60_S250_RA:
        case PRO_H42_20_S300_RA:
        case PRO_H54_100_S500_RA:
        case PRO_H54_200_S500_RA:
        case PRO_H42P_020_S300_R:
        case PRO_H54P_100_S500_R:
        case PRO_H54P_200_S500_R:
        case PRO_M42P_010_S260_R:
        case PRO_M54P_040_S250_R:
        case PRO_M54P_060_S250_R:
            if (state == false) {
                writeControlTableItem(ControlTableItem::LED_GREEN, id, state);
                writeControlTableItem(ControlTableItem::LED_BLUE, id, state);
            }
            ret = writeControlTableItem(ControlTableItem::LED_RED, id, state);
            break;

        default:
            ret = writeControlTableItem(ControlTableItem::LED, id, state);
            break;
    }
    return ret;
}

bool DynamixelInternal::setReturnDelayTime(uint8_t id, uint8_t ms) {
    if (readControlTableItem(ControlTableItem::RETURN_DELAY_TIME, id) != ms) {
        return writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, id, ms);
    }
    return true;
}

bool
DynamixelInternal::setHomingRawOffset(uint8_t id, int32_t offset) {
    if (readControlTableItem(ControlTableItem::HOMING_OFFSET, id) != offset) {
        return writeControlTableItem(ControlTableItem::HOMING_OFFSET, id, offset);
    }
    return true;
}

int32_t
DynamixelInternal::getHomingRawOffset(uint8_t id) {
    return readControlTableItem(ControlTableItem::HOMING_OFFSET, id);
}

int32_t
DynamixelInternal::getPositionPGain(uint8_t id) {
    return readControlTableItem(ControlTableItem::POSITION_P_GAIN, id);
}

int32_t
DynamixelInternal::getPositionIGain(uint8_t id) {
    return readControlTableItem(ControlTableItem::POSITION_I_GAIN, id);
}

int32_t
DynamixelInternal::getPositionDGain(uint8_t id) {
    return readControlTableItem(ControlTableItem::POSITION_D_GAIN, id);
}

bool
DynamixelInternal::setPositionPGain(uint8_t id, int32_t offset) {
    if (readControlTableItem(ControlTableItem::POSITION_P_GAIN, id) != offset) {
        return writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, offset);
    }
    return true;
}

bool
DynamixelInternal::setPositionIGain(uint8_t id, int32_t offset) {
    if (readControlTableItem(ControlTableItem::POSITION_I_GAIN, id) != offset) {
        return writeControlTableItem(ControlTableItem::POSITION_I_GAIN, id, offset);
    }
    return true;
}

bool
DynamixelInternal::setPositionDGain(uint8_t id, int32_t offset) {
    if (readControlTableItem(ControlTableItem::POSITION_D_GAIN, id) != offset) {
        return writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, offset);
    }
    return true;
}

/**
 * @brief Return the min position limit for the actuator.
 * @param id DYNAMIXEL Actuator's ID.
 * @return It returns min position limit for the actuator.
 * If the read fails, 0 is returned. Whether or not this is an actual value can be confirmed with @getLastLibErrCode().
 */
float
DynamixelInternal::getMinPositionLimit(uint8_t id, ParamUnit unit) {
    float ret = 0;
    int32_t raw = readControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, id);
    convertForRangeRead(GET_POSITION, id, raw, ret, unit);
    return ret;
}

/**
 * @brief Return the max position limit for the actuator.
 * @param id DYNAMIXEL Actuator's ID.
 * @return It returns max position limit for the actuator.
 * If the read fails, 0 is returned. Whether or not this is an actual value can be confirmed with @getLastLibErrCode().
 */
float
DynamixelInternal::getMaxPositionLimit(uint8_t id, ParamUnit unit) {
    float ret = 0;
    int32_t raw = readControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, id);
    convertForRangeRead(GET_POSITION, id, raw, ret, unit);
    return ret;
}

bool
DynamixelInternal::setProfileAcceleration(uint8_t id, uint32_t value) {
    return writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, (int32_t)value);
}

bool
DynamixelInternal::setProfileVelocity(uint8_t id, uint32_t value) {
    return writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, (int32_t)value);
}

//TODO: Simplify the code by grouping model numbers.
bool DynamixelInternal::setOperatingMode(uint8_t id, uint8_t mode) {
    bool ret = false;
    uint16_t model_num = getModelNumberFromTable(id);

    switch(model_num) {
        case AX12A:
        case AX12W:
        case AX18A:
        case DX113:
        case DX116:
        case DX117:
        case RX10:
        case RX24F:
        case RX28:
        case RX64:
            if (mode == OP_POSITION) {
                if (writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 1023);
            } else if (mode == OP_VELOCITY) {
                if (writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
            }
            break;

        case EX106:
            if (mode == OP_POSITION) {
                if (writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
            } else if (mode == OP_VELOCITY) {
                if (writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
            }
            break;

        case XL320:
            if (mode == OP_POSITION) {
                ret = writeControlTableItem(ControlTableItem::CONTROL_MODE, id, 2);
            } else if (mode == OP_VELOCITY) {
                ret = writeControlTableItem(ControlTableItem::CONTROL_MODE, id, 1);
            }
            break;

        case MX12W:
        case MX28:
            if (mode == OP_POSITION) {
                if (writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
            } else if (mode == OP_VELOCITY) {
                if (writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
            } else if (mode == OP_EXTENDED_POSITION) {
                if (writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 4095))
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
            }
            break;   

        case MX64:
        case MX106:
            if (mode == OP_POSITION) {
                if (writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0) ||
                    writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
                {
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
                }
            } else if (mode == OP_VELOCITY) {
                if (writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0) ||
                    writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 0))
                {
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 0);
                }
            } else if (mode == OP_EXTENDED_POSITION) {
                if (writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 0) ||
                    writeControlTableItem(ControlTableItem::CW_ANGLE_LIMIT, id, 4095))
                {
                    ret = writeControlTableItem(ControlTableItem::CCW_ANGLE_LIMIT, id, 4095);
                }
            } else if(mode == OP_CURRENT) {
                ret = writeControlTableItem(ControlTableItem::TORQUE_CTRL_MODE_ENABLE, id, 1);
            }
            break;

        case MX28_2:
        case XC430_W150:
        case XC430_W240:
        case XXC430_W250:
        case XL430_W250:
        case XXL430_W250: {
            int32_t opmode = -1;
            switch (mode) {
                case OP_POSITION:
                    opmode = 3;
                    break;
                case OP_VELOCITY:
                    opmode = 1;
                    break;
                case OP_EXTENDED_POSITION:
                    opmode = 4;
                    break;
                case OP_PWM:
                    opmode = 16;
                    break;
                default:
                    break;
            }
            if (opmode == -1) {
                /* unsupported mode */
                ret = false;
            } else if (readControlTableItem(ControlTableItem::OPERATING_MODE, id) != opmode) {
                ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, opmode);
            } else {
                ret = true;
            }
            break;
        }

        case MX64_2:
        case MX106_2:
        case XL330_M288:
        case XL330_M077:
        case XC330_M288:
        case XC330_M181:    
        case XC330_T181:
        case XC330_T288:    
        case XM430_W210:
        case XM430_W350:
        case XH430_V210:
        case XH430_V350:
        case XH430_W210:
        case XH430_W350:
        case XD430_T210:
        case XD430_T350:
        case XM540_W150:
        case XM540_W270:
        case XH540_W150:
        case XH540_W270:
        case XH540_V150:
        case XH540_V270:
        case XD540_T150:
        case XD540_T270:
        case XW430_T200:
        case XW430_T333:
        case XW540_T140:
        case XW540_T260: {
            int32_t opmode = -1;
            switch (mode) {
                case OP_POSITION:
                    opmode = 3;
                    break;
                case OP_VELOCITY:
                    opmode = 1;
                    break;
                case OP_EXTENDED_POSITION:
                    opmode = 4;
                    break;
                case OP_CURRENT:
                    opmode = 0;
                    break;
                case OP_PWM:
                    opmode = 16;
                    break;
                case OP_CURRENT_BASED_POSITION:
                    opmode = 5;
                    break;
                default:
                    break;
            }
            if (opmode == -1) {
                /* unsupported mode */
                ret = false;
            } else if (readControlTableItem(ControlTableItem::OPERATING_MODE, id) != opmode) {
                ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, opmode);
            } else {
                ret = true;
            }
            break;
        }    

        // case PRO_L42_10_S300_R:
        // case PRO_L54_30_S400_R:
        // case PRO_L54_30_S500_R:
        // case PRO_L54_50_S290_R:
        // case PRO_L54_50_S500_R:
        case PRO_M42_10_S260_R:
        case PRO_M54_40_S250_R:
        case PRO_M54_60_S250_R:
        case PRO_H42_20_S300_R:
        case PRO_H54_100_S500_R:
        case PRO_H54_200_S500_R: {
            int32_t opmode = -1;
            switch (mode) {
                case OP_POSITION:
                    opmode = 3;
                    break;
                case OP_VELOCITY:
                    opmode = 1;
                    break;
                case OP_EXTENDED_POSITION:
                    opmode = 4;
                    break;
                case OP_CURRENT:
                    opmode = 0;
                    break;
                default:
                    break;
            }
            if (opmode == -1) {
                /* unsupported mode */
                ret = false;
            } else if (readControlTableItem(ControlTableItem::OPERATING_MODE, id) != opmode) {
                ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, opmode);
            } else {
                ret = true;
            }
            break;
        }

        case PRO_M42_10_S260_RA:
        case PRO_M54_40_S250_RA:
        case PRO_M54_60_S250_RA:
        case PRO_H42_20_S300_RA:
        case PRO_H54_100_S500_RA:
        case PRO_H54_200_S500_RA:
        case PRO_H42P_020_S300_R:
        case PRO_H54P_100_S500_R:
        case PRO_H54P_200_S500_R:
        case PRO_M42P_010_S260_R:
        case PRO_M54P_040_S250_R:
        case PRO_M54P_060_S250_R: {
            int32_t opmode = -1;
            switch (mode) {
                case OP_POSITION:
                    opmode = 3;
                    break;
                case OP_VELOCITY:
                    opmode = 1;
                    break;
                case OP_EXTENDED_POSITION:
                    opmode = 4;
                    break;
                case OP_CURRENT:
                    opmode = 0;
                    break;
                case OP_PWM:
                    opmode = 16;
                    break;
                default:
                    break;
            }
            if (opmode == -1) {
                /* unsupported mode */
                ret = false;
            } else if (readControlTableItem(ControlTableItem::OPERATING_MODE, id) != opmode) {
                ret = writeControlTableItem(ControlTableItem::OPERATING_MODE, id, opmode);
            } else {
                ret = true;
            }
            break;
        }

        default:
            break;
    }
    return ret;
}

bool
DynamixelInternal::setGoalPosition(uint8_t id, float value, ParamUnit unit) {
    if (unit != UNIT_RAW && unit != UNIT_DEGREE)
        return false;
    return writeForRangeDependencyFunc(SET_POSITION, id, value, unit);
}

float
DynamixelInternal::getPresentPosition(uint8_t id, ParamUnit unit) {
    if (unit != UNIT_RAW && unit != UNIT_DEGREE)
        return 0.0;
    return readForRangeDependencyFunc(GET_POSITION, id, unit);
}

bool
DynamixelInternal::setGoalVelocity(uint8_t id, float value, ParamUnit unit) {
    if (unit != UNIT_RAW && unit != UNIT_PERCENT && unit != UNIT_RPM)
        return false;
    return writeForRangeDependencyFunc(SET_VELOCITY, id, value, unit);
}

float
DynamixelInternal::getPresentVelocity(uint8_t id, ParamUnit unit) {
    if (unit != UNIT_RAW && unit != UNIT_PERCENT && unit != UNIT_RPM)
        return 0.0;
    return readForRangeDependencyFunc(GET_VELOCITY, id, unit);
}

bool
DynamixelInternal::setGoalPWM(uint8_t id, float value, ParamUnit unit) {
    if (unit != UNIT_RAW && unit != UNIT_PERCENT)
        return false;
    return writeForRangeDependencyFunc(SET_PWM, id, value, unit);
}

float
DynamixelInternal::getPresentPWM(uint8_t id, ParamUnit unit) {
    if (unit != UNIT_RAW && unit != UNIT_PERCENT)
        return 0.0;
    return readForRangeDependencyFunc(GET_PWM, id, unit);
}

bool
DynamixelInternal::setGoalCurrent(uint8_t id, float value, ParamUnit unit) {
    if (unit != UNIT_RAW && unit != UNIT_PERCENT && unit != UNIT_MILLI_AMPERE)
        return false;
    return writeForRangeDependencyFunc(SET_CURRENT, id, value, unit);
}

float
DynamixelInternal::getPresentCurrent(uint8_t id, ParamUnit unit) {
    if (unit != UNIT_RAW && unit != UNIT_PERCENT && unit != UNIT_MILLI_AMPERE)
        return 0.0;
    return readForRangeDependencyFunc(GET_CURRENT, id, unit);
}

bool
DynamixelInternal::getTorqueEnableStat(uint8_t id) {
    return (readControlTableItem(ControlTableItem::TORQUE_ENABLE, id) == DXL_TORQUE_ON);
}

int32_t
DynamixelInternal::readControlTableItem(uint8_t item_idx, uint8_t id, uint32_t timeout) {
    int32_t ret = 0;
    uint16_t model_num = getModelNumberFromTable(id);

    // To use the command function without ping() or model addition.
    if (model_num == UNREGISTERED_MODEL) {
        if (setModelNumber(id, getModelNumber(id)) == true) {
            model_num = getModelNumberFromTable(id);
        }
    }
    if (model_num != UNREGISTERED_MODEL) {
        ret = readControlTableItem(model_num, item_idx, id, timeout);
    } else {
        setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
    }
    return ret;
}

bool
DynamixelInternal::writeControlTableItem(uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout) {
    bool ret = false;
    uint16_t model_num = getModelNumberFromTable(id);

    // To use the command function without ping() or model addition.
    if (model_num == UNREGISTERED_MODEL) {
        if (setModelNumber(id, getModelNumber(id)) == true) {
            model_num = getModelNumberFromTable(id);
        }
    }
    if (model_num != UNREGISTERED_MODEL) {
        ret = writeControlTableItem(model_num, item_idx, id, data, timeout);
    } else {
        setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
    }
    return ret;
}

/* Private Member Function */

int32_t
DynamixelInternal::readControlTableItem(uint16_t model_num, uint8_t item_idx, uint8_t id, uint32_t timeout) {
    int32_t recv_len, ret = 0;
    ControlTableItemInfo_t item_info;

    if (getPort() == nullptr) {
        setLastLibErrCode(D2A_LIB_ERROR_NULLPTR_PORT_HANDLER);
        return 0;
    }

    item_info = getControlTableItemInfo(model_num, item_idx);
    if (item_info.addr_length > 0) {
        recv_len = read(id, item_info.addr, item_info.addr_length, (uint8_t*)&ret, sizeof(ret), timeout);
        if (recv_len == 1) {
            int8_t t_data = (int8_t)ret;
            ret = (int32_t)t_data;
        } else if (recv_len == 2) {
            int16_t t_data = (int16_t)ret;
            ret = (int32_t)t_data;
        }
    }
    return ret;
}

bool
DynamixelInternal::writeControlTableItem(uint16_t model_num, uint8_t item_idx, uint8_t id, int32_t data, uint32_t timeout) {
    bool ret = false;
    ControlTableItemInfo_t item_info;

    if (getPort() == nullptr) {
        setLastLibErrCode(D2A_LIB_ERROR_NULLPTR_PORT_HANDLER);
        return false;
    }

    item_info = getControlTableItemInfo(model_num, item_idx);
    if (item_info.addr_length > 0) {
        ret = write(id, item_info.addr, (uint8_t*)&data, item_info.addr_length, timeout);
    }
    return ret;
}

uint8_t
DynamixelInternal::getModelNumberIndex(uint16_t model_num) {
    uint8_t i, ret = 0xFF;

    // quick shortcut
    if (model_num == model_number_idx_[model_number_idx_last_index_]) {
        ret = model_number_idx_last_index_;
    } else {
        for (i=0; i<model_number_table_count; i++) {
            if (model_num == model_number_table[i]) {
                model_number_idx_last_index_ = i;
                ret = i;
                break;
            }
        }
    }
    return ret;
}

uint16_t
DynamixelInternal::getModelNumberFromTable(uint8_t id) {
    uint8_t idx;
    uint16_t model_num;

    if (id > 254) {
        setLastLibErrCode(DXL_LIB_ERROR_INVALID_ID);
        return UNREGISTERED_MODEL;
    }

    idx = model_number_idx_[id];
    model_num = (idx < model_number_table_count) ? model_number_table[idx] : UNREGISTERED_MODEL;
    return model_num;
}

float
DynamixelInternal::readForRangeDependencyFunc(uint8_t func_idx, uint8_t id, ParamUnit unit) {
    float ret = 0;
    int32_t recv_data = 0;
    uint16_t model_num = getModelNumberFromTable(id);
    ItemAndRangeInfo_t item_info;

    // To use the command function without ping() or model addition.
    if (model_num == UNREGISTERED_MODEL) {
        if (setModelNumber(id, getModelNumber(id)) == true) {
            model_num = getModelNumberFromTable(id);
        }
    }

    if (model_num != UNREGISTERED_MODEL) {
        item_info = getModelDependencyFuncInfo(model_num, func_idx);

        if (item_info.item_idx != ControlTableItem::LAST_DUMMY_ITEM) {
            recv_data = readControlTableItem(model_num, item_info.item_idx, id);
            checkAndconvertReadData(recv_data, ret, unit, item_info);
        } else {
            setLastLibErrCode(D2A_LIB_ERROR_NOT_SUPPORT_FUNCTION);
        }    
    } else {
        setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
    }
    return ret;
}

bool
DynamixelInternal::writeForRangeDependencyFunc(uint8_t func_idx, uint8_t id, float value, ParamUnit unit) {
    bool ret = false;
    int32_t data = 0;
    uint16_t model_num = getModelNumberFromTable(id);
    ItemAndRangeInfo_t item_info;

    // To use the command function without ping() or model addition.
    if (model_num == UNREGISTERED_MODEL) {
        if (setModelNumber(id, getModelNumber(id)) == true) {
            model_num = getModelNumberFromTable(id);
        }
    }

    if (model_num != UNREGISTERED_MODEL) {
        item_info = getModelDependencyFuncInfo(model_num, func_idx);
        if (item_info.item_idx == ControlTableItem::LAST_DUMMY_ITEM)
            return false;

        if (checkAndconvertWriteData(value, data, unit, item_info) == false)
            return false;
        ret = writeControlTableItem(model_num, item_info.item_idx, id, data);
    } else {
        setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
    }
    return ret;
}

bool
DynamixelInternal::convertForRangeRead(uint8_t func_idx, uint8_t id, int32_t in_data, float &out_data, ParamUnit unit) {
    uint16_t model_num = getModelNumberFromTable(id);
    ItemAndRangeInfo_t item_info;

    out_data = 0;
    if (model_num == UNREGISTERED_MODEL) {
        if (setModelNumber(id, getModelNumber(id)) == true) {
            model_num = getModelNumberFromTable(id);
        }
    }

    if (model_num != UNREGISTERED_MODEL) {
        item_info = getModelDependencyFuncInfo(model_num, func_idx);

        if (item_info.item_idx != ControlTableItem::LAST_DUMMY_ITEM) {
            if (checkAndconvertReadData(in_data, out_data, unit, item_info) == false)
                return false;

            return true;
        } else {
            setLastLibErrCode(D2A_LIB_ERROR_NOT_SUPPORT_FUNCTION);
        }    
    } else {
        setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
    }
    return false;
}

bool
DynamixelInternal::convertForRangeWrite(uint8_t func_idx, uint8_t id, float in_data, int32_t &out_data, ParamUnit unit) {
    uint16_t model_num = getModelNumberFromTable(id);
    ItemAndRangeInfo_t item_info;

    out_data = 0;
    if (model_num == UNREGISTERED_MODEL) {
        if (setModelNumber(id, getModelNumber(id)) == true) {
            model_num = getModelNumberFromTable(id);
        }
    }

    if (model_num != UNREGISTERED_MODEL) {
        item_info = getModelDependencyFuncInfo(model_num, func_idx);
        if (item_info.item_idx == ControlTableItem::LAST_DUMMY_ITEM)
            return false;
        if (checkAndconvertWriteData(in_data, out_data, unit, item_info) == false)
            return false;

        return true;
    } else {
        setLastLibErrCode(D2A_LIB_ERROR_UNKNOWN_MODEL_NUMBER);
    }
    return false;
}
/* Const structure & Static Function */
using namespace ControlTableItem;

/* AX series, XL320 (DX,RX,EX) */
static const ModelDependencyFuncItemAndRangeInfo_t dependency_ctable_1_0_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_AX || ENABLE_ACTUATOR_DX || \
     ENABLE_ACTUATOR_RX || ENABLE_ACTUATOR_EX)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 1023, 0.29},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, 0, 1023, 0.29},

    {SET_VELOCITY, MOVING_SPEED, UNIT_PERCENT, 0, 2047, 0.1},
    {GET_VELOCITY, PRESENT_SPEED, UNIT_PERCENT, 0, 2047, 0.1},  
#endif 
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_ex[] PROGMEM = {
#if (ENABLE_ACTUATOR_EX)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 4095, 0.06},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, 0, 4095, 0.06},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

/* MX(1.0) series */
static const ModelDependencyFuncItemAndRangeInfo_t dependency_ctable_1_1_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX12W || ENABLE_ACTUATOR_MX28 || \
     ENABLE_ACTUATOR_MX64 || ENABLE_ACTUATOR_MX106)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -28672, 28672, 0.088},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -28672, 28672, 0.088},

    {SET_VELOCITY, MOVING_SPEED, UNIT_RPM, 0, 2047, 0.114},
    {GET_VELOCITY, PRESENT_SPEED, UNIT_RPM, 0, 2047, 0.114},  
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_mx12[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX12W)
    {SET_VELOCITY, MOVING_SPEED, UNIT_RPM, 0, 2047, 0.916},
    {GET_VELOCITY, PRESENT_SPEED, UNIT_RPM, 0, 2047, 0.916},  
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_mx64_mx106[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX64 || ENABLE_ACTUATOR_MX106)
    {SET_CURRENT, GOAL_TORQUE, UNIT_MILLI_AMPERE, 0, 2047, 4.5},
    {GET_CURRENT, CURRENT, UNIT_MILLI_AMPERE, 0, 4095, 4.5},  
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xl320[] PROGMEM = {
#if (ENABLE_ACTUATOR_XL320)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, 0, 1023, 0.29},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, 0, 1023, 0.29},

    {SET_VELOCITY, MOVING_SPEED, UNIT_PERCENT, 0, 2047, 0.1},
    {GET_VELOCITY, PRESENT_SPEED, UNIT_PERCENT, 0, 2047, 0.1},  
#endif 
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

/* MX2.0, X series(without XL320) */
static const ModelDependencyFuncItemAndRangeInfo_t dependency_ctable_2_0_common[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX28_PROTOCOL2  || \
     ENABLE_ACTUATOR_MX64_PROTOCOL2  || \
     ENABLE_ACTUATOR_MX106_PROTOCOL2 || \
     ENABLE_ACTUATOR_XL330           || \
     ENABLE_ACTUATOR_XC330           || \
     ENABLE_ACTUATOR_XC430           || \
     ENABLE_ACTUATOR_XL430           || \
     ENABLE_ACTUATOR_XM430           || \
     ENABLE_ACTUATOR_XH430           || \
     ENABLE_ACTUATOR_XD430           || \
     ENABLE_ACTUATOR_XM540           || \
     ENABLE_ACTUATOR_XH540           || \
     ENABLE_ACTUATOR_XD540           || \
     ENABLE_ACTUATOR_XW540           || \
     ENABLE_ACTUATOR_XW430)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -1048575, 1048575, 0.088},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.088},

    {SET_PWM, GOAL_PWM, UNIT_RAW, -885, 885, 1},
    {GET_PWM, PRESENT_PWM, UNIT_RAW, -885, 885, 1},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_mx64_2[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX64_PROTOCOL2)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -1193, 1193, 3.36},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1193, 1193, 3.36},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_mx106_2[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX106_PROTOCOL2)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 3.36},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 3.36},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xl330_M288_M077[] PROGMEM = {
#if (ENABLE_ACTUATOR_XL330)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -1750, 1750, 1},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1750, 1750, 1},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2047, 2047, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2047, 2047, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xl430_xc430[] PROGMEM = {
#if (ENABLE_ACTUATOR_MX28_PROTOCOL2 || ENABLE_ACTUATOR_XL430 || ENABLE_ACTUATOR_XC430)
    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xc330_m181_m288[] PROGMEM = {
#if (ENABLE_ACTUATOR_XC330)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -2352, 2352, 1},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -2352, 2352, 1},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2047, 2047, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2047, 2047, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xc330_t181_t288[] PROGMEM = {
#if (ENABLE_ACTUATOR_XC330)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -910, 910, 1},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -910, 910, 1},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2047, 2047, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2047, 2047, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xm430_w210_w350[] PROGMEM = {
#if (ENABLE_ACTUATOR_XM430)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -1193, 1193, 2.69},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1193, 1193, 2.69},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xh430_wt210_wt350[] PROGMEM = {
#if (ENABLE_ACTUATOR_XH430 || ENABLE_ACTUATOR_XD430)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -648, 648, 2.69},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -648, 648, 2.69},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xw430_t200_t333[] PROGMEM = {
#if (ENABLE_ACTUATOR_XW430)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 2.69},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 2.69},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xh430_v210_v350[] PROGMEM = {
#if (ENABLE_ACTUATOR_XH430)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -689, 689, 1.34},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -689, 689, 1.34},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xm540_xh540_xt540_xw540[] PROGMEM = {
#if (ENABLE_ACTUATOR_XM540 || ENABLE_ACTUATOR_XH540 || \
     ENABLE_ACTUATOR_XD540 || ENABLE_ACTUATOR_XW540)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 2.69},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -2047, 2047, 2.69},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_xh540_v150_v270[] PROGMEM = {
#if (ENABLE_ACTUATOR_XH540 || ENABLE_ACTUATOR_XD540)
    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -1188, 1188, 2.69},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1188, 1188, 2.69},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -1023, 1023, 0.229},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

/* PRO R series */
static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_r_m42_10[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_R)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00136785},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00136785},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -8000, 8000, 0.00389076},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -8000, 8000, 0.00389076},

    {SET_CURRENT, GOAL_TORQUE, UNIT_MILLI_AMPERE, -900, 900, 4.02832},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -900, 900, 4.02832},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_r_m54_40[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_R)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00143188},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00143188},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -8000, 8000, 0.00397746},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -8000, 8000, 0.00397746},  

    {SET_CURRENT, GOAL_TORQUE, UNIT_MILLI_AMPERE, -360, 360, 16.11328},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -360, 360, 16.11328},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_r_m54_60[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_R)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00143188},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647,  0.00143188},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -8000, 8000, 0.00397746},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -8000, 8000, 0.00397746},  

    {SET_CURRENT, GOAL_TORQUE, UNIT_MILLI_AMPERE, -540, 540, 16.11328},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -540, 540, 16.11328},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_r_h42_20[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_R)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00118518},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00118518},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -10300, 10300, 0.00329218},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -10300, 10300, 0.00329218},  

    {SET_CURRENT, GOAL_TORQUE, UNIT_MILLI_AMPERE, -1395, 1395, 4.02832},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1395, 1395, 4.02832},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_r_h54_100[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_R)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00071724},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00071724},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -17000, 17000, 0.00199234},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -17000, 17000, 0.00199234},  

    {SET_CURRENT, GOAL_TORQUE, UNIT_MILLI_AMPERE, -930, 930, 16.11328},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -930, 930, 16.11328},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_r_h54_200[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_R)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00071724},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00071724},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -17000, 17000, 0.00199234},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -17000, 17000, 0.00199234},  

    {SET_CURRENT, GOAL_TORQUE, UNIT_MILLI_AMPERE, -1860, 1860, 16.11328},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1860, 1860, 16.11328},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};


/* PRO RA, PRO PLUS series */
static const ModelDependencyFuncItemAndRangeInfo_t dependency_ctable_pro_ra_pro_plus_model[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_RA || ENABLE_ACTUATOR_PRO_PLUS)
    {SET_PWM, GOAL_PWM, UNIT_RAW, -2009, 2009, 1},
    {GET_PWM, PRESENT_PWM, UNIT_RAW, -2009, 2009, 1},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_ra_plus_m42_10[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_RA || ENABLE_ACTUATOR_PRO_PLUS)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00068392},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00068392},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2600, 2600, 0.01},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2600, 2600, 0.01},  

    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -1461, 1461, 1.0},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -1461, 1461, 1.0},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_ra_plus_m54_40[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_RA || ENABLE_ACTUATOR_PRO_PLUS)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00071594},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00071594},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2840, 2840, 0.01},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2840, 2840, 0.01},  

    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -4470, 4470, 1.0},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -4470, 4470, 1.0},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_ra_plus_m54_60[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_RA || ENABLE_ACTUATOR_PRO_PLUS)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00071594},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00071594},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2830, 2830, 0.01},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2830, 2830, 0.01},  

    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -7980, 7980, 1.0},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -7980, 7980, 1.0},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_ra_plus_h42_20[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_RA || ENABLE_ACTUATOR_PRO_PLUS)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00059259},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00059259},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2920, 2920, 0.01},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2920, 2920, 0.01},  

    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -4500, 4500, 1.0},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -4500, 4500, 1.0},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_ra_plus_h54_100[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_RA || ENABLE_ACTUATOR_PRO_PLUS)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00035862},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00035862},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2920, 2920, 0.01},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2920, 2920, 0.01},  

    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -15900, 15900, 1.0},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -15900, 15900, 1.0},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static const ModelDependencyFuncItemAndRangeInfo_t dependency_pro_ra_plus_h54_200[] PROGMEM = {
#if (ENABLE_ACTUATOR_PRO_RA || ENABLE_ACTUATOR_PRO_PLUS)
    {SET_POSITION, GOAL_POSITION, UNIT_DEGREE, -2147483647, 2147483647, 0.00035862},
    {GET_POSITION, PRESENT_POSITION, UNIT_DEGREE, -2147483647 , 2147483647, 0.00035862},

    {SET_VELOCITY, GOAL_VELOCITY, UNIT_RPM, -2900, 2900, 0.01},
    {GET_VELOCITY, PRESENT_VELOCITY, UNIT_RPM, -2900, 2900, 0.01},  

    {SET_CURRENT, GOAL_CURRENT, UNIT_MILLI_AMPERE, -22740, 22740, 1.0},
    {GET_CURRENT, PRESENT_CURRENT, UNIT_MILLI_AMPERE, -22740, 22740, 1.0},
#endif
    {LAST_DUMMY_FUNC, ControlTableItem::LAST_DUMMY_ITEM, UNIT_RAW, 0, 0, 0}
};

static ItemAndRangeInfo_t
getModelDependencyFuncInfo(uint16_t model_num, uint8_t func_num) {
    const ModelDependencyFuncItemAndRangeInfo_t *p_common_ctable = nullptr;
    const ModelDependencyFuncItemAndRangeInfo_t *p_dep_ctable = nullptr;
    uint8_t func_idx, i = 0;
    ItemAndRangeInfo_t item_info;

    memset(&item_info, 0, sizeof(item_info));
    item_info.item_idx = ControlTableItem::LAST_DUMMY_ITEM;
    switch(model_num) {
        case AX12A:
        case AX12W:
        case AX18A:
        case DX113:
        case DX116:
        case DX117:
        case RX10:
        case RX24F:
        case RX28:
        case RX64:
            p_common_ctable = dependency_ctable_1_0_common;
            break;

        case EX106:
            p_common_ctable = dependency_ctable_1_0_common;
            p_dep_ctable = dependency_ex;
            break;

        case MX12W:
            p_common_ctable = dependency_ctable_1_1_common;
            p_dep_ctable = dependency_mx12;
            break;

        case MX28:
            p_common_ctable = dependency_ctable_1_1_common;
            break;

        case MX64:
        case MX106:
            p_common_ctable = dependency_ctable_1_1_common;
            p_dep_ctable = dependency_mx64_mx106;
            break;              

        case XL320:
            p_common_ctable = dependency_xl320;
            break;

        case MX28_2:
        case XC430_W150:
        case XC430_W240:
        case XXC430_W250:
        case XL430_W250:
        case XXL430_W250:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xl430_xc430;
            break;

        case MX64_2:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_mx64_2;
            break;    

        case MX106_2:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_mx106_2;
            break;     

        case XL330_M288:
        case XL330_M077:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xl330_M288_M077;
            break;

        case XC330_M181:
        case XC330_M288:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xc330_m181_m288;
            break;

        case XC330_T181:
        case XC330_T288:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xc330_t181_t288;
            break;

        case XM430_W210:
        case XM430_W350:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xm430_w210_w350;
            break;

        case XH430_V210:
        case XH430_V350:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xh430_v210_v350;
            break;

        case XH430_W210:
        case XH430_W350:
        case XD430_T210:
        case XD430_T350:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xh430_wt210_wt350;
            break;    

        case XW430_T200:
        case XW430_T333:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xw430_t200_t333;
            break;   

        case XM540_W150:
        case XM540_W270:
        case XH540_W150:
        case XH540_W270:    
        case XD540_T150:
        case XD540_T270:    
        case XW540_T140:
        case XW540_T260:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xm540_xh540_xt540_xw540;
            break;

        case XH540_V150:
        case XH540_V270:
            p_common_ctable = dependency_ctable_2_0_common;
            p_dep_ctable = dependency_xh540_v150_v270;
            break;

        // case PRO_L42_10_S300_R:
        // case PRO_L54_30_S400_R:
        // case PRO_L54_30_S500_R:
        // case PRO_L54_50_S290_R:
        // case PRO_L54_50_S500_R:
        case PRO_M42_10_S260_R:
            p_common_ctable = dependency_pro_r_m42_10;
            break;    

        case PRO_M54_40_S250_R:
            p_common_ctable = dependency_pro_r_m54_40;
            break;  

        case PRO_M54_60_S250_R:
            p_common_ctable = dependency_pro_r_m54_60;
            break;  

        case PRO_H42_20_S300_R:
            p_common_ctable = dependency_pro_r_h42_20;
            break;   

        case PRO_H54_100_S500_R:
            p_common_ctable = dependency_pro_r_h54_100;
            break;    

        case PRO_H54_200_S500_R:
            p_common_ctable = dependency_pro_r_h54_200;
            break;

        case PRO_M42_10_S260_RA:    
        case PRO_M42P_010_S260_R:
            p_common_ctable = dependency_ctable_pro_ra_pro_plus_model;
            p_dep_ctable = dependency_pro_ra_plus_m42_10;
            break;

        case PRO_M54_40_S250_RA:
        case PRO_M54P_040_S250_R:
            p_common_ctable = dependency_ctable_pro_ra_pro_plus_model;
            p_dep_ctable = dependency_pro_ra_plus_m54_40;
            break;

        case PRO_M54_60_S250_RA:
        case PRO_M54P_060_S250_R:
            p_common_ctable = dependency_ctable_pro_ra_pro_plus_model;
            p_dep_ctable = dependency_pro_ra_plus_m54_60;
            break;

        case PRO_H42_20_S300_RA:
        case PRO_H42P_020_S300_R:
            p_common_ctable = dependency_ctable_pro_ra_pro_plus_model;
            p_dep_ctable = dependency_pro_ra_plus_h42_20;
            break;

        case PRO_H54_100_S500_RA:  
        case PRO_H54P_100_S500_R:
            p_common_ctable = dependency_ctable_pro_ra_pro_plus_model;
            p_dep_ctable = dependency_pro_ra_plus_h54_100;
            break;

        case PRO_H54_200_S500_RA:
        case PRO_H54P_200_S500_R:
            p_common_ctable = dependency_ctable_pro_ra_pro_plus_model;
            p_dep_ctable = dependency_pro_ra_plus_h54_200;
            break;

        default:
            break;
    }

    if (p_common_ctable == nullptr) {
        return item_info;
    }

    do {
        func_idx = p_common_ctable[i].func_idx;
        if (func_idx == func_num) {
            item_info.item_idx = p_common_ctable[i].item_idx;
            item_info.min_value = p_common_ctable[i].min_value;
            item_info.max_value = p_common_ctable[i].max_value;
            item_info.unit_type = p_common_ctable[i].unit_type;
            item_info.unit_value = p_common_ctable[i].unit_value;
            break;
        }
        i++;
    } while(func_idx != LAST_DUMMY_FUNC);

    if (p_dep_ctable == nullptr) {
        return item_info;
    }
    i = 0;
    do {
        func_idx = p_dep_ctable[i].func_idx;
        if (func_idx == func_num) {
            item_info.item_idx = p_dep_ctable[i].item_idx;
            item_info.min_value = p_dep_ctable[i].min_value;
            item_info.max_value = p_dep_ctable[i].max_value;
            item_info.unit_type = p_dep_ctable[i].unit_type;
            item_info.unit_value = p_dep_ctable[i].unit_value;
            break;
        }
        i++;
    } while(func_idx != LAST_DUMMY_FUNC);
    return item_info;  
}

static bool
checkAndconvertWriteData(float in_data, int32_t &out_data, ParamUnit unit, ItemAndRangeInfo_t &item_info) {
    float data_f = 0.0;
    int32_t data = 0;  

    switch(unit) {
        case UNIT_RAW:
            data = (int32_t)in_data;
            if (data < item_info.min_value || data > item_info.max_value)
                return false;
            break;

        case UNIT_PERCENT:
            data_f = f_map(in_data, -100.0, 100.0, (float)item_info.min_value, (float)item_info.max_value);
            data = (int32_t)data_f;
            break;

        case UNIT_RPM:
        case UNIT_DEGREE:
        case UNIT_MILLI_AMPERE:
            if (unit != item_info.unit_type)
                return false;
            data = (int32_t)round(in_data/item_info.unit_value);
            if (data < item_info.min_value || data > item_info.max_value)
                return false;
            break;

        default:
            return false;
    }
    out_data = data;
    return true;
}

static bool checkAndconvertReadData(int32_t in_data, float &out_data, ParamUnit unit, ItemAndRangeInfo_t &item_info) {
    float data = 0;

    switch (unit) {
        case UNIT_RAW:
            data = (float)in_data;
            break;

        case UNIT_PERCENT:
            data = (float)f_map(in_data, (float)item_info.min_value, (float)item_info.max_value, -100.0, 100.0);
            break;

        case UNIT_RPM:
        case UNIT_DEGREE:
        case UNIT_MILLI_AMPERE:
            if (unit != item_info.unit_type)
                return false;
            data = (float)in_data*item_info.unit_value;
            break;

        default:
            return false;
    }
    out_data = data;
    return true;
}

static float f_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float
DynamixelInternal::getTorqueConstant(uint8_t id) {
    uint16_t model_num = getModelNumberFromTable(id);
    float Kt_Nm_per_mA = 0;
    switch (model_num) {
        case XM540_W150:
            printf("[%d] XM540_W150\r\n", id);
            // From the XM540-W150 e-Manual:
            // Stall torque: 7.3 Nm @ 12V
            // Stall current: ~4.4 A
            // → Approximate Kt ≈ 7.3 Nm / 4.4 A ≈ 1.659 Nm/A
            // → Or 0.001659 Nm/mA
            Kt_Nm_per_mA = 0.001659;
            break;
        case XM540_W270:
            printf("[%d] XM540_W270\r\n", id);
            // From the XM540-W270 e-Manual:
            // Stall torque: 10.6 Nm @ 12V
            // Stall current: ~4.4 A
            // → Approximate Kt ≈ 10.6 Nm / 4.4 A ≈ 2.409 Nm/A
            // → Or 0.002409 Nm/mA
            Kt_Nm_per_mA = 0.002409;
            break;

        case XH540_V150:
            printf("[%d] XH540_V150\r\n", id);
            // From the XH540-V150 e-Manual:
            // Stall torque: 6.4 Nm @ 24V
            // Stall current: ~2.4 A
            // → Approximate Kt ≈ 6.4 Nm / 2.4 A ≈ 2.67 Nm/A
            // → Or 0.00267 Nm/mA
            Kt_Nm_per_mA = 0.00267;
            break;
        case XH540_V270:
            printf("[%d] XH540_V270\r\n", id);
            // From the XH540-V270 e-Manual:
            // Stall torque: 9.2 Nm @ 24V
            // Stall current: ~2.4 A
            // → Approximate Kt ≈ 9.2 Nm / 2.4 A ≈ 3.83 Nm/A
            // → Or 0.00296 Nm/mA
            Kt_Nm_per_mA = 0.00383;
            break;
        case XH540_W150:
            printf("[%d] XH540_W150\r\n", id);
            // From the XH540-W150 e-Manual:
            // Stall torque: 7.1 Nm @ 12V
            // Stall current: ~4.9 A
            // → Approximate Kt ≈ 7.1 Nm / 4.9 A ≈ 1.44 Nm/A
            // → Or 0.00144 Nm/mA
            Kt_Nm_per_mA = 0.00144;
            break;
        case XH540_W270:
            printf("[%d] XH540_W270\r\n", id);
            // From the XH540-W270 e-Manual:
            // Stall torque: 9.9 Nm @ 12V
            // Stall current: ~4.9 A
            // → Approximate Kt ≈ 9.9 Nm / 4.9 A ≈ 2.02 Nm/A
            // → Or 0.00202 Nm/mA
            Kt_Nm_per_mA = 0.00202;
            break;
        case XC330_T181:
            printf("[%d] XC330_T181\r\n", id);
            // From the XC330-T181 e-Manual:
            // Stall torque: 0.8 Nm @ 12V
            // Stall current: 0.88 A
            // → Approximate Kt ≈ 0.8 Nm / 0.88 ≈ 0.91 Nm/A
            // → Or 0.00091 Nm/mA
            Kt_Nm_per_mA = 0.00091;
            break;
        case XC330_T288:
            printf("[%d] XC330_T288\r\n", id);
            // From the XC330-T288 e-Manual:
            // Stall torque: 1.0 Nm @ 12V
            // Stall current: 0.88 A
            // → Approximate Kt ≈ 1.0 Nm / 0.88 ≈ 1.136 Nm/A
            // → Or 0.001136 Nm/mA
            Kt_Nm_per_mA = 0.001136;
            break;

        default:
            PDLOG_ERROR("UNSUPPORTED DXL MODEL 0x%04X [id:%d]. TORQUE CONSTANT UNKNOWN.\n", model_num, id);
            break;
    }
    printf("getTorqueConstant id=%d model_num=%04X => %f\r\n", id, model_num, Kt_Nm_per_mA);
    return Kt_Nm_per_mA;
}

float
DynamixelInternal::getTorqueNm(uint8_t id, float current_mA) {
    return current_mA * getTorqueConstant(id);
}

int32_t
DynamixelInternal::getCurrentLimit(uint8_t id) {
    int32_t ret = 0;
    uint16_t model_num = getModelNumberFromTable(id);

    switch(model_num) {
        default:
            ret = readControlTableItem(ControlTableItem::CURRENT_LIMIT, id);
            break;
    }
    return ret;
}
