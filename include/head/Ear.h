#pragma once

////////////////////////////////

#include "encoder/QuadratureEncoder.h"

////////////////////////////////

#define ENCODER_STATUS_RATE 100 // ms (10Hz)
class EarStatus
{
public:
    EarStatus(QuadratureEncoder& encoder, int minValue = 2) :
        fEncoder(encoder),
        fMinValue(minValue)
    {
        fEncoder.resetChangedState();
        fLastStatus = millis();
    }

    bool isMoving() {
        uint32_t now = millis();
        if (fLastStatus + ENCODER_STATUS_RATE < now) {
            fLastStatus = now;
            if (fEncoder.getChangedState() <= fMinValue) {
                return false;
            }
            fEncoder.resetChangedState();
        }
        //return (servoDispatch.isActive(LEFT_EAR_MOTOR_A) || servoDispatch.isActive(LEFT_EAR_MOTOR_B));
        return true;
    }

private:
    QuadratureEncoder& fEncoder;
    uint32_t fLastStatus;
    int fMinValue;
};

////////////////////////////////
