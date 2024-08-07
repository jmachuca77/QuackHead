#pragma once

////////////////////////////////

#include "ServoDispatchPCA9685.h"
#include "core/AnimatedEvent.h"

////////////////////////////////

class FlashLight: public AnimatedEvent {
public:
    FlashLight(ServoDispatch& servoDispatch, int pin) :
        fServoDispatch(servoDispatch),
        fPin(pin)
    {
    }

    virtual void animate() override {
        uint32_t now = millis();
        if (fNextTime > now)
            return;
        uint32_t duration = random(1000, 4000);
        fNextTime = now + random(4000, 8000) + duration;
        fState = !fState;
        if (fState) {
            fServoDispatch.setPWM(fPin, 0, 2000);
            printf("FLASHLIGHT?\n");
        } else {
            fServoDispatch.setPWMOff(fPin);
        }
    }

protected:
    ServoDispatch& fServoDispatch;
    int fPin;
    bool fState = true;
    uint32_t fNextTime;
};

////////////////////////////////
