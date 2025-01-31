#pragma once

////////////////////////////////

#include "core/AnimatedEvent.h"
#include "core/SetupEvent.h"
#include <TFT_eSPI.h>
#include "pin-map.h"

////////////////////////////////

class LCDEye : public AnimatedEvent, public SetupEvent {
public:
    LCDEye(int numEyes, const int* csPins, int resolutionX = 240, int resolutionY = 240) 
        : tft(TFT_eSPI(resolutionX, resolutionY)), numEyes(numEyes) {
        for (int i = 0; i < numEyes; i++) {
            chipSelectPins[i] = csPins[i];
        }
    }

    void syncWith(LCDEye* otherEye) {
        if (otherEye != nullptr) {
            otherEye->fPassive = true;
        } else if (fSyncEye != nullptr) {
            fSyncEye->reset();
        }
        fSyncEye = otherEye;
    }

    void reset() {
        fState = 0;
        fPassive = false;
        fRapidBlink = false;
        fRapidBlinkCount = 0;
        fNextTime = 0;
        fOnColor = TFT_WHITE;
        fOffColor = TFT_BLACK;
        fMinBlinkDelay = 1000;
        fMaxBlinkDelay = 4000;
        fMinCloseTime = 180;
        fMaxCloseTime = 200;
        fInterval = 20;
        printf("LCD RESET\n");
    }

    void setOnColor(uint16_t color) {
        fState = 1;
        fNextTime = 0;
        fOnColor = color;
    }

    void setOffColor(uint16_t color) {
        fState = 1;
        fNextTime = 0;
        fOffColor = color;
    }

    void setBlinkTime(uint32_t minDelay, uint32_t maxDelay, uint32_t minCloseTime = 100, uint32_t maxCloseTime = 200, uint32_t interval = 20) {
        fMinBlinkDelay = minDelay;
        fMaxBlinkDelay = maxDelay;
        fMinCloseTime = minCloseTime;
        fMaxCloseTime = maxCloseTime;
        fInterval = interval;
        fState = 1;
        fNextTime = 0;
    }

    void rapidBlink() {
        fRapidBlink = true;
        fRapidBlinkCount = random(1, 5);
        setBlinkTime(100, 500, 10, 50, random(5, 10));
        printf("RAPID BLINK count=%d\n", fRapidBlinkCount);
    }

    void normalBlink() {
        setBlinkTime(1000, 4000);
        fRapidBlink = false;
        fRapidBlinkCount = 0;
        printf("NORMAL BLINK\n");
    }

    virtual void setup() override {
        tft.begin();
        tft.setRotation(0);
        for (int i = 0; i < numEyes; i++) {
            pinMode(chipSelectPins[i], OUTPUT);
            digitalWrite(chipSelectPins[i], LOW);
            clearScreen(fOffColor);
            digitalWrite(chipSelectPins[i], HIGH);  // Ensure all are disabled initially
        }
        printf("LCD SETUP\n");
    }

    virtual void animate() override {
        auto now = millis();
        if (fPassive || fNextTime > now)
            return;

        int nextState = fState + 1;
        uint32_t delayTime = fInterval;

        switch (fState) {
            case 0:
                if (fRapidBlink) {
                    if (fRapidBlinkCount == 0)
                        normalBlink();
                    else
                        fRapidBlinkCount -= 1;
                } else if (random(100) < 10) {
                    rapidBlink();
                }
                delayTime = random(fMinBlinkDelay, fMaxBlinkDelay);
                break;
            case 8:
                delayTime = random(delayTime, delayTime + random(fMinCloseTime, fMaxCloseTime));
                break;
            case 16:
                nextState = 0;
                break;
        }

        showState(fState, fOffColor, fOnColor);

        if (fSyncEye) {
            fSyncEye->showState(fState, fOffColor, fOnColor);
        }

        fNextTime = now + delayTime;
        fState = nextState;
        printf("LCD ANIMATE %d\n", fState);
    }

protected:
    TFT_eSPI tft;
    int numEyes;
    int chipSelectPins[2];  // Supports up to 2 eyes
    int fState;
    bool fPassive;
    bool fRapidBlink;
    int32_t fRapidBlinkCount;
    uint32_t fNextTime;
    uint16_t fOnColor;
    uint16_t fOffColor;
    uint32_t fMinBlinkDelay;
    uint32_t fMaxBlinkDelay;
    uint32_t fMinCloseTime;
    uint32_t fMaxCloseTime;
    uint32_t fInterval;
    LCDEye* fSyncEye = nullptr;

    void clearScreen(uint16_t color) {
        tft.fillScreen(color);
    }

    void showState(int state, uint16_t offColor, uint16_t onColor) {
        for (int i = 0; i < numEyes; i++) {
            //printf("LCD %d SHOW %d\n", i, state);
            digitalWrite(chipSelectPins[i], LOW);
            if (state <= 8) {
                clearScreen(offColor);
            } else {
                clearScreen(onColor);
            }
            digitalWrite(chipSelectPins[i], HIGH);
        }
    }
};
