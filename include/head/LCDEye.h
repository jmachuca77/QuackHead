#pragma once

////////////////////////////////

#include "core/AnimatedEvent.h"
#include "core/SetupEvent.h"
#include <TFT_eSPI.h>
#include "pin-map.h"

////////////////////////////////
enum EyeState {
    EYE_OFF = 0,
    EYE_OPEN = 1,
    EYE_BLINK = 2,
    EYE_CLOSE = 3,
    EYE_ANGRY = 4
};

class LCDEye : public AnimatedEvent, public SetupEvent {
public:
    LCDEye(int numEyes, const int* csPins, int resolutionX = 240, int resolutionY = 240) 
        : tft(TFT_eSPI(resolutionX, resolutionY)), numEyes(numEyes) {
        for (int i = 0; i < numEyes; i++) {
            chipSelectPins[i] = csPins[i];
        }
        for (int i = 0; i < numEyes; i++) { // Set all chip select pins to output and high
            pinMode(chipSelectPins[i], OUTPUT);
            digitalWrite(chipSelectPins[i], HIGH); 
        }
        reset();
    }

    void reset() {
        fState = EYE_OPEN;
        fPassive = false;
        fRapidBlink = false;
        fRapidBlinkCount = 0;
        fNextTime = 0;
        fOnColor = TFT_WHITE;
        fOffColor = TFT_BLACK;
        fAngryColor = TFT_RED;
        fMinBlinkDelay = 200;
        fMaxBlinkDelay = 700;
        fMinOpenTime = 1000;
        fMaxOpenTime = 10000;
        fMinCloseTime = 180;
        fMaxCloseTime = 200;
        fInterval = 20;
        fDelayTime = millis();
        printf("LCD RESET\n");
    }

    void setOnColor(uint16_t color) {
        fOnColor = color;
    }

    void setOffColor(uint16_t color) {
        fOffColor = color;
    }

    void setAngryColor() {
        uint32_t now = millis();
        fState = EYE_ANGRY;
        uint32_t interval = random(fMinOpenTime, fMaxOpenTime);
        fDelayTime = now + interval;
        showState(fState, fOffColor, fOnColor);
        printf("Time %d Angry Eyes for %dms, %d\n", now, interval, fState);
    }

    void turnOffEyes() {
        fState = EYE_OFF;
        showState(fState, fOffColor, fOnColor);
    }

    void turnOnEyes() {
        fState = EYE_OPEN;
        showState(fState, fOffColor, fOnColor);
        fDelayTime = millis() + random(fMinOpenTime, fMaxOpenTime);
    }

    virtual void setup() override {
        for (int i = 0; i < numEyes; i++) {
            digitalWrite(chipSelectPins[i], LOW);
        }

        tft.begin();
        tft.setRotation(0);
        clearScreen(fOffColor);

        for (int i = 0; i < numEyes; i++) {
            digitalWrite(chipSelectPins[i], HIGH); 
        }
        printf("LCD SETUP\n");
    }

    virtual void animate() override {
        auto now = millis();
        uint32_t interval = 0;
        switch (fState) {
            case EYE_ANGRY:   // Angry and open are basically the same just differnt color
            case EYE_OPEN:    // Eye will be open until delayTime is reached
                if (now >= fDelayTime) {
                    interval = random(fMinBlinkDelay, fMaxBlinkDelay);
                    fDelayTime = now + interval;
                    fState = EYE_BLINK;
                    showState(fState, fOffColor, fOnColor);
                    printf("Time %d Closing Eyes for %dms, %d\n", now, interval, fState);
                }
                break;

            case EYE_BLINK:     // Eye will be closed until delayTime is reached
                if (now >= fDelayTime) {
                    interval = random(fMinOpenTime, fMaxOpenTime);
                    fDelayTime = now + interval;
                    fState = EYE_OPEN;
                    showState(fState, fOffColor, fOnColor);
                    printf("Time %d Opening Eyes for %dms, %d\n", now, interval, fState);
                }
                break;

            case EYE_CLOSE:
                fDelayTime = now + random(fMinCloseTime, fMaxCloseTime);
                showState(fState, fOffColor, fOnColor);
                break;

            case EYE_OFF:
                break;
        }
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
    uint16_t fAngryColor;
    uint32_t fMinBlinkDelay;
    uint32_t fMaxBlinkDelay;
    uint32_t fMinOpenTime;
    uint32_t fMaxOpenTime;
    uint32_t fMinCloseTime;
    uint32_t fMaxCloseTime;
    uint32_t fInterval;
    uint32_t fDelayTime;
    LCDEye* fSyncEye = nullptr;

    void clearScreen(uint16_t color) {
        tft.fillScreen(color);
    }

    void showState(int state, uint16_t offColor, uint16_t onColor) {
        for (int i = 0; i < numEyes; i++) {
            //printf("LCD %d SHOW %d\n", i, state);
            digitalWrite(chipSelectPins[i], LOW);
            switch (state) {
                case EYE_OFF:
                    clearScreen(offColor);
                    break;
                case EYE_OPEN:
                    clearScreen(onColor);
                    break;
                case EYE_ANGRY:
                    clearScreen(fAngryColor);
                    break;
                case EYE_BLINK:
                    clearScreen(offColor);
                    break;
                case EYE_CLOSE:
                    clearScreen(offColor);
                    break;
            }
            digitalWrite(chipSelectPins[i], HIGH);
        }
    }
};
