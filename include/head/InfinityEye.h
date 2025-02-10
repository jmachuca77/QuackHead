#pragma once

////////////////////////////////

#include "core/AnimatedEvent.h"
#include "core/SetupEvent.h"
#include <Adafruit_NeoPixel.h>

////////////////////////////////

class InfinityEye: protected Adafruit_NeoPixel, public AnimatedEvent, public SetupEvent {
public:
    InfinityEye(int pin, int numPixels = 17) :
        Adafruit_NeoPixel(numPixels, pin, NEO_GRB + NEO_KHZ800)
    {
        reset();
    }

    void syncWith(InfinityEye* otherEye) {
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
        fOnColor = Color(127, 127, 127);
        fOffColor = 0;
        fMinBlinkDelay = 1000;
        fMaxBlinkDelay = 4000;
        fMinCloseTime = 180;
        fMaxCloseTime = 200;
        fInterval = 20;
    }

    void setOnColor(int r, int g, int b) {
        fState = 1;
        fNextTime = 0;
        fOnColor = Color(r, g, b);
    }

    void setOffColor(int r, int g, int b) {
        fState = 1;
        fNextTime = 0;
        fOffColor = Color(r, g, b);
    }

    void setBlinkTime(
        uint32_t minDelay,
        uint32_t maxDelay,
        uint32_t minCloseTime = 100,
        uint32_t maxCloseTime = 200,
        uint32_t interval = 20)
    {
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
        begin();
        clear();
        for (int i = 0; i < numPixels(); i++) {
            setPixelColor(i, fOnColor);
        }
        show();
    }

    virtual void animate() override {
        // Basic blink-state machine with flexible # of states
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
        if (fSyncEye)
            fSyncEye->showState(fState, fOffColor, fOnColor);
        fNextTime = now + delayTime;
        fState = nextState;
    }

protected:
    int fState;
    bool fPassive;
    bool fRapidBlink;
    int32_t fRapidBlinkCount;
    uint32_t fNextTime;
    uint32_t fOnColor;
    uint32_t fOffColor;
    uint32_t fMinBlinkDelay;
    uint32_t fMaxBlinkDelay;
    uint32_t fMinCloseTime;
    uint32_t fMaxCloseTime;
    uint32_t fInterval;
    InfinityEye* fSyncEye = nullptr;

    void showState(int state, uint32_t offColor, uint32_t onColor) {
        switch (state) {
            case 0:
                break;
            case 1:
                setPixelColor(0, offColor);
                setPixelColor(15, offColor);
                break;
            case 2:
                setPixelColor(1, offColor);
                setPixelColor(14, offColor);
                break;
            case 3:
                setPixelColor(2, offColor);
                setPixelColor(13, offColor);
                break;
            case 4:
                setPixelColor(3, offColor);
                setPixelColor(12, offColor);
                break;
            case 5:
                setPixelColor(4, offColor);
                setPixelColor(11, offColor);
                break;
            case 6:
                setPixelColor(5, offColor);
                setPixelColor(10, offColor);
                break;
            case 7:
                setPixelColor(6, offColor);
                setPixelColor(9, offColor);
                break;
            case 8:
                setPixelColor(7, offColor);
                setPixelColor(8, offColor);
                break;
            case 9:
                setPixelColor(7, onColor);
                setPixelColor(8, onColor);
                break;
            case 10:
                setPixelColor(6, onColor);
                setPixelColor(9, onColor);
                break;
            case 11:
                setPixelColor(5, onColor);
                setPixelColor(10, onColor);
                break;
            case 12:
                setPixelColor(4, onColor);
                setPixelColor(11, onColor);
                break;
            case 13:
                setPixelColor(3, onColor);
                setPixelColor(12, onColor);
                break;
            case 14:
                setPixelColor(2, onColor);
                setPixelColor(13, onColor);
                break;
            case 15:
                setPixelColor(1, onColor);
                setPixelColor(14, onColor);
                break;
            case 16:
                setPixelColor(0, onColor);
                setPixelColor(15, onColor);
                break;
        }
        show();
    }
};

