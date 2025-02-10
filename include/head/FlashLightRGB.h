#pragma once

////////////////////////////////

#include "core/AnimatedEvent.h"
#include "core/SetupEvent.h"
#include <Adafruit_NeoPixel.h>

////////////////////////////////

class FlashLightRGB: protected Adafruit_NeoPixel, public AnimatedEvent, public SetupEvent {
public:
    static constexpr int MAX_POWER_PERCENTAGE = 25;

    FlashLightRGB(int pin, int numPixels = 1) :
        Adafruit_NeoPixel(numPixels, pin, NEO_GRBW + NEO_KHZ800)
    {
        setColor(255, 255, 255);
    }

    void setColor(int r, int g, int b) {
        // Scale each color channel by MAX_POWER_PERCENTAGE%
        r = (r * MAX_POWER_PERCENTAGE) / 100;
        g = (g * MAX_POWER_PERCENTAGE) / 100;
        b = (b * MAX_POWER_PERCENTAGE) / 100;
        fColor = Color(r, g, b);
        fRefresh = true;
    }

    virtual void setup() override {
        begin();
        clear();
    }

    void setState(bool enabled, uint32_t duration = 0) {
        fState = enabled;
        if (duration != 0) {
            fAutoOffTime = millis() + duration;
        }
        showState();
    }

    virtual void animate() override {
        if (fRefresh || (fState && fAutoOffTime && fAutoOffTime < millis())) {
            if (fState) {
                // Switch off flashlight
                fState = false;
            }
            showState();
            fRefresh = false;
        }
    }

protected:
    bool fState = false;
    bool fRefresh = false;
    uint32_t fColor = 0;
    uint32_t fAutoOffTime = 0;

    void showState() {
        if (fState) {
            for (int i = 0; i < numPixels(); i++) {
                setPixelColor(i, fColor);
            }
            DEBUG_PRINTLN("FLASHLIGHT ON\n");
        } else {
            clear();
            DEBUG_PRINTLN("FLASHLIGHT OFF\n");
        }
        show();
    }
};

////////////////////////////////
