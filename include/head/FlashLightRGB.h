#pragma once

////////////////////////////////

#include "core/AnimatedEvent.h"
#include "core/SetupEvent.h"
#include <Adafruit_NeoPixel.h>

////////////////////////////////

class FlashLightRGB: protected Adafruit_NeoPixel, public AnimatedEvent, public SetupEvent {
public:

    FlashLightRGB(int pin, int numPixels = 1) :
        Adafruit_NeoPixel(numPixels, pin, NEO_GRBW + NEO_KHZ800)
    {
        setColor(255, 255, 255, false);
    }

    // Set the power level of the flashlight
    void setPower(int power) {
        if (power < 0) {
            power = 0;
        } else if (power > 100) {
            power = 100;
        }
        fMax_Power_Percentage = power;
        printf("Power: %d\n", fMax_Power_Percentage);
        setColor(fRed, fGreen, fBlue);
    }

    // Set the color of the flashlight, added an update flag to prevent crash when instantiating
    // the class since setColor is called in the constructor.
    void setColor(int r, int g, int b, bool update = true) {
        // Scale each color channel by fMax_Power_Percentage%
        fRed = r;
        fGreen = g; 
        fBlue = b;

        r = (r * fMax_Power_Percentage) / 100;
        g = (g * fMax_Power_Percentage) / 100;
        b = (b * fMax_Power_Percentage) / 100;

        fColor = Color(r, g, b);
        printf("Color: %d %d %d\n", r, g, b);
        if (update) {
            showState();
        }
    }

    virtual void setup() override {
        begin();
        clear();
    }

    // Get the current state of the flashlight, needed for toggling
    // the state of the flashlight
    bool getState() {
        return fState;
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

    uint8_t fMax_Power_Percentage = 25;
    uint32_t fColor = 0;

    // Need to keep the color state to be able to update the power and
    // keep the same color on the flashlight
    uint8_t fRed = 255;
    uint8_t fGreen = 255;
    uint8_t fBlue = 255;

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
