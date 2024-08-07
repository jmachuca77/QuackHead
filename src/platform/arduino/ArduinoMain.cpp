#include <Arduino.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

///////////////////////////////////

#if __has_include("build_version.h")
#include "build_version.h"
#endif

#if __has_include("reeltwo_build_version.h")
#include "reeltwo_build_version.h"
#endif

///////////////////////////////////
// CONFIGURABLE OPTIONS
///////////////////////////////////

#define MAX_OPEN_FILES 2

#define USE_DEBUG                       // Define to enable debug diagnostic
//#define USE_SMQDEBUG
#define SMQ_HOSTNAME                    "Warbler"
#define SMQ_SECRET                      "Astromech"

///////////////////////////////////

#include "pin-map.h"

///////////////////////////////////

#include "ReelTwo.h"
#include "audio/Audio.h"
#include "audio/Warbler.h"
#include "drive/TargetSteering.h"
#include "ServoEasing.h"
#include "ServoDispatchPCA9685.h"

////////////////////////////////

#include "head/Ear.h"
#include "head/FlashLight.h"
#include "head/InfinityEye.h"

////////////////////////////////

#ifdef USE_SDCARD
#include "SD.h"
#endif

////////////////////////////////
// AudioFrequencyBitmap sAudioBitmap;
WarblerAudio sWarblerAudio(SD, nullptr, I2S_BCLK_PIN, I2S_LRC_PIN, I2S_DOUT_PIN);

static bool sSDCardMounted;

////////////////////////////////

#define FLASHLIGHT_GROUP    0x0001
#define LEFT_EAR_GROUP_A    0x0010
#define LEFT_EAR_GROUP_B    0x0020
#define RIGHT_EAR_GROUP_A   0x0100
#define RIGHT_EAR_GROUP_B   0x0200

#define ENCODER_STATUS_RATE 100 // ms (10Hz)

#define FLASHLIGHT_PWM      1
#define LEFT_EAR_MOTOR_A    13
#define LEFT_EAR_MOTOR_B    14
#define RIGHT_EAR_MOTOR_B   15
#define RIGHT_EAR_MOTOR_A   16

const ServoSettings servoSettings[] PROGMEM = {
    {  1, 800, 2200, FLASHLIGHT_GROUP },
    {  2, 800, 2200, 0 },
    {  3, 800, 2200, 0 },
    {  4, 800, 2200, 0 },

    {  5, 800, 2200, 0 },
    {  6, 800, 2200, 0 },
    {  7, 800, 2200, 0 },
    {  8, 800, 2200, 0 },

    {  9, 800, 2200, 0 },
    { 10, 800, 2200, 0 },
    { 11, 800, 2200, 0 },
    { 12, 800, 2200, 0 },

    { 13, 800, 2500, LEFT_EAR_GROUP_A },
    { 14, 800, 2500, LEFT_EAR_GROUP_B },

    { 15, 800, 2500, RIGHT_EAR_GROUP_B },
    { 16, 800, 2500, RIGHT_EAR_GROUP_A },
};

ServoDispatchPCA9685<SizeOfArray(servoSettings)> servoDispatch(servoSettings);

////////////////////////////////

InfinityEye leftEye(LEFT_EYE_PIN);
InfinityEye rightEye(RIGHT_EYE_PIN);
FlashLight flashLight(servoDispatch, FLASHLIGHT_PWM);

QuadratureEncoder encoderLeftEar(LEFT_EAR_ENC_A, LEFT_EAR_ENC_B, false);
QuadratureEncoder encoderRightEar(RIGHT_EAR_ENC_A, RIGHT_EAR_ENC_B, true);

////////////////////////////////

bool getSDCardMounted()
{
    return sSDCardMounted;
}

bool mountSDFileSystem()
{
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (SD.begin(SD_CS_PIN, SPI, 4000000, "/sd", MAX_OPEN_FILES/*, false*/))
    {
        DEBUG_PRINTLN("Card Mount Success");
        sSDCardMounted = true;
        return true;
    }
    DEBUG_PRINTLN("Card Mount Failed");
    return false;
}

void unmountSDFileSystem()
{
    if (sSDCardMounted)
    {
        sSDCardMounted = false;
        SD.end();
    }
}

void unmountFileSystems()
{
    unmountSDFileSystem();
}

void warble(unsigned type)
{
    sWarblerAudio.playNext(type);
}

bool earSoundsActive;

void moveBothEarsToPosition(float pos, bool playSound = true, int fudge = 5, float speed = 1.0) {
    int targetPos = pos * 520;
    int currentLeftPos = encoderLeftEar.getValue();
    int currentRightPos = encoderRightEar.getValue();
    if (currentLeftPos >= targetPos - fudge && currentLeftPos <= targetPos + fudge &&
        currentRightPos >= targetPos - fudge && currentRightPos <= targetPos + fudge)
    {
        return;
    }

    TargetSteering leftSteering(targetPos);
    leftSteering.setDistanceTunings(1.0, 0.5, 0.5);
    leftSteering.setSampleTime(1);

    TargetSteering rightSteering(targetPos);
    rightSteering.setDistanceTunings(1.0, 0.5, 0.5);
    rightSteering.setSampleTime(1);

    if (abs(targetPos - currentLeftPos) > 60 ||
        abs(targetPos - currentRightPos) > 60)
    {
        if (playSound) {
            sWarblerAudio.startMotor();
        }
    }

    bool leftMotorOn = true;
    bool rightMotorOn = true;
    EarStatus leftEarStatus(encoderLeftEar, 5);
    EarStatus rightEarStatus(encoderRightEar, 5);
    uint32_t lastTimeCheck = 0;
    while (!Serial.available() && (leftMotorOn || rightMotorOn)) {
        if (playSound) {
            sWarblerAudio.animate();
        }
        currentLeftPos = encoderLeftEar.getValue();
        currentRightPos = encoderRightEar.getValue();
        leftSteering.setCurrentDistance(currentLeftPos);
        rightSteering.setCurrentDistance(currentRightPos);
        auto moveLeft = leftSteering.getThrottle()/* * speed*/;
        auto moveRight = rightSteering.getThrottle()/* * speed*/;
        int range = 2500 * speed;
        if (leftMotorOn) {
            if (moveLeft > 0) {
                servoDispatch.setPWM(LEFT_EAR_MOTOR_A, 0, range);
                servoDispatch.setPWM(LEFT_EAR_MOTOR_B, 0, range - (range * abs(moveLeft)));
            } else if (moveLeft < 0) {
                servoDispatch.setPWM(LEFT_EAR_MOTOR_B, 0, range);
                servoDispatch.setPWM(LEFT_EAR_MOTOR_A, 0, range - (range * abs(moveLeft)));
            } else {
                servoDispatch.setPWM(LEFT_EAR_MOTOR_A, 0, range);
                servoDispatch.setPWM(LEFT_EAR_MOTOR_B, 0, range);
            }
        }
        if (rightMotorOn) {
            if (moveRight > 0) {
                servoDispatch.setPWM(RIGHT_EAR_MOTOR_A, 0, range);
                servoDispatch.setPWM(RIGHT_EAR_MOTOR_B, 0, range - (range * abs(moveRight)));
            } else if (moveRight < 0) {
                servoDispatch.setPWM(RIGHT_EAR_MOTOR_B, 0, range);
                servoDispatch.setPWM(RIGHT_EAR_MOTOR_A, 0, range - (range * abs(moveRight)));
            } else {
                servoDispatch.setPWM(RIGHT_EAR_MOTOR_A, 0, range);
                servoDispatch.setPWM(RIGHT_EAR_MOTOR_B, 0, range);
            }
        }
        if (!leftEarStatus.isMoving()) {
            servoDispatch.setPWMFull(LEFT_EAR_MOTOR_A);
            servoDispatch.setPWMFull(LEFT_EAR_MOTOR_B);
            leftMotorOn = false;
        }
        if (!rightEarStatus.isMoving()) {
            servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_A);
            servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_B);
            rightMotorOn = false;
        }
        if (!leftEarStatus.isMoving() && !rightEarStatus.isMoving()) {
             printf("ABORT NOT MOVING\n");
             break;
        }
        uint32_t now = millis();
        if (lastTimeCheck + 100 < now) {
            lastTimeCheck = now;
            if (currentLeftPos >= targetPos - fudge && currentLeftPos <= targetPos + fudge) {
                printf("LEFT REACHED TARGET %d [%d]\n", targetPos, currentLeftPos);
                servoDispatch.setPWMFull(LEFT_EAR_MOTOR_A);
                servoDispatch.setPWMFull(LEFT_EAR_MOTOR_B);
                leftMotorOn = false;
            }
            if (currentRightPos >= targetPos - fudge && currentRightPos <= targetPos + fudge) {
                printf("RIGHT REACHED TARGET %d [%d]\n", targetPos, currentRightPos);
                servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_A);
                servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_B);
                rightMotorOn = false;
            }
        }
    }
    delay(1);
    servoDispatch.setPWMOff(LEFT_EAR_MOTOR_A);
    servoDispatch.setPWMOff(LEFT_EAR_MOTOR_B);
    servoDispatch.setPWMOff(RIGHT_EAR_MOTOR_A);
    servoDispatch.setPWMOff(RIGHT_EAR_MOTOR_B);
    if (playSound) {
        sWarblerAudio.stopMotor();
    }
}

void moveLeftEarToPosition(float pos, int fudge = 5, float speed = 1.0) {
    int targetPos = pos * 520;
    int currentPos = encoderLeftEar.getValue();
    if (currentPos >= targetPos - fudge && currentPos <= targetPos + fudge) {
        return;
    }

    TargetSteering steering(targetPos);
    steering.setDistanceTunings(1.0, 0.5, 0.5);
    steering.setSampleTime(1);

    printf("currentPos: %d targetPos: %d [%d]\n", currentPos, targetPos, abs(targetPos - currentPos));
    if (abs(targetPos - currentPos) > 100) {
        sWarblerAudio.startMotor();
    }

    EarStatus earStatus(encoderLeftEar, 5);
    uint32_t lastTimeCheck = 0;
    while (!Serial.available()) {
        sWarblerAudio.animate();
        int currentPos = encoderLeftEar.getValue();
        steering.setCurrentDistance(currentPos);
        auto move = steering.getThrottle()/* * speed*/;
        int range = 2500 * speed;
        if (move > 0) {
            servoDispatch.setPWM(LEFT_EAR_MOTOR_A, 0, range);
            servoDispatch.setPWM(LEFT_EAR_MOTOR_B, 0, range - (range * abs(move)));
        } else if (move < 0) {
            servoDispatch.setPWM(LEFT_EAR_MOTOR_B, 0, range);
            servoDispatch.setPWM(LEFT_EAR_MOTOR_A, 0, range - (range * abs(move)));
        } else {
            servoDispatch.setPWM(LEFT_EAR_MOTOR_A, 0, range);
            servoDispatch.setPWM(LEFT_EAR_MOTOR_B, 0, range);
        }
        if (!earStatus.isMoving()) {
             printf("ABORT NOT MOVING\n");
             break;
        }
        uint32_t now = millis();
        if (lastTimeCheck + 100 < now) {
            lastTimeCheck = now;
            if (currentPos >= targetPos - fudge && currentPos <= targetPos + fudge) {
                printf("REACHED TARGET %d [%d]\n", targetPos, currentPos);
                servoDispatch.setPWMFull(LEFT_EAR_MOTOR_A);
                servoDispatch.setPWMFull(LEFT_EAR_MOTOR_B);
                delay(1);
                break;
            }
        }
    }
    servoDispatch.setPWMOff(LEFT_EAR_MOTOR_A);
    servoDispatch.setPWMOff(LEFT_EAR_MOTOR_B);
    sWarblerAudio.stopMotor();
}

void moveRightEarToPosition(float pos, int fudge = 5, float speed = 1.0) {
    int targetPos = pos * 520;
    int currentPos = encoderRightEar.getValue();
    if (currentPos >= targetPos - fudge && currentPos <= targetPos + fudge) {
        return;
    }

    TargetSteering steering(targetPos);
    steering.setDistanceTunings(1.0, 0.5, 0.5);
    steering.setSampleTime(1);

    printf("currentPos: %d targetPos: %d [%d]\n", currentPos, targetPos, abs(targetPos - currentPos));
    if (abs(targetPos - currentPos) > 100) {
        sWarblerAudio.startMotor();
    }

    EarStatus earStatus(encoderRightEar, 5);
    uint32_t lastTimeCheck = 0;
    while (!Serial.available()) {
        sWarblerAudio.animate();
        int currentPos = encoderRightEar.getValue();
        steering.setCurrentDistance(currentPos);
        auto move = steering.getThrottle()/* * speed*/;
        int range = 2500 * speed;
        if (move > 0) {
            servoDispatch.setPWM(RIGHT_EAR_MOTOR_A, 0, range);
            servoDispatch.setPWM(RIGHT_EAR_MOTOR_B, 0, range - (range * abs(move)));
        } else if (move < 0) {
            servoDispatch.setPWM(RIGHT_EAR_MOTOR_B, 0, range);
            servoDispatch.setPWM(RIGHT_EAR_MOTOR_A, 0, range - (range * abs(move)));
        } else {
            servoDispatch.setPWM(RIGHT_EAR_MOTOR_A, 0, range);
            servoDispatch.setPWM(RIGHT_EAR_MOTOR_B, 0, range);
        }
        if (!earStatus.isMoving()) {
             printf("ABORT NOT MOVING\n");
             break;
        }
        uint32_t now = millis();
        if (lastTimeCheck + 100 < now) {
            lastTimeCheck = now;
            if (currentPos >= targetPos - fudge && currentPos <= targetPos + fudge) {
                printf("REACHED TARGET %d [%d]\n", targetPos, currentPos);
                servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_A);
                servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_B);
                delay(1);
                break;
            }
        }
    }
    servoDispatch.setPWMOff(RIGHT_EAR_MOTOR_A);
    servoDispatch.setPWMOff(RIGHT_EAR_MOTOR_B);
    sWarblerAudio.stopMotor();
}

bool findEarLimits() {
    bool leftMotorOn = true;
    bool rightMotorOn = true;
    encoderLeftEar.setValue(0);
    EarStatus leftEarStatus(encoderLeftEar);
    EarStatus rightEarStatus(encoderRightEar);

    servoDispatch.setPWMOff(LEFT_EAR_MOTOR_A);
    servoDispatch.setPWM(LEFT_EAR_MOTOR_B, 3200, 0);
    servoDispatch.setPWMOff(RIGHT_EAR_MOTOR_A);
    servoDispatch.setPWM(RIGHT_EAR_MOTOR_B, 3200, 0);
    while (!Serial.available() && (leftMotorOn || rightMotorOn)) {
        if (!leftEarStatus.isMoving()) {
            servoDispatch.setPWMFull(LEFT_EAR_MOTOR_A);
            servoDispatch.setPWMFull(LEFT_EAR_MOTOR_B);
            leftMotorOn = false;
            break;
        }
        if (!rightEarStatus.isMoving()) {
            servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_A);
            servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_B);
            rightMotorOn = false;
            break;
        }
    }
    servoDispatch.setPWMFull(LEFT_EAR_MOTOR_A);
    servoDispatch.setPWMFull(LEFT_EAR_MOTOR_B);
    servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_A);
    servoDispatch.setPWMFull(RIGHT_EAR_MOTOR_B);
    delay(100);
    bool moved = (encoderLeftEar.getValue() != 0 && encoderRightEar.getValue());
    encoderLeftEar.setValue(0);
    encoderRightEar.setValue(0);
    servoDispatch.setPWMOff(LEFT_EAR_MOTOR_A);
    servoDispatch.setPWMOff(LEFT_EAR_MOTOR_B);
    servoDispatch.setPWMOff(RIGHT_EAR_MOTOR_A);
    servoDispatch.setPWMOff(RIGHT_EAR_MOTOR_B);
    return moved;
}

static uint32_t nextEarMovetime;

void playSound(int num);

void setup()
{   
    REELTWO_READY();

    Wire.begin();

    mountSDFileSystem();

    pinMode(RS_RTS_PIN, OUTPUT);
    digitalWrite(RS_RTS_PIN, LOW);
    RS_SERIAL_INIT(4000000);
    EXT_SERIAL_INIT(115200);

    // if (!preferences.begin("rseries", false))
    // {
    //     DEBUG_PRINTLN("Failed to init prefs");
    // }
    printf("TESTING TESTING\n");
    SetupEvent::ready();

    leftEye.syncWith(&rightEye);

    sWarblerAudio.setVolume(21); // 0...21

    ///////////////////////////////////////////////////

    earSoundsActive = findEarLimits();
    if (!earSoundsActive) {
        DEBUG_PRINTLN("Ear motor sounds disabled");
    }

    ///////////////////////////////////////////////////

    if (getSDCardMounted())
    {
        // sWarblerAudio.queue(0, 2);
        //sWarblerAudio.play("/speech/Leia.wav");
        // sWarblerAudio.play("/music/vader-1.mp3");
        // sWarblerAudio.play(SD, "/HarlemShake.mp3");
    }
    else
    {
        DEBUG_PRINTLN("Failed to mount SD card");
    }
    DEBUG_PRINT("Total heap:  "); DEBUG_PRINTLN(ESP.getHeapSize());
    DEBUG_PRINT("Free heap:   "); DEBUG_PRINTLN(ESP.getFreeHeap());
    DEBUG_PRINT("Total PSRAM: "); DEBUG_PRINTLN(ESP.getPsramSize());
    DEBUG_PRINT("Free PSRAM:  "); DEBUG_PRINTLN(ESP.getFreePsram());
    DEBUG_PRINTLN();

    DEBUG_PRINTLN("READY");

    moveBothEarsToPosition(0.5, false);
    nextEarMovetime = millis() + 10000;
    playSound(100);
}

double currentEarPos = 0.5;
uint32_t nextSoundtime;

void playSound(int num) {
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "/bd/%d.wav", num);
    printf("PLAY: %s\n", buffer);
    sWarblerAudio.play(buffer);
    nextSoundtime = millis() + random(1000, 20000);
}

void randomSound() {
    playSound(random(1, 602));
}

void randomEarPosition() {
    double range = double(random(1, 4)) / 10 * 2;
    double newPos = currentEarPos + (random(10) < 5) ? range : -range;
    if (currentEarPos != newPos) {
        printf("current ear: %f new: %f\n", currentEarPos, newPos);
        newPos = max(min(newPos, 1.0), 0.0);
        int earMove = random(100);
        if (earMove < 10) {
            if (random(1)) {
                moveLeftEarToPosition(newPos, earSoundsActive);
            } else {
                moveRightEarToPosition(newPos, earSoundsActive);
            }
        } else {
            moveBothEarsToPosition(newPos, earSoundsActive);
        }
    }
    nextEarMovetime = millis() + random(500, 6000);
    currentEarPos = newPos;
}

void reboot()
{
    Serial.println(F("Restarting..."));
    servoDispatch.ensureEnabled();
    servoDispatch.ensureDisabled();
#ifdef ESP32
 #ifdef USE_DROID_REMOTE
    DisconnectRemote();
#endif
    unmountFileSystems();
#ifdef USE_PREFERENCES
    preferences.end();
#endif
    ESP.restart();
#else
    Serial.println(F("Restart not supported."));
#endif
}

////////////////////////////////

uint16_t calculateCRC(uint16_t crc, uint8_t data) {
    static const unsigned short sCRC[256] {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };
    return (crc << 8) ^ sCRC[((uint16_t)(crc >> 8) ^ data) & 0xFF];
}

// Function to read and parse the Dynamixel packet
bool read_and_parse_packet(Stream &stream, uint8_t id, uint8_t* data, uint16_t &dataLength) {
    static int syncHeaderCount;
    unsigned char buffer[256 + 11];
    int index = 0;

    dataLength = 0;
    if (!stream.available())
        return false;

    int byte = stream.read();
    switch (syncHeaderCount)
    {
        case 0:
        case 1:
            if (byte == 0xFF) {
                syncHeaderCount++;
                return false;
            } else {
                syncHeaderCount= 0;
            }
            break;
        case 2:
            if (byte == 0xFD) {
                syncHeaderCount++;
                return false;
            } else {
                syncHeaderCount = 0;
            }
            break;
        case 3:
            if (byte == 0xFD) {
                syncHeaderCount++;
                break;
            } else {
                syncHeaderCount = 0;
            }
            break;
    }
    syncHeaderCount = 0;
    buffer[0] = 0xFF;
    buffer[1] = 0xFF;
    buffer[2] = 0xFD;
    buffer[3] = 0xFD;
    int cmdid = stream.read();
    if (cmdid == -1) {
        printf("MISSING ID\n");
        return false;
    }
    buffer[4] = uint8_t(cmdid);
    printf("cmdid: %d\n", cmdid);

    int len_l = stream.read();
    int len_h = stream.read();
    if (len_l == -1 || len_h == -1) {
        printf("MISSING LEN\n");
        return false;
    }
    buffer[5] = uint8_t(len_l);
    buffer[6] = uint8_t(len_h);

    uint16_t crc = 0;
    for (uint16_t i = 0; i < 7; i++) {
        crc = calculateCRC(crc, buffer[i]);
    }

    int bytesRead = 0;
    int length = ((buffer[6] << 8) | buffer[5]);
    if (length + 2 >= sizeof(buffer)) {
        printf("TOO BIG\n");
        return false;
    }
    for (int i = 0; i < length; i++) {
        int byte = stream.read();
        if (byte == -1) {
            printf("MISSING DATA\n");
            return false;
        }
        data[index++] = (unsigned char)byte;
        crc = calculateCRC(crc, (unsigned char)byte);
        bytesRead++;
    }
    int crc_l = stream.read();
    int crc_h = stream.read();
    if (crc_l == -1 || crc_h == -1) {
        printf("MISSING CRC\n");
        return false;
    }
    if (crc_l == ((crc >> 0) & 0xFF) &&
        crc_h == ((crc >> 7) & 0xFF))
    {
        if (id == cmdid) {
            dataLength = length;
            return true;
        }
    }
    return false;
}

////////////////////////////////

void loop()
{
    AnimatedEvent::process();

    // uint8_t buffer[256];
    // uint16_t cmdLength;
    // if (read_and_parse_packet(RS_SERIAL, 0xFD, buffer, cmdLength)) {
    //  printf("GOT COMMAND\n");
    // }
    auto now = millis();
    if (nextEarMovetime < now) {
        randomEarPosition();

        // digitalWrite(RS_RTS_PIN, HIGH);
        // RS_SERIAL.println("EAR TIME");
        // RS_SERIAL.flush();
        // digitalWrite(RS_RTS_PIN, LOW);
    } else if (nextSoundtime < now) {
        randomSound();
    }

    if (Serial.available())
    {
        char ch = Serial.read();
        DEBUG_PRINT("RECEIVED: "); DEBUG_PRINTLN(ch);
        switch (ch)
        {
            // case 'q': {
            //  digitalWrite(PIN_EAR1_MOTOR_IN1, LOW);
            //  analogWrite(PIN_EAR1_MOTOR_IN2, 80);
            //  long nextPos = getEar1Position() - 20;
            //  while (!Serial.available() && getEar1Position() > nextPos) {
            //      delayMicroseconds(10);
            //  }
            //  analogWrite(PIN_EAR1_MOTOR_IN2, 0);
            //  digitalWrite(PIN_EAR1_MOTOR_IN2, LOW);
            //  break;
            // }
            // case 'w': {
            //  digitalWrite(PIN_EAR1_MOTOR_IN1, LOW);
            //  analogWrite(PIN_EAR1_MOTOR_IN2, 80);
            //  long nextPos = getEar1Position() - 10;
            //  while (!Serial.available() && getEar1Position() > nextPos) {
            //      delayMicroseconds(10);
            //  }
            //  analogWrite(PIN_EAR1_MOTOR_IN2, 0);
            //  digitalWrite(PIN_EAR1_MOTOR_IN2, LOW);
            //  break;
            // }
            // case 'e': {
            //  digitalWrite(PIN_EAR1_MOTOR_IN2, LOW);
            //  analogWrite(PIN_EAR1_MOTOR_IN1, 80);
            //  long nextPos = getEar1Position() + 10;
            //  while (!Serial.available() && getEar1Position() < nextPos) {
            //      delayMicroseconds(10);
            //  }
            //  analogWrite(PIN_EAR1_MOTOR_IN1, 0);
            //  digitalWrite(PIN_EAR1_MOTOR_IN1, LOW);
            //  break;
            // }
            // case 'r': {
            //  digitalWrite(PIN_EAR1_MOTOR_IN2, LOW);
            //  analogWrite(PIN_EAR1_MOTOR_IN1, 80);
            //  long nextPos = getEar1Position() + 20;
            //  while (!Serial.available() && getEar1Position() < nextPos) {
            //      delayMicroseconds(10);
            //  }
            //  analogWrite(PIN_EAR1_MOTOR_IN1, 0);
            //  digitalWrite(PIN_EAR1_MOTOR_IN1, LOW);
            //  break;
            // }
            // case 'r':
            //     sWarblerAudio.play("/speech/Leia.wav");
            //  break;
            case 'q':
                randomEarPosition();
                break;
            case 'w':
                randomSound();
                break;
            case 'e':
                playSound(100);
                break;
            case 'r':
                reboot();
                break;
            case 'z':
                encoderLeftEar.setValue(0);
                encoderRightEar.setValue(0);
                break;
            case 's':
                sWarblerAudio.stop();
                break;
            case '1':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 1, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
            case '2':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 2, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
            case '3':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 3, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
            case '4':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 4, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
            case '5':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 5, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
            case '6':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 6, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
            case '7':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 7, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
            case '8':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 8, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
            case '9':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 9, warble++);
                    if (warble > 10)
                        warble = 0;
                }
                break;
        }
    }
}
