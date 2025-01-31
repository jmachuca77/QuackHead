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
#ifdef FLASHLIGHT_RGB
#include "head/FlashLightRGB.h"
#else
#include "head/FlashLightPWM.h"
#endif

#ifdef LCD_EYES
#include "head/LCDEye.h"
#else
#include "head/InfinityEye.h"
#endif

#include "head/IMU.h"

////////////////////////////////

#include "SD.h"

////////////////////////////////

#include <Dynamixel2Arduino.h>

class DXLQuackHead: public SetupEvent {
public:
    DXLQuackHead(HardwareSerial &serial, unsigned baud, uint8_t id, unsigned rtsPin) :
        fPort(serial, rtsPin),
        fDXL(fPort, DXL_MODEL_NUM),
        fID(id),
        fBaud(baud)
    {
    }

    virtual void setup() override {
        // We must call SerialPortHandler begin but we assume that the hardware
        // serial port was already configured correctly for esp32
        fPort.begin(fBaud);

        fDXL.setPortProtocolVersion(DXL_PROTOCOL_VER_2_0);
        fDXL.setFirmwareVersion(1);
        fDXL.setID(fID);

        fDXL.addControlItem(ADDR_CONTROL_EYES, fControl_Eyes);
        fDXL.addControlItem(ADDR_CONTROL_FLASHLIGHT, fControl_Flashlight);
        fDXL.addControlItem(ADDR_CONTROL_SOUND, fControl_Sound);
        fDXL.addControlItem(ADDR_CONTROL_ANALOG, fControl_Analog);
        fDXL.addControlItem(ADDR_CONTROL_ORIENTATION, (uint8_t*)&fControl_Orientation, sizeof(fControl_Orientation));

        fDXL.setReadCallbackFunc(read_callback, this);
        fDXL.setWriteCallbackFunc(write_callback, this);
    }

    void controlEyes(uint8_t eyes);
    void controlFlashlight(uint8_t durationSec);
    void controlSound(uint8_t sound);

    int16_t readAnalog() {
        DEBUG_PRINTLN("READ ANALOG\n");
        return 42;
    }

    void process() {
        while (fPort.available()) {
            if (fDXL.processPacket() == false){
                DEBUG_PRINT("Last lib err code: ");
                DEBUG_PRINT(fDXL.getLastLibErrCode());
                DEBUG_PRINT(", ");
                DEBUG_PRINT("Last status packet err code: ");
                DEBUG_PRINT(fDXL.getLastStatusPacketError());
                DEBUG_PRINTLN();
            }
        }
    }

    void updateOrientation(const IMU::Orientation &orientation) {
        fControl_Orientation = orientation;
    }

private:
    static constexpr float DXL_PROTOCOL_VER_1_0 = 1.0;
    static constexpr float DXL_PROTOCOL_VER_2_0 = 2.0;
    static constexpr uint16_t DXL_MODEL_NUM = 0x5005;

    static constexpr uint16_t ADDR_CONTROL_EYES = 10;
    static constexpr uint16_t ADDR_CONTROL_FLASHLIGHT = 20;
    static constexpr uint16_t ADDR_CONTROL_SOUND = 30;
    static constexpr uint16_t ADDR_CONTROL_ANALOG = 40;
    static constexpr uint16_t ADDR_CONTROL_ORIENTATION = 50;

    static void write_callback(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
    {
        (void)dxl_err_code;
        DXLQuackHead* quack = ((DXLQuackHead*)arg);
        DEBUG_PRINT("WRITE: "); DEBUG_PRINTLN(item_addr);
        switch (item_addr) {
            case ADDR_CONTROL_EYES:
                quack->controlEyes(quack->fControl_Eyes);
                break;
            case ADDR_CONTROL_FLASHLIGHT:
                quack->controlFlashlight(quack->fControl_Flashlight);
                break;
            case ADDR_CONTROL_SOUND:
                quack->controlSound(quack->fControl_Sound);
                break;
            default:
                DEBUG_PRINTLN("NO IDEA");
                break;
        }
    }

    static void read_callback(uint16_t item_addr, uint8_t &dxl_err_code, void* arg)
    {
        (void)dxl_err_code;

        DXLQuackHead* quack = ((DXLQuackHead*)arg);
        switch (item_addr) {
            case ADDR_CONTROL_ANALOG:
                quack->fControl_Analog = quack->readAnalog();
                break;
            case ADDR_CONTROL_ORIENTATION:
                /* Updated once through the control loop */
                break;
        }
    }

    DYNAMIXEL::SerialPortHandler fPort;
    DYNAMIXEL::Slave fDXL;
    uint8_t fID;
    unsigned fBaud;
    uint8_t fControl_Eyes = 0;
    uint8_t fControl_Flashlight = 0;
    uint8_t fControl_Sound = 0;
    int16_t fControl_Analog = 0;
    IMU::Orientation fControl_Orientation = { };
};

////////////////////////////////
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

#ifdef LCD_EYES
int csPins[2] = { LEFT_EYE_PIN , RIGHT_EYE_PIN };
LCDEye Eyes(2,csPins);
#else
InfinityEye leftEye(LEFT_EYE_PIN);
InfinityEye rightEye(RIGHT_EYE_PIN);
#endif

#ifdef FLASHLIGHT_RGB
FlashLightRGB flashLight(FLASHLIGHT_RGB);
#else
FlashLight flashLight(servoDispatch, FLASHLIGHT_PWM);
#endif

#ifdef LEFT_EAR_ENC_A
QuadratureEncoder encoderLeftEar(LEFT_EAR_ENC_A, LEFT_EAR_ENC_B, false);
QuadratureEncoder encoderRightEar(RIGHT_EAR_ENC_A, RIGHT_EAR_ENC_B, true);
#endif

IMU imu;

////////////////////////////////

bool getSDCardMounted()
{
    return sSDCardMounted;
}

bool mountSDFileSystem()
{
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (SD.begin(SD_CS_PIN, SPI, 40000000, "/sd", MAX_OPEN_FILES/*, false*/))
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
#ifdef LEFT_EAR_ENC_A
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
#endif
}

void moveLeftEarToPosition(float pos, int fudge = 5, float speed = 1.0) {
#ifdef LEFT_EAR_ENC_A
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
#endif
}

void moveRightEarToPosition(float pos, int fudge = 5, float speed = 1.0) {
#ifdef LEFT_EAR_ENC_A
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
#endif
}

bool findEarLimits() {
#ifdef LEFT_EAR_ENC_A
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
#else
    return true;
#endif
}

static uint32_t nextEarMovetime;
static int sSoundVolume = 10; //0-21

void playSound(int num);

DXLQuackHead fQuack(RS_SERIAL, QUACKHEAD_BAUD, 6, RS_RTS_PIN);

void setup()
{   
    REELTWO_READY();

    Wire.begin();
    Wire.setClock(400000); //Set i2c frequency to 400 kHz.

    mountSDFileSystem();

    pinMode(RS_RTS_PIN, OUTPUT);
    digitalWrite(RS_RTS_PIN, LOW);
    RS_SERIAL_INIT(QUACKHEAD_BAUD);
    EXT_SERIAL_INIT(115200);

    // if (!preferences.begin("rseries", false))
    // {
    //     DEBUG_PRINTLN("Failed to init prefs");
    // }
    printf("TESTING TESTING\n");
    SetupEvent::ready();

    Eyes.reset();

    sWarblerAudio.setVolume(sSoundVolume); // 0...21

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
        // printf("current ear: %f new: %f\n", currentEarPos, newPos);
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

void DXLQuackHead::controlEyes(uint8_t eyes) {
    DEBUG_PRINT("DXL EYES: "); DEBUG_PRINTLN(eyes);
}

void DXLQuackHead::controlFlashlight(uint8_t durationSec) {
    DEBUG_PRINT("DXL FLASHLIGHT: "); DEBUG_PRINTLN(durationSec);
    flashLight.setState(true, random(1000, 4000));
}

void DXLQuackHead::controlSound(uint8_t sound) {
    DEBUG_PRINT("DXL SND: "); DEBUG_PRINTLN(sound);
    if (sWarblerAudio.isComplete()) {
        sWarblerAudio.queue(0, 0, sound);
    }
}

////////////////////////////////

void loop()
{
    AnimatedEvent::process();

    if (imu.hasOrientation()) {
        fQuack.updateOrientation(imu.orientation());
        // printf("IMU:%f %f %f %d\n",
        //     imu.x(), imu.y(), imu.z(), imu.accuracy());
    }

    // if (quack != nullptr) {
        fQuack.process();
    // }

    auto now = millis();
    if (nextEarMovetime < now) {
        randomEarPosition();

        // digitalWrite(RS_RTS_PIN, HIGH);
        // RS_SERIAL.println("EAR TIME");
        // RS_SERIAL.flush();
        // digitalWrite(RS_RTS_PIN, LOW);
    } else if (nextSoundtime < now) {
        //randomSound();
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
                sWarblerAudio.play("/music/ducktales.wav");
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
            case '+':
            case '=':
                if (sSoundVolume < 21) {
                    sSoundVolume++;
                    printf("VOL: %d\n", sSoundVolume);
                    sWarblerAudio.setVolume(sSoundVolume);
                }
                break;
            case '-':
                if (sSoundVolume > 0) {
                    sSoundVolume--;
                    printf("VOL: %d\n", sSoundVolume);
                    sWarblerAudio.setVolume(sSoundVolume);
                }
                break;
            case 'a':
                leftEye.setOnColor(127, 0, 0);
                rightEye.setOnColor(127, 0, 0);
                break;
            case 'h':
                leftEye.setOnColor(127, 127, 127);
                rightEye.setOnColor(127, 127, 127);
                break;
            case 'z':
            #ifdef LEFT_EAR_ENC_A
                encoderLeftEar.setValue(0);
                encoderRightEar.setValue(0);
            #endif
                break;
            case 's':
                sWarblerAudio.stop();
                break;
            case '0':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 0, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '1':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 1, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '2':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 2, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '3':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 3, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '4':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 4, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '5':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 5, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '6':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 6, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '7':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 7, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '8':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 8, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
            case '9':
                if (sWarblerAudio.isComplete())
                {
                    static int warble;
                    sWarblerAudio.queue(0, 9, warble++);
                    if (warble > 22)
                        warble = 0;
                }
                break;
        }
    }
}
