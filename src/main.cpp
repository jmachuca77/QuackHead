#define FASTLED_ALLOW_INTERRUPTS 1
#define FASTLED_INTERRUPT_RETRY_COUNT 1

#include <Arduino.h>
#include <Audio.h>
#include <SPI.h>
#include <FastLED.h>
#include <SD.h>
#include <Wire.h>

#if __has_include(<LittleFS.h>)
#include <LittleFS.h>
#define QH4_HAS_LITTLEFS 1
#else
#define QH4_HAS_LITTLEFS 0
#endif

#include <array>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "LedMatrixEyes.h"
#include "AudioFrequencyBitmap.h"
#include "pin_map.h"

#ifndef EXTMEM
#define EXTMEM
#endif

#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559f
#endif

using EyeDriver = eye_driver::DualLedMatrixEyes<qh4::kEyeLeftPin, qh4::kEyeRightPin>;

namespace {

constexpr uint8_t LOGO_LEFT_EYE  = 2;
constexpr uint8_t LOGO_RIGHT_EYE = 3;

constexpr uint32_t kConsoleBaud = 115200;
constexpr size_t kLineBufLen = 192;

constexpr uint32_t kEyeRefreshMs = 25;
constexpr uint32_t kHeartbeatMs = 500;
constexpr uint32_t kAntennaUpdateMs = 20;
constexpr uint32_t kAntennaKeepAliveMs = 500;
constexpr uint32_t kTorchDefaultTimeoutMs = 6000;
constexpr uint32_t kTorchI2cClockHz = 400000;

constexpr uint8_t kAntennaLeftId  = 4;
constexpr uint8_t kAntennaRightId = 5;
constexpr uint32_t kAntennaBaud = 4000000;

constexpr uint8_t kPeripheralDxlId = 10;
constexpr uint32_t kPeripheralDxlBaud = 4000000;
constexpr size_t kPeripheralDxlMaxPacket = 256;

// Tune these on-bench if the ears move the wrong direction or sit off-center.
constexpr int32_t kAntennaCenterLeft  = 2048;
constexpr int32_t kAntennaCenterRight = 2048;
constexpr int8_t  kAntennaSignLeft    = +1;
constexpr int8_t  kAntennaSignRight   = -1;
constexpr int32_t kAntennaTravelCounts = 520;
constexpr int32_t kAntennaMinCounts = 1200;
constexpr int32_t kAntennaMaxCounts = 2895;

constexpr uint32_t kProfileAcceleration = 50;
constexpr uint32_t kProfileVelocity = 120;

constexpr uint16_t kDxlAddrTorqueEnable = 64;
constexpr uint16_t kDxlAddrStatusReturnLevel = 68;
constexpr uint16_t kDxlAddrProfileAcceleration = 108;
constexpr uint16_t kDxlAddrProfileVelocity = 112;
constexpr uint16_t kDxlAddrGoalPosition = 116;

constexpr uint16_t kMcp4728AddrMin = 0x60;
constexpr uint16_t kMcp4728AddrMax = 0x67;
constexpr float kDacVddVolts = 3.30f;
constexpr float kTorchDimMinVolts = 0.30f;
constexpr float kTorchDimMaxVolts = 2.50f;

constexpr uint32_t kAudioPcmRate = 44100;
constexpr float kAudioLampGain = 0.80f;
constexpr float kAudioAuxGain = 0.75f;
constexpr float kAudioWhineMaxGain = 0.16f;
constexpr float kAudioWhineMinHz = 210.0f;
constexpr float kAudioWhineMaxHz = 650.0f;
constexpr float kAudioAntennaLoopMaxGain = 0.38f;
constexpr float kAudioAntennaLoopStartThresh = 0.05f;
constexpr uint32_t kAntennaLoopRetryMs = 2000;
constexpr size_t kLampCacheCapacitySamples = kAudioPcmRate * 12;   // 12 seconds mono 16-bit
constexpr size_t kAuxCacheCapacitySamples  = kAudioPcmRate * 8;    // 8 seconds mono 16-bit
constexpr size_t kAntennaLoopCapacitySamples = kAudioPcmRate * 6;  // 6 seconds mono 16-bit

constexpr char kFlashConfigPath[] = "/cfg/runtime.bin";
constexpr char kFlashAssetDir[] = "/assets";
constexpr char kFlashKvDir[] = "/kv";
constexpr char kFlashDefaultAntennaLoopAsset[] = "/antenna.wav";
constexpr char kFlashWarblerPodAsset[] = "/assets/warbler.pod";

constexpr uint16_t kDxlControlTableSize = 160;
constexpr uint16_t kModelNumber = 0x5148;  // 'QH'
constexpr uint8_t kFirmwareVersion = 2;
constexpr uint8_t kStatusReturnLevelDefault = 2;

// Custom Dynamixel control table for the Teensy peripheral on UART1.
enum ControlAddr : uint16_t {
  ADDR_MODEL_NUMBER = 0,          // uint16 RO
  ADDR_FIRMWARE_VERSION = 2,      // uint8  RO
  ADDR_ID = 3,                    // uint8  RO
  ADDR_BAUDRATE = 4,              // uint8  RO, 4M preset code
  ADDR_STATUS_RETURN_LEVEL = 5,   // uint8  RO

  ADDR_FLASHLIGHT = 64,           // uint16 RW, 0..1000 level, 0=off
  ADDR_FLASHLIGHT_TIMEOUT_MS = 66,// uint32 RW
  ADDR_SETVOLUME = 70,            // uint16 RW, 0..1000 master volume
  ADDR_PLAYSOUND_INDEX = 72,      // uint16 RW command register, auto-clears to 0
  ADDR_PLAYSOUND_ARG0 = 74,       // uint32 RW optional argument used by indexed sounds
  ADDR_ANTENNA_MODE = 78,         // uint8 RW, enum below
  ADDR_ANTENNA_CLIP = 79,         // uint8 RW command register, auto-clears to 0
  ADDR_ANTENNA_LEFT = 80,         // int16 RW, -1000..1000 manual override
  ADDR_ANTENNA_RIGHT = 82,        // int16 RW, -1000..1000 manual override
  ADDR_ANTENNA_BOTH = 84,         // int16 RW, -1000..1000 manual override
  ADDR_EYE_MODE = 86,             // uint8 RW, 0=normal 1=on 2=off
  ADDR_EYE_EFFORT = 88,           // uint16 RW, 0..1000
  ADDR_EYE_BRIGHTNESS = 90,       // uint8 RW, 0..255

  ADDR_STATUS_FLAGS = 92,         // uint32 RO bitfield
  ADDR_TORCH_REMAINING_MS = 96,   // uint32 RO
  ADDR_PRESENT_ANTENNA_LEFT = 100,// int32  RO goal counts
  ADDR_PRESENT_ANTENNA_RIGHT = 104,// int32 RO goal counts
  ADDR_ACTIVE_SOUND_INDEX = 108,  // uint16 RO, currently active AUX sound index
  ADDR_AUDIO_FLAGS = 110,         // uint8  RO
  ADDR_FLASH_OP = 112,            // uint8  RW command register, auto-clears to 0
  ADDR_FLASH_FLAGS = 113,         // uint8  RO
  ADDR_FLASH_USED_KB = 116,       // uint32 RO
  ADDR_FLASH_TOTAL_KB = 120,      // uint32 RO
  ADDR_FLASH_RESULT = 124,        // uint8  RO
};

enum StatusFlags : uint32_t {
  kStatusTorchReady    = 1u << 0,
  kStatusTorchOn       = 1u << 1,
  kStatusSdReady       = 1u << 2,
  kStatusLampPlaying   = 1u << 3,
  kStatusAuxPlaying    = 1u << 4,
  kStatusAntennaReady  = 1u << 5,
  kStatusUsbDualSerial = 1u << 6,
  kStatusFlashReady    = 1u << 7,
  kStatusAntennaLoop   = 1u << 8,
};

enum AudioFlags : uint8_t {
  kAudioFlagLampCached        = 1u << 0,
  kAudioFlagAuxCached         = 1u << 1,
  kAudioFlagAntennaLoopCached = 1u << 2,
  kAudioFlagAntennaLoopFlash  = 1u << 3,
  kAudioFlagWarblerPodLoaded  = 1u << 4,
  kAudioFlagWarblerPodFlash   = 1u << 5,
};

enum FlashFlags : uint8_t {
  kFlashFlagFsReady           = 1u << 0,
  kFlashFlagConfigSaved       = 1u << 1,
  kFlashFlagAntennaLoopCached = 1u << 2,
  kFlashFlagAntennaLoopFlash  = 1u << 3,
  kFlashFlagAntennaLoopAsset  = 1u << 4,
  kFlashFlagWarblerPodAsset   = 1u << 5,
};

enum FlashOpCode : uint8_t {
  kFlashOpNone = 0,
  kFlashOpSaveConfig = 1,
  kFlashOpLoadConfig = 2,
  kFlashOpEraseConfig = 3,
  kFlashOpImportAntennaLoop = 4,
  kFlashOpClearAntennaLoopCache = 5,
};

enum FlashResultCode : uint8_t {
  kFlashResultOk = 0,
  kFlashResultNoFlash = 1,
  kFlashResultNotFound = 2,
  kFlashResultIoError = 3,
  kFlashResultBadOp = 4,
  kFlashResultBadData = 5,
};

enum class StorageSource : uint8_t {
  kNone = 0,
  kSd = 1,
  kFlash = 2,
};

constexpr uint32_t kPersistMagic = 0x51484346u;  // "QHCF"
constexpr uint16_t kPersistVersion = 1u;

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int16_t clampi16(int32_t v, int16_t lo, int16_t hi) {
  if (v < (int32_t)lo) return lo;
  if (v > (int32_t)hi) return hi;
  return (int16_t)v;
}

static inline int32_t clampi32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float randf(float lo, float hi) {
  const long r = random(0L, 1000000L);
  const float u = (float)r / 1000000.0f;
  return lo + ((hi - lo) * u);
}

template <typename T>
void printlnBoth(const T& msg) {
  Serial.println(msg);
#if defined(USB_DUAL_SERIAL_AUDIO)
  SerialUSB1.println(msg);
#endif
}

template <typename T>
void printBoth(const T& msg) {
  Serial.print(msg);
#if defined(USB_DUAL_SERIAL_AUDIO)
  SerialUSB1.print(msg);
#endif
}

namespace dxl {

static inline void writeLe16(uint8_t* dst, uint16_t value) {
  dst[0] = (uint8_t)(value & 0xFFu);
  dst[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static inline void writeLe32(uint8_t* dst, uint32_t value) {
  dst[0] = (uint8_t)(value & 0xFFu);
  dst[1] = (uint8_t)((value >> 8) & 0xFFu);
  dst[2] = (uint8_t)((value >> 16) & 0xFFu);
  dst[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static inline uint16_t readLe16(const uint8_t* src) {
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static inline uint32_t readLe32(const uint8_t* src) {
  return (uint32_t)src[0] |
         ((uint32_t)src[1] << 8) |
         ((uint32_t)src[2] << 16) |
         ((uint32_t)src[3] << 24);
}

static uint16_t updateCrc(uint16_t crc, const uint8_t* data, size_t len) {
  for (size_t j = 0; j < len; ++j) {
    crc ^= (uint16_t)data[j] << 8;
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 0x8000u) {
        crc = (uint16_t)((crc << 1) ^ 0x8005u);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

}  // namespace dxl

struct DxlBulkWriteEntry {
  uint8_t id = 0;
  uint16_t addr = 0;
  uint16_t len = 0;
  const uint8_t* data = nullptr;
};

class DynamixelTtlMasterBus {
 public:
  DynamixelTtlMasterBus(HardwareSerial& port, uint8_t txen_pin)
      : port_(port), txen_pin_(txen_pin) {}

  void begin(uint32_t baud) {
    pinMode(txen_pin_, OUTPUT);
    digitalWrite(txen_pin_, LOW);
    port_.begin(baud);
    flushInput();
  }

  void flushInput() {
    while (port_.available() > 0) {
      port_.read();
    }
  }

  bool writeInstruction(uint8_t id, uint8_t instruction, const uint8_t* params, size_t param_len) {
    if (param_len > 220) return false;

    uint8_t packet[256];
    size_t n = 0;
    packet[n++] = 0xFF;
    packet[n++] = 0xFF;
    packet[n++] = 0xFD;
    packet[n++] = 0x00;
    packet[n++] = id;

    const uint16_t length = (uint16_t)(param_len + 3u);
    packet[n++] = (uint8_t)(length & 0xFFu);
    packet[n++] = (uint8_t)((length >> 8) & 0xFFu);
    packet[n++] = instruction;

    for (size_t i = 0; i < param_len; ++i) {
      packet[n++] = params[i];
    }

    const uint16_t crc = dxl::updateCrc(0, packet, n);
    packet[n++] = (uint8_t)(crc & 0xFFu);
    packet[n++] = (uint8_t)((crc >> 8) & 0xFFu);

    digitalWrite(txen_pin_, HIGH);
    delayMicroseconds(2);
    port_.write(packet, n);
    port_.flush();
    delayMicroseconds(2);
    digitalWrite(txen_pin_, LOW);
    return true;
  }

  bool bulkWrite(const DxlBulkWriteEntry* entries, size_t count) {
    if (entries == nullptr || count == 0) return false;

    uint8_t params[220];
    size_t p = 0;
    for (size_t i = 0; i < count; ++i) {
      const DxlBulkWriteEntry& e = entries[i];
      if (e.data == nullptr || e.len == 0) continue;
      if ((p + 5u + e.len) > sizeof(params)) return false;
      params[p++] = e.id;
      dxl::writeLe16(&params[p], e.addr);
      p += 2;
      dxl::writeLe16(&params[p], e.len);
      p += 2;
      memcpy(&params[p], e.data, e.len);
      p += e.len;
    }

    if (p == 0) return false;
    return writeInstruction(0xFE, 0x93, params, p);
  }

 private:
  HardwareSerial& port_;
  uint8_t txen_pin_;
};

enum class TorchPdMode : uint8_t {
  kNormal = 0b00,
  k1kToGnd = 0b01,
  k100kToGnd = 0b10,
  k500kToGnd = 0b11,
};

struct TorchChannelState {
  uint16_t code = 0;
  bool use_internal_vref = false;
  bool gain_2x = false;
  TorchPdMode pd = TorchPdMode::kNormal;
};

class TorchDriver {
 public:
  bool begin(TwoWire& wire = Wire) {
    wire_ = &wire;
    wire_->begin();
    wire_->setClock(kTorchI2cClockHz);
    address_ = 0;
    for (uint8_t addr = kMcp4728AddrMin; addr <= kMcp4728AddrMax; ++addr) {
      wire_->beginTransmission(addr);
      if (wire_->endTransmission() == 0) {
        address_ = addr;
        break;
      }
    }
    if (!ready()) return false;
    setOff();
    return true;
  }

  bool ready() const { return (wire_ != nullptr) && (address_ != 0); }
  uint8_t address() const { return address_; }
  float level() const { return level_; }
  bool isOn() const { return level_ > 0.0f; }

  bool turnOn(float level01 = 1.0f, uint32_t timeout_ms = kTorchDefaultTimeoutMs) {
    if (!setLevel(level01)) return false;
    deadline_ms_ = (timeout_ms == 0) ? 0u : (millis() + timeout_ms);
    return true;
  }

  bool setLevel(float level01) {
    if (!ready()) return false;
    level01 = clampf(level01, 0.0f, 1.0f);
    level_ = level01;
    if (level01 <= 0.0f) {
      return writeFrame(makeAllOffFrame());
    }

    std::array<TorchChannelState, 4> frame = makeAllOffFrame();
    TorchChannelState& white = frame[3];
    white.pd = TorchPdMode::kNormal;
    white.code = flashlightLevelToCode(level01);
    return writeFrame(frame);
  }

  bool setOff() {
    deadline_ms_ = 0;
    level_ = 0.0f;
    if (!ready()) return false;
    return writeFrame(makeAllOffFrame());
  }

  void service(uint32_t now_ms) {
    if (deadline_ms_ != 0 && now_ms >= deadline_ms_) {
      setOff();
    }
  }

  uint32_t timeoutRemainingMs(uint32_t now_ms) const {
    if (deadline_ms_ == 0 || now_ms >= deadline_ms_) return 0;
    return deadline_ms_ - now_ms;
  }

 private:
  static uint16_t voltsToCode(float volts) {
    volts = clampf(volts, 0.0f, kDacVddVolts);
    return (uint16_t)lroundf((volts / kDacVddVolts) * 4095.0f);
  }

  static uint16_t flashlightLevelToCode(float pwr) {
    pwr = clampf(pwr, 0.0f, 1.0f);
    if (pwr <= 0.0f) return 0;
    const float volts = kTorchDimMinVolts + ((kTorchDimMaxVolts - kTorchDimMinVolts) * pwr);
    return voltsToCode(volts);
  }

  static TorchChannelState makeOffState() {
    TorchChannelState s;
    s.code = 0;
    s.use_internal_vref = false;
    s.gain_2x = false;
    s.pd = TorchPdMode::k1kToGnd;
    return s;
  }

  static std::array<TorchChannelState, 4> makeAllOffFrame() {
    std::array<TorchChannelState, 4> frame{};
    for (TorchChannelState& s : frame) {
      s = makeOffState();
    }
    return frame;
  }

  static uint8_t buildMultiWriteCommand(uint8_t channel, bool update_now) {
    const uint8_t udac = update_now ? 0U : 1U;
    return (uint8_t)(0x40U | ((channel & 0x03U) << 1U) | udac);
  }

  static uint8_t buildMsb(const TorchChannelState& s) {
    const uint16_t code = (uint16_t)(s.code & 0x0FFFU);
    return (uint8_t)((s.use_internal_vref ? 0x80U : 0x00U) |
                     (((uint8_t)s.pd & 0x03U) << 5U) |
                     (s.gain_2x ? 0x10U : 0x00U) |
                     ((code >> 8U) & 0x0FU));
  }

  static uint8_t buildLsb(const TorchChannelState& s) {
    return (uint8_t)(s.code & 0xFFU);
  }

  bool writeFrame(const std::array<TorchChannelState, 4>& frame) {
    if (!ready()) return false;
    wire_->beginTransmission(address_);
    for (uint8_t ch = 0; ch < 4; ++ch) {
      const TorchChannelState& s = frame[ch];
      wire_->write(buildMultiWriteCommand(ch, true));
      wire_->write(buildMsb(s));
      wire_->write(buildLsb(s));
    }
    return wire_->endTransmission() == 0;
  }

  TwoWire* wire_ = nullptr;
  uint8_t address_ = 0;
  float level_ = 0.0f;
  uint32_t deadline_ms_ = 0;
};

class AntennaShow {
 public:
  enum BaseMode : uint8_t {
    kOff = 0,
    kIdle = 1,
    kExcited = 2,
    kHappy = 3,
    kScared = 4,
    kAnxious = 5,
    kAngry = 6,
    kCurious = 7,
    kRetract = 8,
  };

  enum Clip : uint8_t {
    kNone = 0,
    kYes = 1,
    kNo = 2,
    kScan = 3,
  };

  void begin(uint32_t now_ms) { reset(now_ms); }

  void reset(uint32_t now_ms) {
    base_mode_ = kIdle;
    prev_mode_ = kIdle;
    base_change_ms_ = now_ms;
    clip_ = kNone;
    clip_start_ms_ = now_ms;
    clip_duration_ms_ = 0;
    manual_left_set_ = false;
    manual_right_set_ = false;
    manual_left_ = 0.0f;
    manual_right_ = 0.0f;

    idle_phase_l_ = randf(0.0f, TWO_PI);
    idle_phase_r_ = randf(0.0f, TWO_PI);
    idle_phase2_l_ = randf(0.0f, TWO_PI);
    idle_phase2_r_ = randf(0.0f, TWO_PI);
    next_twitch_ms_[0] = now_ms + (uint32_t)lroundf(randf(800.0f, 2000.0f));
    next_twitch_ms_[1] = now_ms + (uint32_t)lroundf(randf(800.0f, 2000.0f));
    twitch_active_[0] = false;
    twitch_active_[1] = false;
    left_norm_ = 0.0f;
    right_norm_ = 0.0f;
  }

  const char* baseModeName() const {
    switch (base_mode_) {
      case kOff: return "off";
      case kIdle: return "idle";
      case kExcited: return "excited";
      case kHappy: return "happy";
      case kScared: return "scared";
      case kAnxious: return "anxious";
      case kAngry: return "angry";
      case kCurious: return "curious";
      case kRetract: return "retract";
      default: return "idle";
    }
  }

  const char* clipName() const {
    switch (clip_) {
      case kNone: return "none";
      case kYes: return "yes";
      case kNo: return "no";
      case kScan: return "scan";
      default: return "none";
    }
  }

  BaseMode baseModeCode() const { return base_mode_; }
  Clip activeClipCode() const { return clip_; }

  static BaseMode parseBaseMode(const char* mode) {
    if (mode == nullptr) return kIdle;
    if (strcmp(mode, "off") == 0) return kOff;
    if (strcmp(mode, "idle") == 0) return kIdle;
    if (strcmp(mode, "excited") == 0) return kExcited;
    if (strcmp(mode, "happy") == 0) return kHappy;
    if (strcmp(mode, "scared") == 0) return kScared;
    if (strcmp(mode, "anxious") == 0) return kAnxious;
    if (strcmp(mode, "angry") == 0) return kAngry;
    if (strcmp(mode, "curious") == 0) return kCurious;
    if (strcmp(mode, "retract") == 0) return kRetract;
    return kIdle;
  }

  static Clip parseClip(const char* clip) {
    if (clip == nullptr) return kNone;
    if (strcmp(clip, "yes") == 0) return kYes;
    if (strcmp(clip, "no") == 0) return kNo;
    if (strcmp(clip, "scan") == 0) return kScan;
    return kNone;
  }

  void clearManual() {
    manual_left_set_ = false;
    manual_right_set_ = false;
  }

  void setManualLeft(float v_norm) {
    manual_left_ = clampf(v_norm, -1.0f, 1.0f);
    manual_left_set_ = true;
  }

  void setManualRight(float v_norm) {
    manual_right_ = clampf(v_norm, -1.0f, 1.0f);
    manual_right_set_ = true;
  }

  void setManualBoth(float v_norm) {
    const float v = clampf(v_norm, -1.0f, 1.0f);
    manual_left_ = v;
    manual_right_ = v;
    manual_left_set_ = true;
    manual_right_set_ = true;
  }

  void setBaseMode(BaseMode mode, uint32_t now_ms) {
    prev_mode_ = base_mode_;
    base_mode_ = mode;
    base_change_ms_ = now_ms;
  }

  void setBaseMode(const char* mode, uint32_t now_ms) { setBaseMode(parseBaseMode(mode), now_ms); }

  void triggerClip(Clip clip, uint32_t now_ms) {
    if (clip == kNone) return;
    clip_ = clip;
    clip_start_ms_ = now_ms;
    clip_duration_ms_ = (clip == kScan) ? 1200U : 900U;
  }

  void triggerClip(const char* name, uint32_t now_ms) { triggerClip(parseClip(name), now_ms); }

  void tick(uint32_t now_ms, float speed_frac = 0.0f) {
    const float t = now_ms * 0.001f;
    float left = 0.0f;
    float right = 0.0f;
    evalBase(t, now_ms, left, right);

    if (clip_ != kNone) {
      float clip_l = 0.0f;
      float clip_r = 0.0f;
      float a = 0.0f;
      evalClip(t, now_ms, clip_l, clip_r, a);
      if (a <= 0.0f) {
        clip_ = kNone;
      } else {
        left = ((1.0f - a) * left) + (a * clip_l);
        right = ((1.0f - a) * right) + (a * clip_r);
      }
    }

    const float sf = clampf(speed_frac, 0.0f, 1.0f);
    if (sf > 0.70f) {
      const float f = clampf((sf - 0.70f) / 0.30f, 0.0f, 1.0f);
      const float duck_back = -0.45f * f;
      const float tremble = 0.03f * sinf(TWO_PI * 8.0f * t);
      left += duck_back + tremble;
      right += duck_back + tremble;
    }

    if (manual_left_set_) left = manual_left_;
    if (manual_right_set_) right = manual_right_;

    left_norm_ = clampf(left, -1.0f, 1.0f);
    right_norm_ = clampf(right, -1.0f, 1.0f);
  }

  float leftNorm() const { return left_norm_; }
  float rightNorm() const { return right_norm_; }

  int32_t leftGoal() const { return normToCounts(left_norm_, kAntennaCenterLeft, kAntennaSignLeft); }
  int32_t rightGoal() const { return normToCounts(right_norm_, kAntennaCenterRight, kAntennaSignRight); }

 private:
  static int32_t normToCounts(float v_norm, int32_t center, int8_t sign) {
    const float clamped = clampf(v_norm, -1.0f, 1.0f);
    const int32_t pos = center + (int32_t)lroundf((float)sign * clamped * (float)kAntennaTravelCounts);
    return clampi32(pos, kAntennaMinCounts, kAntennaMaxCounts);
  }

  void maybeStartTwitch(uint8_t side, uint32_t now_ms) {
    if (twitch_active_[side]) return;
    if (now_ms < next_twitch_ms_[side]) return;
    twitch_active_[side] = true;
    twitch_start_ms_[side] = now_ms;
    twitch_dur_ms_[side] = (uint32_t)lroundf(randf(250.0f, 600.0f));
    twitch_amp_[side] = randf(-0.18f, 0.18f);
    next_twitch_ms_[side] = now_ms + (uint32_t)lroundf(randf(2000.0f, 6000.0f));
  }

  float twitchContribution(uint8_t side, uint32_t now_ms) {
    maybeStartTwitch(side, now_ms);
    if (!twitch_active_[side]) return 0.0f;
    const uint32_t dt_ms = now_ms - twitch_start_ms_[side];
    if (dt_ms >= twitch_dur_ms_[side]) {
      twitch_active_[side] = false;
      return 0.0f;
    }
    const float u = (float)dt_ms / (float)twitch_dur_ms_[side];
    return twitch_amp_[side] * sinf(PI * u);
  }

  void pattern(BaseMode mode, float t, uint32_t now_ms, float& left, float& right) {
    switch (mode) {
      case kOff:
        left = 0.0f;
        right = 0.0f;
        return;
      case kRetract:
        left = -1.0f;
        right = -1.0f;
        return;
      case kHappy:
        left = 0.20f * sinf((TWO_PI * 4.5f * t) + idle_phase_l_);
        right = 0.20f * sinf((TWO_PI * 4.8f * t) + idle_phase_r_);
        left += 0.06f * sinf((TWO_PI * 8.0f * t) + idle_phase2_l_);
        right += 0.06f * sinf((TWO_PI * 7.5f * t) + idle_phase2_r_);
        return;
      case kAnxious:
        left = -0.45f + (0.09f * sinf((TWO_PI * 9.0f * t) + idle_phase_l_));
        right = -0.45f + (0.09f * sinf((TWO_PI * 8.5f * t) + idle_phase_r_));
        left += 0.04f * sinf((TWO_PI * 13.0f * t) + idle_phase2_l_);
        right += 0.04f * sinf((TWO_PI * 14.0f * t) + idle_phase2_r_);
        return;
      case kAngry:
        left = 0.35f + (0.10f * sinf((TWO_PI * 7.0f * t) + idle_phase_l_));
        right = 0.35f + (0.10f * sinf((TWO_PI * 7.5f * t) + idle_phase_r_));
        left += 0.04f * sinf((TWO_PI * 17.0f * t) + idle_phase2_l_);
        right += 0.04f * sinf((TWO_PI * 16.0f * t) + idle_phase2_r_);
        return;
      case kExcited:
        left = 0.28f * sinf((TWO_PI * 6.0f * t) + idle_phase_l_);
        right = 0.28f * sinf((TWO_PI * 6.5f * t) + idle_phase_r_);
        left += 0.08f * sinf((TWO_PI * 11.0f * t) + idle_phase2_l_);
        right += 0.08f * sinf((TWO_PI * 10.0f * t) + idle_phase2_r_);
        return;
      case kScared:
        left = -0.80f + (0.06f * sinf((TWO_PI * 12.0f * t) + idle_phase_l_));
        right = -0.80f + (0.03f * sinf((TWO_PI * 18.0f * t) + idle_phase_r_));
        return;
      case kCurious:
        left = 0.60f + (0.10f * sinf((TWO_PI * 1.2f * t) + idle_phase_l_));
        right = 0.60f - (0.10f * sinf((TWO_PI * 1.0f * t) + idle_phase_r_));
        return;
      case kIdle:
      default:
        left = 0.08f * sinf((TWO_PI * 0.13f * t) + idle_phase_l_);
        right = 0.08f * sinf((TWO_PI * 0.11f * t) + idle_phase_r_);
        left += 0.05f * sinf((TWO_PI * 0.05f * t) + idle_phase2_l_);
        right += 0.05f * sinf((TWO_PI * 0.06f * t) + idle_phase2_r_);
        left += twitchContribution(0, now_ms);
        right += twitchContribution(1, now_ms);
        return;
    }
  }

  void evalBase(float t, uint32_t now_ms, float& left, float& right) {
    const uint32_t dt_ms = now_ms - base_change_ms_;
    if (dt_ms >= 100 || prev_mode_ == base_mode_) {
      pattern(base_mode_, t, now_ms, left, right);
      return;
    }

    float prev_l = 0.0f;
    float prev_r = 0.0f;
    float next_l = 0.0f;
    float next_r = 0.0f;
    pattern(prev_mode_, t, now_ms, prev_l, prev_r);
    pattern(base_mode_, t, now_ms, next_l, next_r);
    const float a = clampf((float)dt_ms / 100.0f, 0.0f, 1.0f);
    left = ((1.0f - a) * prev_l) + (a * next_l);
    right = ((1.0f - a) * prev_r) + (a * next_r);
  }

  void evalClip(float t, uint32_t now_ms, float& left, float& right, float& alpha) {
    const uint32_t dt_ms = now_ms - clip_start_ms_;
    if (clip_duration_ms_ == 0 || dt_ms >= clip_duration_ms_) {
      left = 0.0f;
      right = 0.0f;
      alpha = 0.0f;
      return;
    }

    const float u = (float)dt_ms / (float)clip_duration_ms_;
    alpha = 0.5f - (0.5f * cosf(TWO_PI * u));

    if (clip_ == kScan) {
      const float sweep = 0.65f * sinf(TWO_PI * 0.65f * t);
      left = sweep;
      right = -sweep;
      return;
    }

    const float pulse = sinf(TWO_PI * 3.0f * t);
    const float amp = 0.35f * pulse;
    if (clip_ == kNo) {
      left = amp;
      right = -amp;
      return;
    }

    left = amp;
    right = amp;
  }

  BaseMode base_mode_ = kIdle;
  BaseMode prev_mode_ = kIdle;
  uint32_t base_change_ms_ = 0;

  Clip clip_ = kNone;
  uint32_t clip_start_ms_ = 0;
  uint32_t clip_duration_ms_ = 0;

  bool manual_left_set_ = false;
  bool manual_right_set_ = false;
  float manual_left_ = 0.0f;
  float manual_right_ = 0.0f;

  float idle_phase_l_ = 0.0f;
  float idle_phase_r_ = 0.0f;
  float idle_phase2_l_ = 0.0f;
  float idle_phase2_r_ = 0.0f;

  uint32_t next_twitch_ms_[2] = {0, 0};
  bool twitch_active_[2] = {false, false};
  uint32_t twitch_start_ms_[2] = {0, 0};
  uint32_t twitch_dur_ms_[2] = {0, 0};
  float twitch_amp_[2] = {0.0f, 0.0f};

  float left_norm_ = 0.0f;
  float right_norm_ = 0.0f;
};


#if QH4_HAS_LITTLEFS
using Qh4FlashFsType = LittleFS_QSPIFlash;  // Change to LittleFS_QPINAND if your populated bottom-side flash is NAND.
extern Qh4FlashFsType gFlashFs;
#endif

struct RuntimePersistedConfig {
  uint32_t magic = kPersistMagic;
  uint16_t version = kPersistVersion;
  uint16_t reserved0 = 0;
  uint32_t flashlight_timeout_ms = kTorchDefaultTimeoutMs;
  uint16_t volume_raw = 850;
  uint8_t antenna_mode = (uint8_t)AntennaShow::kIdle;
  uint8_t eye_mode = 0;
  uint16_t eye_effort = 0;
  uint8_t eye_brightness = eye_driver::kDefaultBrightness;
  uint8_t reserved1[7] = {0};
};

class FlashManager {
 public:
  bool begin() {
#if QH4_HAS_LITTLEFS
    ready_ = gFlashFs.begin();
    if (ready_) {
      gFlashFs.mkdir(kFlashKvDir);
      gFlashFs.mkdir(kFlashAssetDir);
      gFlashFs.mkdir("/cfg");
    }
#else
    ready_ = false;
#endif
    return ready_;
  }

  bool ready() const { return ready_; }

  uint32_t usedKilobytes() const {
#if QH4_HAS_LITTLEFS
    return ready_ ? (uint32_t)(gFlashFs.usedSize() / 1024u) : 0u;
#else
    return 0u;
#endif
  }

  uint32_t totalKilobytes() const {
#if QH4_HAS_LITTLEFS
    return ready_ ? (uint32_t)(gFlashFs.totalSize() / 1024u) : 0u;
#else
    return 0u;
#endif
  }

  bool existsPath(const char* path) const {
#if QH4_HAS_LITTLEFS
    if (!ready_ || path == nullptr || path[0] == 0) return false;
    File file = gFlashFs.open(path, FILE_READ);
    if (!file) return false;
    file.close();
    return true;
#else
    (void)path;
    return false;
#endif
  }

  bool openReadPath(const char* path, File& file) const {
#if QH4_HAS_LITTLEFS
    if (!ready_ || path == nullptr || path[0] == 0) return false;
    file = gFlashFs.open(path, FILE_READ);
    return (bool)file;
#else
    (void)path;
    (void)file;
    return false;
#endif
  }

  bool configExists() const { return existsPath(kFlashConfigPath); }

  bool saveRuntimeConfig(const RuntimePersistedConfig& cfg) {
    return writeBuffer(kFlashConfigPath, (const uint8_t*)&cfg, sizeof(cfg));
  }

  bool loadRuntimeConfig(RuntimePersistedConfig& cfg) const {
    File file;
    if (!openReadPath(kFlashConfigPath, file)) return false;
    const size_t expected = sizeof(cfg);
    const int got = file.read((uint8_t*)&cfg, expected);
    file.close();
    if (got != (int)expected) return false;
    if (cfg.magic != kPersistMagic || cfg.version != kPersistVersion) return false;
    return true;
  }

  bool eraseRuntimeConfig() {
#if QH4_HAS_LITTLEFS
    return ready_ ? gFlashFs.remove(kFlashConfigPath) : false;
#else
    return false;
#endif
  }

  bool removePath(const char* path) {
#if QH4_HAS_LITTLEFS
    if (!ready_ || path == nullptr || path[0] == 0) return false;
    return gFlashFs.remove(path);
#else
    (void)path;
    return false;
#endif
  }

  bool writeTextKey(const char* key, const char* value) {
    char path[96];
    if (!makeKeyPath(key, path, sizeof(path))) return false;
    const char* safe_value = (value != nullptr) ? value : "";
    return writeBuffer(path, (const uint8_t*)safe_value, strlen(safe_value));
  }

  bool printTextKey(const char* key, Print& out) const {
    char path[96];
    if (!makeKeyPath(key, path, sizeof(path))) return false;
    File file;
    if (!openReadPath(path, file)) return false;
    while (file.available() > 0) {
      const int raw = file.read();
      if (raw < 0) break;
      out.write((uint8_t)raw);
    }
    file.close();
    out.println();
    return true;
  }

  bool deleteTextKey(const char* key) {
#if QH4_HAS_LITTLEFS
    char path[96];
    if (!makeKeyPath(key, path, sizeof(path))) return false;
    return ready_ ? gFlashFs.remove(path) : false;
#else
    (void)key;
    return false;
#endif
  }

  bool copySdFileToAsset(const char* sd_path, const char* asset_name) {
#if QH4_HAS_LITTLEFS
    if (!ready_ || sd_path == nullptr || sd_path[0] == 0) return false;

    char flash_path[96];
    if (!makeAssetPath(asset_name, flash_path, sizeof(flash_path))) return false;

    File src = SD.open(sd_path, FILE_READ);
    if (!src) return false;

    gFlashFs.remove(flash_path);
    File dst = gFlashFs.open(flash_path, FILE_WRITE);
    if (!dst) {
      src.close();
      return false;
    }

    const bool ok = copyFile(src, dst);
    src.close();
    dst.close();
    return ok;
#else
    (void)sd_path;
    (void)asset_name;
    return false;
#endif
  }

  bool copyAssetToSd(const char* asset_name, const char* sd_path) const {
#if QH4_HAS_LITTLEFS
    if (!ready_ || sd_path == nullptr || sd_path[0] == 0) return false;

    char flash_path[96];
    if (!makeAssetPath(asset_name, flash_path, sizeof(flash_path))) return false;

    File src;
    if (!openReadPath(flash_path, src)) return false;

    SD.remove(sd_path);
    File dst = SD.open(sd_path, FILE_WRITE);
    if (!dst) {
      src.close();
      return false;
    }

    const bool ok = copyFile(src, dst);
    src.close();
    dst.close();
    return ok;
#else
    (void)asset_name;
    (void)sd_path;
    return false;
#endif
  }

  void list(Print& out) const {
#if QH4_HAS_LITTLEFS
    if (!ready_) {
      out.println(F("flash: unavailable"));
      return;
    }

    out.print(F("flash.used_kb="));
    out.print(usedKilobytes());
    out.print(F(" flash.total_kb="));
    out.println(totalKilobytes());

    listDirectory(kFlashKvDir, out, "kv");
    listDirectory(kFlashAssetDir, out, "assets");
    if (configExists()) {
      out.println(F("cfg/runtime.bin"));
    } else {
      out.println(F("cfg: <none>"));
    }
#else
    out.println(F("flash: unavailable"));
#endif
  }

  static bool makeAssetPath(const char* asset_name, char* out, size_t out_size) {
    char clean[64];
    if (!sanitizeLeafName(asset_name, clean, sizeof(clean))) return false;
    return snprintf(out, out_size, "%s/%s", kFlashAssetDir, clean) > 0;
  }

 private:
  static bool sanitizeLeafName(const char* in, char* out, size_t out_size) {
    if (in == nullptr || in[0] == 0 || out == nullptr || out_size < 4u) return false;

    size_t n = 0;
    while (*in != 0 && n + 1 < out_size) {
      const char c = *in++;
      const bool ok = ((c >= 'a' && c <= 'z') ||
                       (c >= 'A' && c <= 'Z') ||
                       (c >= '0' && c <= '9') ||
                       c == '_' || c == '-' || c == '.');
      if (!ok) return false;
      out[n++] = c;
    }
    out[n] = 0;
    return n > 0;
  }

  static bool makeKeyPath(const char* key, char* out, size_t out_size) {
    char clean[48];
    if (!sanitizeLeafName(key, clean, sizeof(clean))) return false;
    return snprintf(out, out_size, "%s/%s.txt", kFlashKvDir, clean) > 0;
  }

  bool writeBuffer(const char* path, const uint8_t* data, size_t len) {
#if QH4_HAS_LITTLEFS
    if (!ready_ || path == nullptr || path[0] == 0) return false;
    gFlashFs.remove(path);
    File file = gFlashFs.open(path, FILE_WRITE);
    if (!file) return false;
    const size_t wrote = (len > 0u) ? file.write(data, len) : 0u;
    file.close();
    return wrote == len;
#else
    (void)path;
    (void)data;
    (void)len;
    return false;
#endif
  }

  static bool copyFile(File& src, File& dst) {
    uint8_t buf[512];
    while (src.available() > 0) {
      const int got = src.read(buf, sizeof(buf));
      if (got < 0) return false;
      if (got == 0) break;
      if (dst.write(buf, (size_t)got) != (size_t)got) return false;
    }
    return true;
  }

  void listDirectory(const char* path, Print& out, const char* label) const {
#if QH4_HAS_LITTLEFS
    File dir = gFlashFs.open(path);
    if (!dir) {
      out.print(label);
      out.println(F(": <missing>"));
      return;
    }

    bool any = false;
    while (true) {
      File entry = dir.openNextFile();
      if (!entry) break;
      any = true;
      out.print(label);
      out.print('/');
      out.print(entry.name());
      out.print(' ');
      out.println((unsigned long)entry.size());
      entry.close();
    }
    dir.close();

    if (!any) {
      out.print(label);
      out.println(F(": <empty>"));
    }
#else
    (void)path;
    (void)out;
    (void)label;
#endif
  }

  bool ready_ = false;
};

#if QH4_HAS_LITTLEFS
Qh4FlashFsType gFlashFs;
#endif

FlashManager gFlashStore;

const char* storageSourceName(StorageSource source) {
  switch (source) {
    case StorageSource::kSd:    return "sd";
    case StorageSource::kFlash: return "flash";
    default:                    return "none";
  }
}

bool openStoredFileRead(const char* path, File& file, StorageSource& source) {
  source = StorageSource::kNone;
  Serial.println("openStoredFileRead");
  Serial.println(path);
  if (path == nullptr || path[0] == 0) {
  Serial.println("openStoredFileRead fail1");
    return false;
  }

  if (gFlashStore.ready() && gFlashStore.openReadPath(path, file)) {
    source = StorageSource::kFlash;
    return true;
  }

  file = SD.open(path, FILE_READ);
  if (file) {
    Serial.println("StorageSource::kSd");
    source = StorageSource::kSd;
    return true;
  }

  Serial.println("openStoredFileRead FAIL");
  return false;
}

struct WavMeta {
  uint16_t channels = 0;
  uint16_t bits_per_sample = 0;
  uint16_t format = 0;
  uint32_t sample_rate = 0;
  uint32_t data_offset = 0;
  uint32_t data_size = 0;
};

static bool readExactFile(File& file, void* dst, size_t len) {
  return file.read((uint8_t*)dst, len) == (int)len;
}

static uint32_t readLe32Raw(const uint8_t* src) {
  return (uint32_t)src[0] |
         ((uint32_t)src[1] << 8) |
         ((uint32_t)src[2] << 16) |
         ((uint32_t)src[3] << 24);
}

static uint16_t readLe16Raw(const uint8_t* src) {
  return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static bool parseWavFile(File& file, WavMeta& meta) {
  if (!file.seek(0)) return false;

  uint8_t header[12];
  if (!readExactFile(file, header, sizeof(header))) return false;
  if (memcmp(header, "RIFF", 4) != 0) return false;
  if (memcmp(header + 8, "WAVE", 4) != 0) return false;

  bool found_fmt = false;
  bool found_data = false;

  while (file.available() > 0) {
    uint8_t chunk[8];
    if (!readExactFile(file, chunk, sizeof(chunk))) break;
    const uint32_t chunk_size = readLe32Raw(chunk + 4);
    const uint32_t next_pos = file.position() + chunk_size + (chunk_size & 1u);

    if (memcmp(chunk, "fmt ", 4) == 0) {
      uint8_t fmt[16];
      if (chunk_size < sizeof(fmt)) return false;
      if (!readExactFile(file, fmt, sizeof(fmt))) return false;
      meta.format = readLe16Raw(fmt + 0);
      meta.channels = readLe16Raw(fmt + 2);
      meta.sample_rate = readLe32Raw(fmt + 4);
      meta.bits_per_sample = readLe16Raw(fmt + 14);
      found_fmt = true;
    } else if (memcmp(chunk, "data", 4) == 0) {
      meta.data_offset = file.position();
      meta.data_size = chunk_size;
      found_data = true;
    }

    if (!file.seek(next_pos)) return false;
    if (found_fmt && found_data) break;
  }

  return found_fmt && found_data;
}

static bool loadMonoPcm16Wav(File& file, int16_t* storage, size_t capacity_samples, size_t& sample_count_out) {
  WavMeta meta;
  if (!parseWavFile(file, meta)) {
    Serial.println("loadMonoPcm16Wav failed1\n");
    return false;
  }

  if (meta.format != 1 || (meta.channels != 1 && meta.channels != 2) ||
      meta.bits_per_sample != 16 || meta.sample_rate != kAudioPcmRate) {
    Serial.println("loadMonoPcm16Wav failed2\n");
    return false;
  }

  const size_t frame_bytes = (size_t)meta.channels * 2u;
  const size_t samples_needed = meta.data_size / frame_bytes;
  if (samples_needed == 0 || samples_needed > capacity_samples) {
    Serial.println("loadMonoPcm16Wav failed3\n");
    return false;
  }
  if (!file.seek(meta.data_offset)) {
    Serial.println("loadMonoPcm16Wav failed4\n");
    return false;
  }

  sample_count_out = 0;
  uint8_t temp[512];
  uint32_t remaining = meta.data_size;
  while (remaining >= frame_bytes && sample_count_out < capacity_samples) {
    size_t want = remaining;
    if (want > sizeof(temp)) want = sizeof(temp);
    want -= want % frame_bytes;
    if (want == 0) want = frame_bytes;
    const int got = file.read(temp, want);
    if (got <= 0) break;
    const size_t usable = (size_t)got - ((size_t)got % frame_bytes);
    for (size_t i = 0; i + frame_bytes <= usable && sample_count_out < capacity_samples; i += frame_bytes) {
      const int16_t s0 = (int16_t)readLe16Raw(&temp[i]);
      int32_t mix = s0;
      if (meta.channels == 2) {
        const int16_t s1 = (int16_t)readLe16Raw(&temp[i + 2]);
        mix = ((int32_t)s0 + (int32_t)s1) / 2;
      }
      storage[sample_count_out++] = (int16_t)mix;
    }
    if (remaining >= usable) remaining -= usable;
    else remaining = 0;
  }
  Serial.print("sample_count_out: ");Serial.println(sample_count_out);
  int16_t mn = 32767, mx = -32768;
  for (size_t i = 0; i < sample_count_out; ++i) {
    if (storage[i] < mn) mn = storage[i];
    if (storage[i] > mx) mx = storage[i];
  }
  Serial.printf("WAV stats: count=%u min=%d max=%d\n",
                (unsigned)sample_count_out, (int)mn, (int)mx);
  return sample_count_out > 0;
}


static bool readExactFileAt(File& file, uint32_t pos, void* dst, size_t len) {
  return file.seek(pos) && readExactFile(file, dst, len);
}

static uint32_t crc32Compute(const void* data, size_t len) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  uint32_t crc = 0xFFFFFFFFu;
  while (len-- > 0) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
  }
  return ~crc;
}

enum : uint16_t {
  kPodCodecUlaw8Mono = 1u,
};

struct PodTrackInfo {
  uint16_t clip_count = 0;
  uint16_t reserved = 0;
  uint32_t first_clip_index = 0;
};

struct PodFooterV1 {
  char magic[4] = {'Q', 'P', 'O', 'D'};
  uint16_t version = 1u;
  uint16_t footer_size = 36u;
  uint16_t codec = kPodCodecUlaw8Mono;
  uint16_t sample_rate = kAudioPcmRate;
  uint16_t num_tracks = 0;
  uint16_t flags = 0;
  uint32_t total_clips = 0;
  uint32_t toc_offset = 0;
  uint32_t payload_bytes = 0;
  uint32_t toc_crc32 = 0;
  uint32_t payload_crc32 = 0;
};

static_assert(sizeof(PodTrackInfo) == 8, "PodTrackInfo size changed");
static_assert(sizeof(PodFooterV1) == 36, "PodFooterV1 size changed");

class WarblerPodCatalog {
 public:
  ~WarblerPodCatalog() { unload(); }

  bool isLoaded() const { return loaded_; }
  StorageSource loadedSource() const { return loaded_source_; }
  const char* loadedPath() const { return path_; }
  uint16_t numTracks() const { return footer_.num_tracks; }
  uint32_t totalClips() const { return footer_.total_clips; }
  uint32_t payloadBytes() const { return footer_.payload_bytes; }

  uint16_t clipCount(uint16_t track_index) const {
    if (!loaded_ || track_index >= footer_.num_tracks) return 0u;
    return tracks_[track_index].clip_count;
  }

  void unload() {
    if (toc_storage_ != nullptr) {
      free(toc_storage_);
      toc_storage_ = nullptr;
    }
    tracks_ = nullptr;
    clip_end_offsets_ = nullptr;
    memset(&footer_, 0, sizeof(footer_));
    loaded_ = false;
    loaded_source_ = StorageSource::kNone;
    path_[0] = 0;
  }

  bool loadPath(const char* path) {
    if (path == nullptr || path[0] == 0) return false;
    File file;
    StorageSource source = StorageSource::kNone;
    if (!openStoredFileRead(path, file, source)) return false;
    Serial.println("BULLSHIT");
    const bool ok = loadOpenFile(file, path, source);
    file.close();
    if (!ok) unload();
    return ok;
  }

  bool loadFirstExisting(const char* const* candidates, size_t count) {
    if (candidates == nullptr) return false;
    for (size_t i = 0; i < count; ++i) {
      const char* path = candidates[i];
      if (path == nullptr || path[0] == 0) continue;
      if (loadPath(path)) return true;
    }
    return false;
  }

  bool resolveClip(uint16_t track_index, uint16_t clip_index, uint32_t& start_out, uint32_t& length_out) const {
    start_out = 0u;
    length_out = 0u;
    if (!loaded_ || track_index >= footer_.num_tracks) return false;

    const PodTrackInfo& track = tracks_[track_index];
    if (clip_index >= track.clip_count) return false;

    const uint32_t global_index = track.first_clip_index + clip_index;
    if (global_index >= footer_.total_clips) return false;

    const uint32_t end = clip_end_offsets_[global_index];
    const uint32_t start = (global_index == 0u) ? 0u : clip_end_offsets_[global_index - 1u];
    if (end < start || end > footer_.payload_bytes) return false;

    start_out = start;
    length_out = end - start;
    return (length_out > 0u);
  }

 private:
  bool loadOpenFile(File& file, const char* path, StorageSource source) {
    unload();

    Serial.println("loadOpenFile");
    const uint32_t file_size = (uint32_t)file.size();
    if (file_size < sizeof(PodFooterV1)) return false;

    PodFooterV1 footer{};
    if (!readExactFileAt(file, file_size - sizeof(PodFooterV1), &footer, sizeof(footer))) return false;
    if (memcmp(footer.magic, "QPOD", 4) != 0) return false;
    if (footer.version != 1u || footer.footer_size != sizeof(PodFooterV1)) return false;
    if (footer.codec != kPodCodecUlaw8Mono || footer.sample_rate != kAudioPcmRate) return false;
    if (footer.num_tracks == 0u || footer.total_clips == 0u) return false;
    if (footer.toc_offset != footer.payload_bytes) return false;

    const size_t toc_size = (size_t)footer.num_tracks * sizeof(PodTrackInfo) +
                            (size_t)footer.total_clips * sizeof(uint32_t);
    if ((uint64_t)footer.toc_offset + (uint64_t)toc_size + sizeof(PodFooterV1) != (uint64_t)file_size) return false;

    toc_storage_ = static_cast<uint8_t*>(malloc(toc_size));
    if (toc_storage_ == nullptr) return false;
    if (!readExactFileAt(file, footer.toc_offset, toc_storage_, toc_size)) return false;
    if (footer.toc_crc32 != 0u && crc32Compute(toc_storage_, toc_size) != footer.toc_crc32) return false;

    tracks_ = reinterpret_cast<PodTrackInfo*>(toc_storage_);
    clip_end_offsets_ = reinterpret_cast<uint32_t*>(toc_storage_ + ((size_t)footer.num_tracks * sizeof(PodTrackInfo)));

    uint32_t expected_first = 0u;
    for (uint16_t i = 0; i < footer.num_tracks; ++i) {
      const PodTrackInfo& track = tracks_[i];
      if (track.first_clip_index != expected_first) return false;
      expected_first += track.clip_count;
      if (expected_first > footer.total_clips) return false;
    }
    if (expected_first != footer.total_clips) return false;

    uint32_t prev_end = 0u;
    for (uint32_t i = 0; i < footer.total_clips; ++i) {
      const uint32_t end = clip_end_offsets_[i];
      if (end < prev_end || end > footer.payload_bytes) return false;
      prev_end = end;
    }
    if (clip_end_offsets_[footer.total_clips - 1u] != footer.payload_bytes) return false;

    footer_ = footer;
    loaded_ = true;
    loaded_source_ = source;
    strncpy(path_, path, sizeof(path_) - 1);
    path_[sizeof(path_) - 1] = 0;
    Serial.println("loadOpenFile GOOD");
    return true;
  }

  uint8_t* toc_storage_ = nullptr;
  PodTrackInfo* tracks_ = nullptr;
  uint32_t* clip_end_offsets_ = nullptr;
  PodFooterV1 footer_{};
  bool loaded_ = false;
  StorageSource loaded_source_ = StorageSource::kNone;
  char path_[64] = {};
};

class WarblerPodPlayer {
 public:
  WarblerPodPlayer(AudioPlayQueue& player, WarblerPodCatalog& catalog)
      : player_(player), catalog_(catalog) {}

  bool isActive() const { return active_; }
  uint16_t activeIndex() const { return active_index_; }
  uint16_t activeTrack() const { return active_track_; }
  uint16_t activeClip() const { return active_clip_; }
  StorageSource activeSource() const { return active_source_; }

  void stop() {
    // player_.stop();
    closeActiveFile();
    active_ = false;
    active_index_ = 0u;
    active_track_ = 0u;
    active_clip_ = 0u;
    active_source_ = StorageSource::kNone;
  }

  bool play(uint16_t track_index, uint16_t clip_index, uint16_t active_index = 0u) {
    uint32_t clip_start = 0u;
    uint32_t clip_length = 0u;
    if (!catalog_.resolveClip(track_index, clip_index, clip_start, clip_length)) return false;

    stop();

    StorageSource source = StorageSource::kNone;
    if (!openStoredFileRead(catalog_.loadedPath(), file_, source)) return false;
    Serial.println("BULLSHIT2");
    if (!file_.seek(clip_start)) {
      file_.close();
      return false;
    }

    active_ = true;
    active_index_ = active_index;
    active_track_ = track_index;
    active_clip_ = clip_index;
    active_source_ = source;
    clip_remaining_ = clip_length;
    decode_len_ = 0u;
    decode_pos_ = 0u;
    return true;
  }

  void service() {
    if (!active_) return;

    while (true) {
      int16_t* block = player_.getBuffer();
      if (block == nullptr) return;

      bool finished = false;
      for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        if (decode_pos_ >= decode_len_ && !refillDecodeBuffer()) {
          block[i] = 0;
          finished = true;
        } else {
          block[i] = decodeUlaw(decode_buffer_[decode_pos_++]);
        }
      }
      player_.playBuffer();

      if (finished || (clip_remaining_ == 0u && decode_pos_ >= decode_len_)) {
        closeActiveFile();
        active_ = false;
        active_index_ = 0u;
        active_track_ = 0u;
        active_clip_ = 0u;
        active_source_ = StorageSource::kNone;
        return;
      }
    }
  }

 private:
  static int16_t decodeUlaw(uint8_t u_val) {
    u_val = ~u_val;
    int t = ((u_val & 0x0Fu) << 3) + 0x84;
    t <<= ((unsigned)u_val & 0x70u) >> 4;
    return (u_val & 0x80u) ? (int16_t)(0x84 - t) : (int16_t)(t - 0x84);
  }

  bool refillDecodeBuffer() {
    if (!active_ || clip_remaining_ == 0u) {
      decode_len_ = 0u;
      decode_pos_ = 0u;
      return false;
    }

    const size_t want = (clip_remaining_ < sizeof(decode_buffer_))
        ? (size_t)clip_remaining_
        : sizeof(decode_buffer_);
    const int got = file_.read(decode_buffer_, want);
    if (got <= 0) {
      clip_remaining_ = 0u;
      decode_len_ = 0u;
      decode_pos_ = 0u;
      return false;
    }

    decode_len_ = (size_t)got;
    decode_pos_ = 0u;
    clip_remaining_ -= decode_len_;
    return true;
  }

  void closeActiveFile() {
    if (file_) file_.close();
    clip_remaining_ = 0u;
    decode_len_ = 0u;
    decode_pos_ = 0u;
  }

  AudioPlayQueue& player_;
  WarblerPodCatalog& catalog_;
  File file_;
  bool active_ = false;
  uint16_t active_index_ = 0u;
  uint16_t active_track_ = 0u;
  uint16_t active_clip_ = 0u;
  StorageSource active_source_ = StorageSource::kNone;
  uint32_t clip_remaining_ = 0u;
  uint8_t decode_buffer_[512] = {};
  size_t decode_len_ = 0u;
  size_t decode_pos_ = 0u;
};

class AudioFrequencyTapStereo : public AudioStream {
 public:
  explicit AudioFrequencyTapStereo(AudioFrequency* bitmap = nullptr)
      : AudioStream(2, input_queue_array_), bitmap_(bitmap) {}

  void setBitmap(AudioFrequency* bitmap) { bitmap_ = bitmap; }
  AudioFrequency* bitmap() const { return bitmap_; }

  virtual void update() override {
    audio_block_t* left = receiveReadOnly(0);
    audio_block_t* right = receiveReadOnly(1);
    if (left == nullptr && right == nullptr) return;

    if (bitmap_ != nullptr) {
      int16_t mono[AUDIO_BLOCK_SAMPLES];
      for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        const int32_t l = (left != nullptr) ? left->data[i] : 0;
        const int32_t r = (right != nullptr) ? right->data[i] : l;
        mono[i] = (int16_t)((l + r) / 2);
      }
      bitmap_->processSamples(16, 1, mono, AUDIO_BLOCK_SAMPLES);
    }

    if (left != nullptr) {
      transmit(left, 0);
      release(left);
    }
    if (right != nullptr) {
      transmit(right, 1);
      release(right);
    }
  }

 private:
  audio_block_t* input_queue_array_[2] = {};
  AudioFrequency* bitmap_ = nullptr;
};

static constexpr uint8_t kQueueFillBudget = 8;

class PsramQueuedWavClip {
 public:
  PsramQueuedWavClip(AudioPlayQueue& player, int16_t* storage, size_t capacity_samples, const char* label)
      : player_(player), storage_(storage), capacity_samples_(capacity_samples), label_(label) {}

  bool isLoaded() const { return loaded_; }
  bool isActive() const { return active_; }
  uint16_t activeIndex() const { return active_index_; }
  const char* loadedPath() const { return path_; }
  StorageSource loadedSource() const { return loaded_source_; }

  void stop() {
    // Soft stop for now. Do NOT call player_.stop() until that shim is proven.
    active_ = false;
    cursor_ = 0;
    active_index_ = 0;
  }

  void unload() {
    stop();
    loaded_ = false;
    loaded_source_ = StorageSource::kNone;
    sample_count_ = 0;
    size_bytes_ = 0;
    path_[0] = 0;
  }

  bool playPath(const char* path, uint16_t active_index = 0) {
    if (path == nullptr || path[0] == 0) {
      Serial.println("playPath1");
      return false;
    }
    if (!loaded_ || strcmp(path_, path) != 0) {
      if (!loadFromStoredPath(path)) {
        Serial.println("loadFromStoredPath failed");
        return false;
      }
    }
    // Do NOT call player_.stop() here for now.
    cursor_ = 0;
    active_ = true;
    active_index_ = active_index;
    Serial.println("active_index_");
    Serial.println(active_index);
    return true;
  }

  bool playFirstExisting(const char* const* candidates, size_t count, uint16_t active_index = 0) {
    if (candidates == nullptr) return false;
    for (size_t i = 0; i < count; ++i) {
      const char* path = candidates[i];
      if (path == nullptr || path[0] == 0) continue;
      File f;
      StorageSource source = StorageSource::kNone;
      if (openStoredFileRead(path, f, source)) {
        f.close();
        return playPath(path, active_index);
      }
    }
    return false;
  }

  void service() {
    if (!active_) return;

    uint8_t queued = 0;
    while (queued < kQueueFillBudget) {
      int16_t* block = player_.getBuffer();
      if (block == nullptr) break;

      for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        block[i] = (cursor_ < sample_count_) ? storage_[cursor_++] : 0;
      }
      player_.playBuffer();
      ++queued;

      if (cursor_ >= sample_count_) {
        Serial.println("SOUND END");
        active_ = false;
        active_index_ = 0;
        break;
      }
    }
    Serial.print(".");
  }

 private:
  bool loadFromStoredPath(const char* path) {
    File file;
    StorageSource source = StorageSource::kNone;
    if (!openStoredFileRead(path, file, source)) return false;
    const bool ok = loadFromOpenFile(file, path, source);
    file.close();
    return ok;
  }

  bool loadFromOpenFile(File& file, const char* path, StorageSource source) {
    size_t sample_count = 0;
    if (!loadMonoPcm16Wav(file, storage_, capacity_samples_, sample_count)) return false;

    sample_count_ = sample_count;
    size_bytes_ = sample_count_ * sizeof(int16_t);
    strncpy(path_, path, sizeof(path_) - 1);
    path_[sizeof(path_) - 1] = 0;
    loaded_ = true;
    loaded_source_ = source;
    return true;
  }

  AudioPlayQueue& player_;
  int16_t* storage_ = nullptr;
  size_t capacity_samples_ = 0;
  const char* label_ = nullptr;

  bool loaded_ = false;
  bool active_ = false;
  StorageSource loaded_source_ = StorageSource::kNone;
  uint16_t active_index_ = 0;
  size_t sample_count_ = 0;
  size_t size_bytes_ = 0;
  size_t cursor_ = 0;
  char path_[64] = {};
};

class PsramLoopingWavAsset {
 public:
  PsramLoopingWavAsset(int16_t* storage, size_t capacity_samples, const char* label)
      : storage_(storage), capacity_samples_(capacity_samples), label_(label) {}

  bool isLoaded() const { return loaded_; }
  const char* loadedPath() const { return path_; }
  StorageSource loadedSource() const { return loaded_source_; }
  size_t sampleCount() const { return sample_count_; }
  const int16_t* data() const { return storage_; }

  void unload() {
    loaded_ = false;
    loaded_source_ = StorageSource::kNone;
    sample_count_ = 0;
    path_[0] = 0;
  }

  bool loadPath(const char* path) {
    if (path == nullptr || path[0] == 0) return false;
    File file;
    StorageSource source = StorageSource::kNone;
    if (!openStoredFileRead(path, file, source)) return false;
    const bool ok = loadOpenFile(file, path, source);
    file.close();
    return ok;
  }

  bool loadFirstExisting(const char* const* candidates, size_t count) {
    if (candidates == nullptr) return false;
    for (size_t i = 0; i < count; ++i) {
      const char* path = candidates[i];
      if (path == nullptr || path[0] == 0) continue;
      if (loadPath(path)) return true;
    }
    return false;
  }

 private:
  bool loadOpenFile(File& file, const char* path, StorageSource source) {
    size_t sample_count = 0;
    if (!loadMonoPcm16Wav(file, storage_, capacity_samples_, sample_count)) return false;
    sample_count_ = sample_count;
    strncpy(path_, path, sizeof(path_) - 1);
    path_[sizeof(path_) - 1] = 0;
    loaded_source_ = source;
    loaded_ = true;
    return true;
  }

  int16_t* storage_ = nullptr;
  size_t capacity_samples_ = 0;
  const char* label_ = nullptr;
  bool loaded_ = false;
  StorageSource loaded_source_ = StorageSource::kNone;
  size_t sample_count_ = 0;
  char path_[64] = {};
};

class LoopingPsramPlayer {
 public:
  LoopingPsramPlayer(AudioPlayQueue& player, PsramLoopingWavAsset& asset)
      : player_(player), asset_(asset) {}

  bool isActive() const { return active_; }

  void start() {
    if (asset_.sampleCount() == 0u) return;
    if (!active_) {
      cursor_ = 0;
      active_ = true;
    }
  }

  void stop() {
    // Soft stop: let already-queued audio drain.
    active_ = false;
    cursor_ = 0;
    // player_.stop();   // leave disabled until stop() is proven safe
  }

  void service() {
    if (!active_ || asset_.sampleCount() == 0u) return;

    const int16_t* data = asset_.data();
    const size_t count = asset_.sampleCount();

    uint8_t queued = 0;
    while (queued < kQueueFillBudget) {
      int16_t* block = player_.getBuffer();
      if (block == nullptr) break;

      for (size_t i = 0; i < AUDIO_BLOCK_SAMPLES; ++i) {
        block[i] = data[cursor_++];
        if (cursor_ >= count) cursor_ = 0;
      }
      player_.playBuffer();
      ++queued;
    }
  }
 private:
  AudioPlayQueue& player_;
  PsramLoopingWavAsset& asset_;
  bool active_ = false;
  size_t cursor_ = 0;
};

EyeDriver gEyes;
DynamixelTtlMasterBus gAntennaBusMaster(Serial7, qh4::kTXEN);
TorchDriver gTorch;
AntennaShow gAntenna;

AudioInputUSB gUsbIn;
AudioOutputUSB gUsbOut;
AudioPlayQueue gLampQueue;
AudioPlayQueue gAuxQueue;
AudioPlayQueue gAntennaLoopLeftQueue;
AudioPlayQueue gAntennaLoopRightQueue;
AudioSynthWaveformSine gWhineLeft;
AudioSynthWaveformSine gWhineRight;
AudioMixer4 gContentMixLeft;
AudioMixer4 gContentMixRight;
AudioFrequencyBitmap gAudioBitmap;
AudioFrequencyTapStereo gContentTap(nullptr);//&gAudioBitmap);
AudioMixer4 gFxMixLeft;
AudioMixer4 gFxMixRight;
AudioMixer4 gMasterMixLeft;
AudioMixer4 gMasterMixRight;
AudioOutputI2S gI2sOut;

AudioConnection gAudioUsbL(gUsbIn, 0, gContentMixLeft, 0);
AudioConnection gAudioUsbR(gUsbIn, 1, gContentMixRight, 0);
AudioConnection gAudioContentTapL(gContentMixLeft, 0, gContentTap, 0);
AudioConnection gAudioContentTapR(gContentMixRight, 0, gContentTap, 1);
AudioConnection gAudioContentToMasterL(gContentTap, 0, gMasterMixLeft, 0);
AudioConnection gAudioContentToMasterR(gContentTap, 1, gMasterMixRight, 0);
AudioConnection gAudioFxBusL(gFxMixLeft, 0, gMasterMixLeft, 1);
AudioConnection gAudioFxBusR(gFxMixRight, 0, gMasterMixRight, 1);
AudioConnection gAudioOutL(gMasterMixLeft, 0, gI2sOut, 0);
AudioConnection gAudioOutR(gMasterMixRight, 0, gI2sOut, 1);
AudioConnection gAudioUsbBackL(gMasterMixLeft, 0, gUsbOut, 0);
AudioConnection gAudioUsbBackR(gMasterMixRight, 0, gUsbOut, 1);
AudioConnection gLampToFxL(gLampQueue, 0, gFxMixLeft, 0);
AudioConnection gLampToFxR(gLampQueue, 0, gFxMixRight, 0);
AudioConnection gAuxToFxL(gAuxQueue, 0, gFxMixLeft, 1);
AudioConnection gAuxToFxR(gAuxQueue, 0, gFxMixRight, 1);
AudioConnection gAntennaLoopToFxL(gAntennaLoopLeftQueue, 0, gFxMixLeft, 2);
AudioConnection gAntennaLoopToFxR(gAntennaLoopRightQueue, 0, gFxMixRight, 2);
AudioConnection gWhineToFxL(gWhineLeft, 0, gFxMixLeft, 3);
AudioConnection gWhineToFxR(gWhineRight, 0, gFxMixRight, 3);

EXTMEM int16_t gLampCacheStorage[kLampCacheCapacitySamples];
EXTMEM int16_t gAuxCacheStorage[kAuxCacheCapacitySamples];
EXTMEM int16_t gAntennaLoopStorage[kAntennaLoopCapacitySamples];
PsramQueuedWavClip gLampClip(gLampQueue, gLampCacheStorage, kLampCacheCapacitySamples, "lamp");
PsramQueuedWavClip gAuxClip(gAuxQueue, gAuxCacheStorage, kAuxCacheCapacitySamples, "aux");
WarblerPodCatalog gWarblerPodCatalog;
WarblerPodPlayer gWarblerPodPlayer(gAuxQueue, gWarblerPodCatalog);
PsramLoopingWavAsset gAntennaLoopAsset(gAntennaLoopStorage, kAntennaLoopCapacitySamples, "antenna");
LoopingPsramPlayer gAntennaLoopLeftPlayer(gAntennaLoopLeftQueue, gAntennaLoopAsset);
LoopingPsramPlayer gAntennaLoopRightPlayer(gAntennaLoopRightQueue, gAntennaLoopAsset);

struct AntennaBusState {
  int32_t left_goal = kAntennaCenterLeft;
  int32_t right_goal = kAntennaCenterRight;
  int32_t last_sent_left = INT32_MIN;
  int32_t last_sent_right = INT32_MIN;
  uint32_t next_update_ms = 0;
  uint32_t last_send_ms = 0;
  bool configured = false;
} gAntennaBus;

struct AudioState {
  bool sd_ready = false;
  float master_volume = 0.85f;
  bool usb_enabled = true;
  float whine_left_amp = 0.0f;
  float whine_right_amp = 0.0f;
  int32_t last_whine_left_goal = kAntennaCenterLeft;
  int32_t last_whine_right_goal = kAntennaCenterRight;
  uint32_t last_whine_ms = 0;
  uint16_t active_aux_index = 0;
  uint32_t active_aux_arg0 = 0;
  StorageSource antenna_loop_source = StorageSource::kNone;
  uint32_t next_antenna_loop_attempt_ms = 0;
} gAudio;

struct ControlShadow {
  uint16_t flashlight_raw = 0;
  uint32_t flashlight_timeout_ms = kTorchDefaultTimeoutMs;
  uint16_t volume_raw = 850;
  uint32_t playsound_arg0 = 0;
  uint8_t antenna_mode = (uint8_t)AntennaShow::kIdle;
  int16_t antenna_left = 0;
  int16_t antenna_right = 0;
  int16_t antenna_both = 0;
  uint8_t eye_mode = 0;
  uint16_t eye_effort = 0;
  uint8_t eye_brightness = eye_driver::kDefaultBrightness;
  uint8_t flash_op = 0;
} gCtl;

uint8_t gControlTable[kDxlControlTableSize] = {};
uint8_t gPeripheralStatusReturnLevel = kStatusReturnLevelDefault;
uint8_t gFlashLastResult = kFlashResultOk;

uint32_t gNextEyeFrameMs = 0;
uint32_t gNextHeartbeatMs = 0;
bool gHeartbeatState = false;

char gSerialLine[kLineBufLen] = {};
size_t gSerialLineLen = 0;
#if defined(USB_DUAL_SERIAL_AUDIO)
char gSerialUsb1Line[kLineBufLen] = {};
size_t gSerialUsb1LineLen = 0;
#endif

static bool ctInRange(uint16_t addr, uint16_t len) {
  return (len > 0u) && (addr < kDxlControlTableSize) && ((uint32_t)addr + (uint32_t)len <= (uint32_t)kDxlControlTableSize);
}

static uint8_t ctGetU8(uint16_t addr) {
  return ctInRange(addr, 1) ? gControlTable[addr] : 0;
}

static uint16_t ctGetU16(uint16_t addr) {
  if (!ctInRange(addr, 2)) return 0;
  return dxl::readLe16(&gControlTable[addr]);
}

static int16_t ctGetI16(uint16_t addr) {
  return (int16_t)ctGetU16(addr);
}

static uint32_t ctGetU32(uint16_t addr) {
  if (!ctInRange(addr, 4)) return 0;
  return dxl::readLe32(&gControlTable[addr]);
}

static int32_t ctGetI32(uint16_t addr) {
  return (int32_t)ctGetU32(addr);
}

static void ctSetU8(uint16_t addr, uint8_t value) {
  if (!ctInRange(addr, 1)) return;
  gControlTable[addr] = value;
}

static void ctSetU16(uint16_t addr, uint16_t value) {
  if (!ctInRange(addr, 2)) return;
  dxl::writeLe16(&gControlTable[addr], value);
}

static void ctSetI16(uint16_t addr, int16_t value) {
  ctSetU16(addr, (uint16_t)value);
}

static void ctSetU32(uint16_t addr, uint32_t value) {
  if (!ctInRange(addr, 4)) return;
  dxl::writeLe32(&gControlTable[addr], value);
}

static void ctSetI32(uint16_t addr, int32_t value) {
  ctSetU32(addr, (uint32_t)value);
}

static bool rangesOverlap(uint16_t a_addr, uint16_t a_len, uint16_t b_addr, uint16_t b_len) {
  const uint32_t a0 = a_addr;
  const uint32_t a1 = a0 + a_len;
  const uint32_t b0 = b_addr;
  const uint32_t b1 = b0 + b_len;
  return (a0 < b1) && (b0 < a1);
}

static uint16_t normToRaw1000(float v01) {
  return (uint16_t)lroundf(clampf(v01, 0.0f, 1.0f) * 1000.0f);
}

static float raw1000ToNorm(uint16_t raw) {
  return clampf((float)raw / 1000.0f, 0.0f, 1.0f);
}

static float signedRaw1000ToNorm(int16_t raw) {
  return clampf((float)raw / 1000.0f, -1.0f, 1.0f);
}

static int16_t normToSignedRaw1000(float v) {
  return clampi16((int32_t)lroundf(clampf(v, -1.0f, 1.0f) * 1000.0f), -1000, 1000);
}

void applyAudioMixerGains() {
  const float master = clampf(gAudio.master_volume, 0.0f, 1.0f);
  const float content = gAudio.usb_enabled ? master : 0.0f;

  gContentMixLeft.gain(0, 1.0f);
  gContentMixRight.gain(0, 1.0f);
  gContentMixLeft.gain(1, 0.0f);
  gContentMixRight.gain(1, 0.0f);
  gContentMixLeft.gain(2, 0.0f);
  gContentMixRight.gain(2, 0.0f);
  gContentMixLeft.gain(3, 0.0f);
  gContentMixRight.gain(3, 0.0f);

  gMasterMixLeft.gain(0, content);
  gMasterMixRight.gain(0, content);
  gMasterMixLeft.gain(1, master);
  gMasterMixRight.gain(1, master);
  gMasterMixLeft.gain(2, 0.0f);
  gMasterMixRight.gain(2, 0.0f);
  gMasterMixLeft.gain(3, 0.0f);
  gMasterMixRight.gain(3, 0.0f);

  gFxMixLeft.gain(0, kAudioLampGain);
  gFxMixRight.gain(0, kAudioLampGain);
  gFxMixLeft.gain(1, kAudioAuxGain);
  gFxMixRight.gain(1, kAudioAuxGain);
  gFxMixLeft.gain(2, 0.0f);
  gFxMixRight.gain(2, 0.0f);
  gFxMixLeft.gain(3, 0.0f);
  gFxMixRight.gain(3, 0.0f);
}

void configureUsbAudio() {
  AudioMemory(160);
  gWhineLeft.frequency(kAudioWhineMinHz);
  gWhineRight.frequency(kAudioWhineMinHz);
  gWhineLeft.amplitude(0.0f);
  gWhineRight.amplitude(0.0f);
  applyAudioMixerGains();
}

bool initSdAudio() {
  if (SD.begin(BUILTIN_SDCARD)) {
    gAudio.sd_ready = true;
    return true;
  }
  gAudio.sd_ready = false;
  return false;
}

void invalidateCachedAudioAssets() {
  gLampClip.unload();
  gAuxClip.unload();
  gAntennaLoopLeftPlayer.stop();
  gAntennaLoopRightPlayer.stop();
  gAntennaLoopAsset.unload();
  gAudio.antenna_loop_source = StorageSource::kNone;
  gAudio.next_antenna_loop_attempt_ms = 0;
}

bool ensureAntennaLoopAssetLoaded(uint32_t now_ms) {
  if (gAntennaLoopAsset.isLoaded()) return true;
  if (now_ms < gAudio.next_antenna_loop_attempt_ms) return false;

  const char* candidates[] = {
    kFlashDefaultAntennaLoopAsset,
    // "/assets/whine_loop.wav",
    // "/sounds/antenna_loop.wav",
    "/antenna.wav",
    // "/sounds/whine_loop.wav",
    // "/whine_loop.wav",
  };

  const bool ok = gAntennaLoopAsset.loadFirstExisting(candidates, sizeof(candidates) / sizeof(candidates[0]));
  if (ok) {
    gAudio.antenna_loop_source = gAntennaLoopAsset.loadedSource();
    gAudio.next_antenna_loop_attempt_ms = 0;
    return true;
  }

  gAudio.next_antenna_loop_attempt_ms = now_ms + kAntennaLoopRetryMs;
  return false;
}

bool importDefaultAntennaLoopAssetToFlash() {
  if (!gFlashStore.ready() || !gAudio.sd_ready) return false;

  const char* candidates[] = {
    "/antenna.wav",
  };

  for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
    File file = SD.open(candidates[i], FILE_READ);
    if (!file) continue;
    file.close();
    const bool ok = gFlashStore.copySdFileToAsset(candidates[i], "antenna.wav");
    if (ok) {
      invalidateCachedAudioAssets();
    }
    return ok;
  }

  return false;
}


bool ensureWarblerPodCatalogLoaded() {
  if (gWarblerPodCatalog.isLoaded()) return true;

  const char* candidates[] = {
    kFlashWarblerPodAsset,
    "/warbler.pod",
    "/sounds/warbler.pod",
    "/assets/warbler.pod",
  };
  return gWarblerPodCatalog.loadFirstExisting(candidates, sizeof(candidates) / sizeof(candidates[0]));
}

bool importWarblerPodAssetFromSd(const char* sd_path, bool delete_source_after_import) {
  if (!gFlashStore.ready() || !gAudio.sd_ready || sd_path == nullptr || sd_path[0] == 0) return false;

  gWarblerPodPlayer.stop();
  gWarblerPodCatalog.unload();

  if (!gFlashStore.copySdFileToAsset(sd_path, "warbler.pod")) return false;

  WarblerPodCatalog verify_catalog;
  if (!verify_catalog.loadPath(kFlashWarblerPodAsset)) {
    gFlashStore.removePath(kFlashWarblerPodAsset);
    return false;
  }

  if (delete_source_after_import) {
    SD.remove(sd_path);
  }
  return true;
}

bool maybeInstallWarblerPodFromSd() {
  if (!gFlashStore.ready() || !gAudio.sd_ready) return false;

  const char* update_candidates[] = {
    "/warbler.pod.new",
    "/sounds/warbler.pod.new",
    "/assets/warbler.pod.new",
    "/warbler.pod",
    "/sounds/warbler.pod",
    "/assets/warbler.pod",
  };

  for (size_t i = 0; i < sizeof(update_candidates) / sizeof(update_candidates[0]); ++i) {
    File src = SD.open(update_candidates[i], FILE_READ);
    if (!src) continue;
    src.close();
    if (importWarblerPodAssetFromSd(update_candidates[i], true)) return true;
  }

  return false;
}

bool playLampSoundForTimeout(uint32_t timeout_ms) {
  if (!gAudio.sd_ready && !gFlashStore.ready()) return false;

  char cand_flash0[48] = {};
  char cand_sd0[48] = {};
  const uint32_t sec = (timeout_ms == 0u) ? 0u : ((timeout_ms + 500u) / 1000u);
  const char* candidates[6] = {
    "/lamp3.wav",
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    nullptr
  };

  if (sec > 0u) {
    snprintf(cand_flash0, sizeof(cand_flash0), "/assets/lamp%lu.wav", (unsigned long)sec);
    snprintf(cand_sd0, sizeof(cand_sd0), "/sounds/lamp%lu.wav", (unsigned long)sec);
    candidates[0] = cand_flash0;
    candidates[1] = cand_sd0;
    candidates[5] = "/lamp3.wav";
  }

  return gLampClip.playFirstExisting(candidates, 6, 0);
}

bool playIndexedSound(uint16_t index, uint32_t arg0) {
  if (index == 0u) return false;

  SerialUSB1.println("playIndexedSound");
  gAuxClip.stop();
  if (ensureWarblerPodCatalogLoaded()) {
    const uint16_t track_index = index - 1u;
    const uint16_t clip_index = (arg0 > 0xFFFFu) ? 0xFFFFu : (uint16_t)arg0;
    if (gWarblerPodPlayer.play(track_index, clip_index, index)) {
      gAudio.active_aux_index = index;
      gAudio.active_aux_arg0 = arg0;
      return true;
    }
  }
  gWarblerPodPlayer.stop();

  char cand_flash0[64] = {};
  char cand_sd0[64] = {};
  char cand_sd1[64] = {};
  char cand_flash2[64] = {};
  char cand_sd2[64] = {};
  char cand_sd3[64] = {};
  char cand_sd4[64] = {};

  if (arg0 != 0u) {
    snprintf(cand_flash0, sizeof(cand_flash0), "/assets/sound%u_%lu.wav", (unsigned)index, (unsigned long)arg0);
    snprintf(cand_sd0, sizeof(cand_sd0), "/sounds/sound%u_%lu.wav", (unsigned)index, (unsigned long)arg0);
    snprintf(cand_sd1, sizeof(cand_sd1), "/sound%u_%lu.wav", (unsigned)index, (unsigned long)arg0);
  }
  snprintf(cand_flash2, sizeof(cand_flash2), "/assets/sound%u.wav", (unsigned)index);
  snprintf(cand_sd2, sizeof(cand_sd2), "/sounds/sound%u.wav", (unsigned)index);
  snprintf(cand_sd3, sizeof(cand_sd3), "/sound%u.wav", (unsigned)index);
  snprintf(cand_sd4, sizeof(cand_sd4), "/lamp.wav");

  const char* candidates[7] = {
    cand_flash0[0] ? cand_flash0 : nullptr,
    cand_sd0[0] ? cand_sd0 : nullptr,
    cand_sd1[0] ? cand_sd1 : nullptr,
    cand_flash2,
    cand_sd2,
    cand_sd3,
    cand_sd4
  };

  const bool ok = gAuxClip.playFirstExisting(candidates, 7, index);
  if (ok) {
    gAudio.active_aux_index = index;
    gAudio.active_aux_arg0 = arg0;
  }
  return ok;
}

void serviceClipPlayers() {
  gLampClip.service();
  gWarblerPodPlayer.service();
  gAuxClip.service();
  if (!gAuxClip.isActive() && !gWarblerPodPlayer.isActive()) {
    gAudio.active_aux_index = 0;
    gAudio.active_aux_arg0 = 0;
  }
}

void serviceAntennaWhine(uint32_t now_ms) {
  if (gAudio.last_whine_ms == 0u) {
    gAudio.last_whine_ms = now_ms;
    gAudio.last_whine_left_goal = gAntennaBus.left_goal;
    gAudio.last_whine_right_goal = gAntennaBus.right_goal;
    return;
  }

  const uint32_t dt_ms = (now_ms > gAudio.last_whine_ms) ? (now_ms - gAudio.last_whine_ms) : 1u;
  const int32_t dl = abs(gAntennaBus.left_goal - gAudio.last_whine_left_goal);
  const int32_t dr = abs(gAntennaBus.right_goal - gAudio.last_whine_right_goal);

  const float speed_l = (float)dl / (float)dt_ms;
  const float speed_r = (float)dr / (float)dt_ms;
  const float target_l = clampf((speed_l - 0.2f) / 5.5f, 0.0f, 1.0f);
  const float target_r = clampf((speed_r - 0.2f) / 5.5f, 0.0f, 1.0f);

  gAudio.whine_left_amp += 0.22f * (target_l - gAudio.whine_left_amp);
  gAudio.whine_right_amp += 0.22f * (target_r - gAudio.whine_right_amp);

  const bool have_loop = ensureAntennaLoopAssetLoaded(now_ms);
  if (have_loop) {
    gAudio.antenna_loop_source = gAntennaLoopAsset.loadedSource();
    gWhineLeft.amplitude(0.0f);
    gWhineRight.amplitude(0.0f);
    gFxMixLeft.gain(3, 0.0f);
    gFxMixRight.gain(3, 0.0f);
    gFxMixLeft.gain(2, gAudio.whine_left_amp * kAudioAntennaLoopMaxGain);
    gFxMixRight.gain(2, gAudio.whine_right_amp * kAudioAntennaLoopMaxGain);

    if (gAudio.whine_left_amp > kAudioAntennaLoopStartThresh) gAntennaLoopLeftPlayer.start();
    else gAntennaLoopLeftPlayer.stop();
    if (gAudio.whine_right_amp > kAudioAntennaLoopStartThresh) gAntennaLoopRightPlayer.start();
    else gAntennaLoopRightPlayer.stop();

    gAntennaLoopLeftPlayer.service();
    gAntennaLoopRightPlayer.service();
  } else {
    gAudio.antenna_loop_source = StorageSource::kNone;
    gAntennaLoopLeftPlayer.stop();
    gAntennaLoopRightPlayer.stop();
    gFxMixLeft.gain(2, 0.0f);
    gFxMixRight.gain(2, 0.0f);
    gFxMixLeft.gain(3, 1.0f);
    gFxMixRight.gain(3, 1.0f);

    const float wobble_l = 25.0f * sinf(0.0081f * (float)now_ms);
    const float wobble_r = 25.0f * sinf(0.0077f * (float)now_ms + 0.7f);
    gWhineLeft.frequency(kAudioWhineMinHz + (kAudioWhineMaxHz - kAudioWhineMinHz) * target_l + wobble_l);
    gWhineRight.frequency(kAudioWhineMinHz + (kAudioWhineMaxHz - kAudioWhineMinHz) * target_r + wobble_r);
    gWhineLeft.amplitude(gAudio.whine_left_amp * kAudioWhineMaxGain);
    gWhineRight.amplitude(gAudio.whine_right_amp * kAudioWhineMaxGain);
  }

  gAudio.last_whine_left_goal = gAntennaBus.left_goal;
  gAudio.last_whine_right_goal = gAntennaBus.right_goal;
  gAudio.last_whine_ms = now_ms;
}

void mirrorWritableShadowToControlTable() {
  ctSetU16(ADDR_FLASHLIGHT, gCtl.flashlight_raw);
  ctSetU32(ADDR_FLASHLIGHT_TIMEOUT_MS, gCtl.flashlight_timeout_ms);
  ctSetU16(ADDR_SETVOLUME, gCtl.volume_raw);
  ctSetU16(ADDR_PLAYSOUND_INDEX, 0);
  ctSetU32(ADDR_PLAYSOUND_ARG0, gCtl.playsound_arg0);
  ctSetU8(ADDR_ANTENNA_MODE, gCtl.antenna_mode);
  ctSetU8(ADDR_ANTENNA_CLIP, 0);
  ctSetI16(ADDR_ANTENNA_LEFT, gCtl.antenna_left);
  ctSetI16(ADDR_ANTENNA_RIGHT, gCtl.antenna_right);
  ctSetI16(ADDR_ANTENNA_BOTH, gCtl.antenna_both);
  ctSetU8(ADDR_EYE_MODE, gCtl.eye_mode);
  ctSetU16(ADDR_EYE_EFFORT, gCtl.eye_effort);
  ctSetU8(ADDR_EYE_BRIGHTNESS, gCtl.eye_brightness);
  ctSetU8(ADDR_FLASH_OP, 0);
}

void updateReadOnlyControlTable(uint32_t now_ms) {
  uint32_t status = 0;
  if (gTorch.ready()) status |= kStatusTorchReady;
  if (gTorch.isOn()) status |= kStatusTorchOn;
  if (gAudio.sd_ready) status |= kStatusSdReady;
  if (gLampClip.isActive()) status |= kStatusLampPlaying;
  if (gAuxClip.isActive() || gWarblerPodPlayer.isActive()) status |= kStatusAuxPlaying;
  if (gAntennaBus.configured) status |= kStatusAntennaReady;
  if (gFlashStore.ready()) status |= kStatusFlashReady;
  if (gAntennaLoopAsset.isLoaded()) status |= kStatusAntennaLoop;
#if defined(USB_DUAL_SERIAL_AUDIO)
  status |= kStatusUsbDualSerial;
#endif
  ctSetU32(ADDR_STATUS_FLAGS, status);
  ctSetU32(ADDR_TORCH_REMAINING_MS, gTorch.timeoutRemainingMs(now_ms));
  ctSetI32(ADDR_PRESENT_ANTENNA_LEFT, gAntennaBus.left_goal);
  ctSetI32(ADDR_PRESENT_ANTENNA_RIGHT, gAntennaBus.right_goal);
  ctSetU16(ADDR_ACTIVE_SOUND_INDEX, gAudio.active_aux_index);

  uint8_t audio_flags = 0;
  if (gLampClip.isLoaded()) audio_flags |= kAudioFlagLampCached;
  if (gAuxClip.isLoaded()) audio_flags |= kAudioFlagAuxCached;
  if (gAntennaLoopAsset.isLoaded()) audio_flags |= kAudioFlagAntennaLoopCached;
  if (gAntennaLoopAsset.isLoaded() && gAntennaLoopAsset.loadedSource() == StorageSource::kFlash) {
    audio_flags |= kAudioFlagAntennaLoopFlash;
  }
  if (gWarblerPodCatalog.isLoaded()) audio_flags |= kAudioFlagWarblerPodLoaded;
  if (gWarblerPodCatalog.isLoaded() && gWarblerPodCatalog.loadedSource() == StorageSource::kFlash) {
    audio_flags |= kAudioFlagWarblerPodFlash;
  }
  ctSetU8(ADDR_AUDIO_FLAGS, audio_flags);

  uint8_t flash_flags = 0;
  if (gFlashStore.ready()) flash_flags |= kFlashFlagFsReady;
  if (gFlashStore.configExists()) flash_flags |= kFlashFlagConfigSaved;
  if (gAntennaLoopAsset.isLoaded()) flash_flags |= kFlashFlagAntennaLoopCached;
  if (gAntennaLoopAsset.isLoaded() && gAntennaLoopAsset.loadedSource() == StorageSource::kFlash) {
    flash_flags |= kFlashFlagAntennaLoopFlash;
  }
  if (gFlashStore.existsPath(kFlashDefaultAntennaLoopAsset)) {
    flash_flags |= kFlashFlagAntennaLoopAsset;
  }
  if (gFlashStore.existsPath(kFlashWarblerPodAsset)) {
    flash_flags |= kFlashFlagWarblerPodAsset;
  }
  ctSetU8(ADDR_FLASH_FLAGS, flash_flags);
  ctSetU32(ADDR_FLASH_USED_KB, gFlashStore.usedKilobytes());
  ctSetU32(ADDR_FLASH_TOTAL_KB, gFlashStore.totalKilobytes());
  ctSetU8(ADDR_FLASH_RESULT, gFlashLastResult);

  // Keep writable mirrors coherent with runtime in case of timeouts or OSC-side changes.
  if (gTorch.ready() && !gTorch.isOn()) {
    gCtl.flashlight_raw = 0u;
  }
  ctSetU16(ADDR_FLASHLIGHT, gCtl.flashlight_raw);
  ctSetU16(ADDR_SETVOLUME, gCtl.volume_raw);
  ctSetU8(ADDR_ANTENNA_MODE, gCtl.antenna_mode);
  ctSetU8(ADDR_EYE_MODE, gCtl.eye_mode);
  ctSetU16(ADDR_EYE_EFFORT, gCtl.eye_effort);
  ctSetU8(ADDR_EYE_BRIGHTNESS, gCtl.eye_brightness);
  ctSetU8(ADDR_FLASH_OP, 0);
}

void initializeControlTable() {
  memset(gControlTable, 0, sizeof(gControlTable));
  ctSetU16(ADDR_MODEL_NUMBER, kModelNumber);
  ctSetU8(ADDR_FIRMWARE_VERSION, kFirmwareVersion);
  ctSetU8(ADDR_ID, kPeripheralDxlId);
  ctSetU8(ADDR_BAUDRATE, 7);  // Dynamixel 4M preset
  ctSetU8(ADDR_STATUS_RETURN_LEVEL, gPeripheralStatusReturnLevel);
  mirrorWritableShadowToControlTable();
  updateReadOnlyControlTable(0);
}

void setEyesModeCommand(uint8_t code) {
  eye_driver::LedMatrixEye::Mode mode = eye_driver::LedMatrixEye::MODE_NORMAL;
  if (code == 1u) mode = eye_driver::LedMatrixEye::MODE_ON;
  else if (code == 2u) mode = eye_driver::LedMatrixEye::MODE_OFF;
  gEyes.left().setMode(mode);
  gEyes.right().setMode(mode);
  gCtl.eye_mode = code;
  ctSetU8(ADDR_EYE_MODE, gCtl.eye_mode);
}

void setEyesEffortCommand(uint16_t raw1000) {
  raw1000 = (raw1000 > 1000u) ? 1000u : raw1000;
  const float v = raw1000ToNorm(raw1000);
  gEyes.left().setNarrow(v);
  gEyes.right().setNarrow(v);
  gCtl.eye_effort = raw1000;
  ctSetU16(ADDR_EYE_EFFORT, gCtl.eye_effort);
}

void setEyesBrightnessCommand(uint8_t brightness) {
  gEyes.left().setBrightness(brightness);
  gEyes.right().setBrightness(brightness);
  gCtl.eye_brightness = brightness;
  ctSetU8(ADDR_EYE_BRIGHTNESS, gCtl.eye_brightness);
}

void setMasterVolumeCommandRaw(uint16_t raw1000) {
  raw1000 = (raw1000 > 1000u) ? 1000u : raw1000;
  gAudio.master_volume = raw1000ToNorm(raw1000);
  gCtl.volume_raw = raw1000;
  ctSetU16(ADDR_SETVOLUME, gCtl.volume_raw);
  applyAudioMixerGains();
}

void setTorchCommandRaw(uint16_t level_raw, uint32_t timeout_ms, bool trigger_sound) {
  if (level_raw > 1000u) level_raw = 1000u;
  gCtl.flashlight_raw = level_raw;
  gCtl.flashlight_timeout_ms = timeout_ms;
  ctSetU16(ADDR_FLASHLIGHT, gCtl.flashlight_raw);
  ctSetU32(ADDR_FLASHLIGHT_TIMEOUT_MS, gCtl.flashlight_timeout_ms);

  if (level_raw == 0u) {
    gTorch.setOff();
    return;
  }

  const float level = raw1000ToNorm(level_raw);
  gTorch.turnOn(level, timeout_ms);
  if (trigger_sound) {
    playLampSoundForTimeout(timeout_ms);
  }
}

void setAntennaModeCommand(uint8_t mode_code) {
  if (mode_code > (uint8_t)AntennaShow::kRetract) mode_code = (uint8_t)AntennaShow::kIdle;
  gAntenna.clearManual();
  gAntenna.setBaseMode((AntennaShow::BaseMode)mode_code, millis());
  gCtl.antenna_mode = mode_code;
  ctSetU8(ADDR_ANTENNA_MODE, gCtl.antenna_mode);
}

void triggerAntennaClipCommand(uint8_t clip_code) {
  if (clip_code > (uint8_t)AntennaShow::kScan) clip_code = 0;
  if (clip_code != 0u) {
    gAntenna.clearManual();
    gAntenna.triggerClip((AntennaShow::Clip)clip_code, millis());
  }
  ctSetU8(ADDR_ANTENNA_CLIP, 0);
}

void setAntennaManualLeftCommand(int16_t raw1000) {
  raw1000 = clampi16(raw1000, -1000, 1000);
  gAntenna.setManualLeft(signedRaw1000ToNorm(raw1000));
  gCtl.antenna_left = raw1000;
  ctSetI16(ADDR_ANTENNA_LEFT, gCtl.antenna_left);
}

void setAntennaManualRightCommand(int16_t raw1000) {
  raw1000 = clampi16(raw1000, -1000, 1000);
  gAntenna.setManualRight(signedRaw1000ToNorm(raw1000));
  gCtl.antenna_right = raw1000;
  ctSetI16(ADDR_ANTENNA_RIGHT, gCtl.antenna_right);
}

void setAntennaManualBothCommand(int16_t raw1000) {
  raw1000 = clampi16(raw1000, -1000, 1000);
  gAntenna.setManualBoth(signedRaw1000ToNorm(raw1000));
  gCtl.antenna_both = raw1000;
  gCtl.antenna_left = raw1000;
  gCtl.antenna_right = raw1000;
  ctSetI16(ADDR_ANTENNA_BOTH, gCtl.antenna_both);
  ctSetI16(ADDR_ANTENNA_LEFT, gCtl.antenna_left);
  ctSetI16(ADDR_ANTENNA_RIGHT, gCtl.antenna_right);
}

void triggerPlaySoundCommand(uint16_t index, uint32_t arg0) {
  gCtl.playsound_arg0 = arg0;
  ctSetU32(ADDR_PLAYSOUND_ARG0, gCtl.playsound_arg0);
  if (index != 0u) {
    playIndexedSound(index, arg0);
  }
  ctSetU16(ADDR_PLAYSOUND_INDEX, 0);
}

RuntimePersistedConfig makeRuntimePersistedConfig() {
  RuntimePersistedConfig cfg;
  cfg.magic = kPersistMagic;
  cfg.version = kPersistVersion;
  cfg.flashlight_timeout_ms = gCtl.flashlight_timeout_ms;
  cfg.volume_raw = gCtl.volume_raw;
  cfg.antenna_mode = gCtl.antenna_mode;
  cfg.eye_mode = gCtl.eye_mode;
  cfg.eye_effort = gCtl.eye_effort;
  cfg.eye_brightness = gCtl.eye_brightness;
  return cfg;
}

bool saveRuntimeConfigToFlash() {
  if (!gFlashStore.ready()) return false;
  return gFlashStore.saveRuntimeConfig(makeRuntimePersistedConfig());
}

bool loadRuntimeConfigFromFlash() {
  RuntimePersistedConfig cfg;
  if (!gFlashStore.loadRuntimeConfig(cfg)) return false;

  gCtl.flashlight_timeout_ms = cfg.flashlight_timeout_ms;
  setMasterVolumeCommandRaw(cfg.volume_raw);
  setAntennaModeCommand(cfg.antenna_mode);
  setEyesModeCommand(cfg.eye_mode);
  setEyesEffortCommand(cfg.eye_effort);
  setEyesBrightnessCommand(cfg.eye_brightness);
  setTorchCommandRaw(0u, gCtl.flashlight_timeout_ms, false);
  return true;
}

bool eraseRuntimeConfigInFlash() {
  return gFlashStore.eraseRuntimeConfig();
}

uint8_t executeFlashOp(uint8_t op) {
  switch (op) {
    case kFlashOpNone:
      return kFlashResultOk;
    case kFlashOpSaveConfig:
      return gFlashStore.ready() ? (saveRuntimeConfigToFlash() ? kFlashResultOk : kFlashResultIoError) : kFlashResultNoFlash;
    case kFlashOpLoadConfig:
      return gFlashStore.ready() ? (loadRuntimeConfigFromFlash() ? kFlashResultOk : kFlashResultBadData) : kFlashResultNoFlash;
    case kFlashOpEraseConfig:
      return gFlashStore.ready() ? (eraseRuntimeConfigInFlash() ? kFlashResultOk : kFlashResultIoError) : kFlashResultNoFlash;
    case kFlashOpImportAntennaLoop:
      return gFlashStore.ready() ? (importDefaultAntennaLoopAssetToFlash() ? kFlashResultOk : kFlashResultNotFound) : kFlashResultNoFlash;
    case kFlashOpClearAntennaLoopCache:
      gAntennaLoopLeftPlayer.stop();
      gAntennaLoopRightPlayer.stop();
      gAntennaLoopAsset.unload();
      gAudio.antenna_loop_source = StorageSource::kNone;
      gAudio.next_antenna_loop_attempt_ms = 0;
      return kFlashResultOk;
    default:
      return kFlashResultBadOp;
  }
}

bool isWritableControlByte(uint16_t addr) {
  if (addr >= ADDR_FLASHLIGHT && addr <= ADDR_EYE_BRIGHTNESS) return true;
  if (addr == ADDR_FLASH_OP) return true;
  return false;
}

uint8_t applyControlTableWrite(uint16_t addr, uint16_t len) {
  if (!ctInRange(addr, len)) return 0x04;  // data range
  for (uint16_t i = 0; i < len; ++i) {
    if (!isWritableControlByte(addr + i)) return 0x07;  // access error
  }

  if (rangesOverlap(addr, len, ADDR_FLASHLIGHT, 2) || rangesOverlap(addr, len, ADDR_FLASHLIGHT_TIMEOUT_MS, 4)) {
    setTorchCommandRaw(ctGetU16(ADDR_FLASHLIGHT), ctGetU32(ADDR_FLASHLIGHT_TIMEOUT_MS), true);
  }
  if (rangesOverlap(addr, len, ADDR_SETVOLUME, 2)) {
    setMasterVolumeCommandRaw(ctGetU16(ADDR_SETVOLUME));
  }
  if (rangesOverlap(addr, len, ADDR_PLAYSOUND_ARG0, 4) && !rangesOverlap(addr, len, ADDR_PLAYSOUND_INDEX, 2)) {
    gCtl.playsound_arg0 = ctGetU32(ADDR_PLAYSOUND_ARG0);
  }
  if (rangesOverlap(addr, len, ADDR_PLAYSOUND_INDEX, 2)) {
    triggerPlaySoundCommand(ctGetU16(ADDR_PLAYSOUND_INDEX), ctGetU32(ADDR_PLAYSOUND_ARG0));
  }
  if (rangesOverlap(addr, len, ADDR_ANTENNA_MODE, 1)) {
    setAntennaModeCommand(ctGetU8(ADDR_ANTENNA_MODE));
  }
  if (rangesOverlap(addr, len, ADDR_ANTENNA_LEFT, 2)) {
    setAntennaManualLeftCommand(ctGetI16(ADDR_ANTENNA_LEFT));
  }
  if (rangesOverlap(addr, len, ADDR_ANTENNA_RIGHT, 2)) {
    setAntennaManualRightCommand(ctGetI16(ADDR_ANTENNA_RIGHT));
  }
  if (rangesOverlap(addr, len, ADDR_ANTENNA_BOTH, 2)) {
    setAntennaManualBothCommand(ctGetI16(ADDR_ANTENNA_BOTH));
  }
  if (rangesOverlap(addr, len, ADDR_ANTENNA_CLIP, 1)) {
    triggerAntennaClipCommand(ctGetU8(ADDR_ANTENNA_CLIP));
  }
  if (rangesOverlap(addr, len, ADDR_EYE_MODE, 1)) {
    setEyesModeCommand(ctGetU8(ADDR_EYE_MODE));
  }
  if (rangesOverlap(addr, len, ADDR_EYE_EFFORT, 2)) {
    setEyesEffortCommand(ctGetU16(ADDR_EYE_EFFORT));
  }
  if (rangesOverlap(addr, len, ADDR_EYE_BRIGHTNESS, 1)) {
    setEyesBrightnessCommand(ctGetU8(ADDR_EYE_BRIGHTNESS));
  }
  if (rangesOverlap(addr, len, ADDR_FLASH_OP, 1)) {
    gCtl.flash_op = ctGetU8(ADDR_FLASH_OP);
    gFlashLastResult = executeFlashOp(gCtl.flash_op);
    ctSetU8(ADDR_FLASH_RESULT, gFlashLastResult);
    gCtl.flash_op = 0u;
    ctSetU8(ADDR_FLASH_OP, 0u);
  }
  return 0x00;
}

void initAntennaBus() {
  gAntennaBusMaster.begin(kAntennaBaud);
  delay(20);

  gAntennaBus.left_goal = kAntennaCenterLeft;
  gAntennaBus.right_goal = kAntennaCenterRight;

  const uint8_t torque_on = 1;
  const uint8_t return_level = 1;
  uint8_t left_goal[4];
  uint8_t right_goal[4];
  uint8_t accel[4];
  uint8_t vel[4];
  dxl::writeLe32(left_goal, (uint32_t)gAntennaBus.left_goal);
  dxl::writeLe32(right_goal, (uint32_t)gAntennaBus.right_goal);
  dxl::writeLe32(accel, kProfileAcceleration);
  dxl::writeLe32(vel, kProfileVelocity);

  const DxlBulkWriteEntry init_entries[] = {
    {kAntennaLeftId,  kDxlAddrStatusReturnLevel, 1, &return_level},
    {kAntennaRightId, kDxlAddrStatusReturnLevel, 1, &return_level},
    {kAntennaLeftId,  kDxlAddrProfileAcceleration, 4, accel},
    {kAntennaRightId, kDxlAddrProfileAcceleration, 4, accel},
    {kAntennaLeftId,  kDxlAddrProfileVelocity, 4, vel},
    {kAntennaRightId, kDxlAddrProfileVelocity, 4, vel},
    {kAntennaLeftId,  kDxlAddrGoalPosition, 4, left_goal},
    {kAntennaRightId, kDxlAddrGoalPosition, 4, right_goal},
    {kAntennaLeftId,  kDxlAddrTorqueEnable, 1, &torque_on},
    {kAntennaRightId, kDxlAddrTorqueEnable, 1, &torque_on},
  };
  gAntennaBusMaster.bulkWrite(init_entries, sizeof(init_entries) / sizeof(init_entries[0]));

  gAntennaBus.last_sent_left = gAntennaBus.left_goal;
  gAntennaBus.last_sent_right = gAntennaBus.right_goal;
  gAntennaBus.last_send_ms = millis();
  gAntennaBus.next_update_ms = millis();
  gAntennaBus.configured = true;
}

void serviceEyes(uint32_t now_ms) {
  if (now_ms < gNextEyeFrameMs) return;
  gNextEyeFrameMs = now_ms + kEyeRefreshMs;
  gEyes.tick(now_ms);
  gEyes.render();
  FastLED.show();
}

void serviceHeartbeat(uint32_t now_ms) {
  if (now_ms < gNextHeartbeatMs) return;
  gNextHeartbeatMs = now_ms + kHeartbeatMs;
  gHeartbeatState = !gHeartbeatState;
  digitalWrite(LED_BUILTIN, gHeartbeatState ? HIGH : LOW);
  digitalWrite(LOGO_LEFT_EYE, gHeartbeatState ? HIGH : LOW);
  digitalWrite(LOGO_RIGHT_EYE, gHeartbeatState ? HIGH : LOW);
}

void serviceAntenna(uint32_t now_ms) {
  gAntenna.tick(now_ms, 0.0f);
  gAntennaBus.left_goal = gAntenna.leftGoal();
  gAntennaBus.right_goal = gAntenna.rightGoal();

  if (!gAntennaBus.configured) return;
  if (now_ms < gAntennaBus.next_update_ms) return;
  gAntennaBus.next_update_ms = now_ms + kAntennaUpdateMs;

  const bool changed = (gAntennaBus.left_goal != gAntennaBus.last_sent_left) ||
                       (gAntennaBus.right_goal != gAntennaBus.last_sent_right);
  const bool keepalive = (now_ms - gAntennaBus.last_send_ms) >= kAntennaKeepAliveMs;
  if (!changed && !keepalive) return;

  uint8_t left_goal[4];
  uint8_t right_goal[4];
  dxl::writeLe32(left_goal, (uint32_t)gAntennaBus.left_goal);
  dxl::writeLe32(right_goal, (uint32_t)gAntennaBus.right_goal);
  const DxlBulkWriteEntry goal_entries[] = {
    {kAntennaLeftId,  kDxlAddrGoalPosition, 4, left_goal},
    {kAntennaRightId, kDxlAddrGoalPosition, 4, right_goal},
  };
  gAntennaBusMaster.bulkWrite(goal_entries, sizeof(goal_entries) / sizeof(goal_entries[0]));
  gAntennaBus.last_sent_left = gAntennaBus.left_goal;
  gAntennaBus.last_sent_right = gAntennaBus.right_goal;
  gAntennaBus.last_send_ms = now_ms;
}

void printAudioStatus(Print& out) {
  out.print(F("audio.volume="));
  out.print(gAudio.master_volume, 3);
  out.print(F(" sd="));
  out.print(gAudio.sd_ready ? F("ready") : F("missing"));
  out.print(F(" flash="));
  out.print(gFlashStore.ready() ? F("ready") : F("missing"));
  out.print(F(" lamp="));
  out.print(gLampClip.isActive() ? F("playing") : F("idle"));
  out.print(F(" aux="));
  out.print((gAuxClip.isActive() || gWarblerPodPlayer.isActive()) ? F("playing") : F("idle"));
  out.print(F(" aux.kind="));
  out.print(gWarblerPodPlayer.isActive() ? F("pod") : (gAuxClip.isActive() ? F("wav") : F("none")));
  out.print(F(" aux.index="));
  out.print(gAudio.active_aux_index);
  out.print(F(" pod="));
  out.print(gWarblerPodCatalog.isLoaded() ? F("ready") : F("missing"));
  out.print(F(" pod.tracks="));
  out.print((unsigned)gWarblerPodCatalog.numTracks());
  out.print(F(" pod.source="));
  out.print(storageSourceName(gWarblerPodCatalog.isLoaded() ? gWarblerPodCatalog.loadedSource() : StorageSource::kNone));
  out.print(F(" pod.path="));
  out.print(gWarblerPodCatalog.isLoaded() ? gWarblerPodCatalog.loadedPath() : "-");
  out.print(F(" loop="));
  out.print((gAntennaLoopLeftPlayer.isActive() || gAntennaLoopRightPlayer.isActive()) ? F("playing") : F("idle"));
  out.print(F(" loop.cached="));
  out.print(gAntennaLoopAsset.isLoaded() ? F("yes") : F("no"));
  out.print(F(" loop.source="));
  out.print(storageSourceName(gAntennaLoopAsset.isLoaded() ? gAntennaLoopAsset.loadedSource() : StorageSource::kNone));
  out.print(F(" loop.path="));
  out.println(gAntennaLoopAsset.isLoaded() ? gAntennaLoopAsset.loadedPath() : "-");
}

void printFlashStatus(Print& out) {
  out.print(F("flash.ready="));
  out.print(gFlashStore.ready() ? F("yes") : F("no"));
  out.print(F(" flash.used_kb="));
  out.print((unsigned long)gFlashStore.usedKilobytes());
  out.print(F(" flash.total_kb="));
  out.print((unsigned long)gFlashStore.totalKilobytes());
  out.print(F(" cfg.saved="));
  out.print(gFlashStore.configExists() ? F("yes") : F("no"));
  out.print(F(" antenna.asset="));
  out.print(gFlashStore.existsPath(kFlashDefaultAntennaLoopAsset) ? F("yes") : F("no"));
  out.print(F(" antenna.cached="));
  out.print(gAntennaLoopAsset.isLoaded() ? F("yes") : F("no"));
  out.print(F(" antenna.source="));
  out.print(storageSourceName(gAntennaLoopAsset.isLoaded() ? gAntennaLoopAsset.loadedSource() : StorageSource::kNone));
  out.print(F(" warbler.asset="));
  out.print(gFlashStore.existsPath(kFlashWarblerPodAsset) ? F("yes") : F("no"));
  out.print(F(" warbler.loaded="));
  out.print(gWarblerPodCatalog.isLoaded() ? F("yes") : F("no"));
  out.print(F(" last="));
  out.println((unsigned)gFlashLastResult);
}

void printStatus(Print& out) {
  const uint32_t now_ms = millis();
  out.print(F("eyes.mode="));
  out.print(gCtl.eye_mode);
  out.print(F(" eyes.sync="));
  out.print(gEyes.synchronized() ? F("on") : F("off"));
  out.print(F(" ears.mode="));
  out.print(gAntenna.baseModeName());
  out.print(F(" ears.clip="));
  out.print(gAntenna.clipName());
  out.print(F(" ears.goalL="));
  out.print(gAntennaBus.left_goal);
  out.print(F(" ears.goalR="));
  out.print(gAntennaBus.right_goal);
  out.print(F(" torch="));
  out.print(gTorch.isOn() ? F("on") : F("off"));
  out.print(F(" torch.level="));
  out.print(gTorch.level(), 3);
  out.print(F(" torch.timeout_ms="));
  out.print(gTorch.timeoutRemainingMs(now_ms));
  out.print(F(" volume="));
  out.print(gAudio.master_volume, 3);
  out.print(F(" sd="));
  out.print(gAudio.sd_ready ? F("ready") : F("missing"));
  out.print(F(" flash="));
  out.print(gFlashStore.ready() ? F("ready") : F("missing"));
  out.print(F(" loop="));
  out.print((gAntennaLoopLeftPlayer.isActive() || gAntennaLoopRightPlayer.isActive()) ? F("playing") : F("idle"));
  out.print(F(" loop.src="));
  out.print(storageSourceName(gAntennaLoopAsset.isLoaded() ? gAntennaLoopAsset.loadedSource() : StorageSource::kNone));
  out.print(F(" loop.cache="));
  out.println(gAntennaLoopAsset.isLoaded() ? F("yes") : F("no"));
}

class PeripheralDxlNode {
 public:
  explicit PeripheralDxlNode(HardwareSerial& port) : port_(port) {}

  void begin(uint32_t baud) {
    port_.begin(baud);
    rx_len_ = 0;
  }

  void service() {
    while (port_.available() > 0) {
      const int raw = port_.read();
      if (raw < 0) break;
      if (rx_len_ < sizeof(rx_)) {
        rx_[rx_len_++] = (uint8_t)raw;
      } else {
        rx_len_ = 0;
      }
      processBuffer();
    }
  }

 private:
  static bool isHeaderAt(const uint8_t* data) {
    return data[0] == 0xFF && data[1] == 0xFF && data[2] == 0xFD && data[3] == 0x00;
  }

  void dropBytes(size_t n) {
    if (n >= rx_len_) {
      rx_len_ = 0;
      return;
    }
    memmove(rx_, rx_ + n, rx_len_ - n);
    rx_len_ -= n;
  }

  void processBuffer() {
    while (rx_len_ >= 7) {
      size_t start = 0;
      while (start + 4 <= rx_len_ && !isHeaderAt(&rx_[start])) ++start;
      if (start > 0) {
        dropBytes(start);
        if (rx_len_ < 7) return;
      }
      if (!isHeaderAt(rx_)) {
        dropBytes(1);
        continue;
      }

      const uint16_t length = dxl::readLe16(&rx_[5]);
      const size_t total = (size_t)length + 7u;
      if (total < 10u || total > sizeof(rx_)) {
        dropBytes(1);
        continue;
      }
      if (rx_len_ < total) return;

      const uint16_t want_crc = dxl::readLe16(&rx_[total - 2]);
      const uint16_t have_crc = dxl::updateCrc(0, rx_, total - 2);
      if (want_crc != have_crc) {
        dropBytes(1);
        continue;
      }

      handlePacket(rx_, total);
      dropBytes(total);
    }
  }

  void sendStatus(uint8_t error, const uint8_t* params, size_t param_len) {
    if (param_len > 220u) return;
    uint8_t packet[256];
    size_t n = 0;
    packet[n++] = 0xFF;
    packet[n++] = 0xFF;
    packet[n++] = 0xFD;
    packet[n++] = 0x00;
    packet[n++] = kPeripheralDxlId;
    const uint16_t length = (uint16_t)(param_len + 4u);
    packet[n++] = (uint8_t)(length & 0xFFu);
    packet[n++] = (uint8_t)((length >> 8) & 0xFFu);
    packet[n++] = 0x55;
    packet[n++] = error;
    for (size_t i = 0; i < param_len; ++i) packet[n++] = params[i];
    const uint16_t crc = dxl::updateCrc(0, packet, n);
    packet[n++] = (uint8_t)(crc & 0xFFu);
    packet[n++] = (uint8_t)((crc >> 8) & 0xFFu);
    port_.write(packet, n);
    port_.flush();
  }

  void sendSimpleStatus(uint8_t error) { sendStatus(error, nullptr, 0); }

  void handlePing() {
    uint8_t params[3];
    dxl::writeLe16(&params[0], kModelNumber);
    params[2] = kFirmwareVersion;
    sendStatus(0, params, sizeof(params));
  }

  void handleRead(const uint8_t* params, size_t param_len) {
    if (param_len < 4u) {
      sendSimpleStatus(0x05);
      return;
    }
    const uint16_t addr = dxl::readLe16(params + 0);
    const uint16_t len = dxl::readLe16(params + 2);
    updateReadOnlyControlTable(millis());
    if (!ctInRange(addr, len) || len > 128u) {
      sendSimpleStatus(0x04);
      return;
    }
    uint8_t out[128];
    memcpy(out, &gControlTable[addr], len);
    sendStatus(0, out, len);
  }

  void handleWrite(const uint8_t* params, size_t param_len, bool send_reply) {
    if (param_len < 3u) {
      if (send_reply) sendSimpleStatus(0x05);
      return;
    }
    const uint16_t addr = dxl::readLe16(params + 0);
    const uint16_t len = (uint16_t)(param_len - 2u);
    if (!ctInRange(addr, len)) {
      if (send_reply) sendSimpleStatus(0x04);
      return;
    }
    memcpy(&gControlTable[addr], params + 2, len);
    const uint8_t err = applyControlTableWrite(addr, len);
    if (send_reply) sendSimpleStatus(err);
  }

  void handleSyncWrite(const uint8_t* params, size_t param_len) {
    if (param_len < 5u) return;
    const uint16_t addr = dxl::readLe16(params + 0);
    const uint16_t len = dxl::readLe16(params + 2);
    if (len == 0u) return;
    size_t p = 4;
    while (p + 1u + len <= param_len) {
      const uint8_t id = params[p++];
      if (id == kPeripheralDxlId) {
        if (ctInRange(addr, len)) {
          memcpy(&gControlTable[addr], params + p, len);
          applyControlTableWrite(addr, len);
        }
        return;
      }
      p += len;
    }
  }

  void handleBulkWrite(const uint8_t* params, size_t param_len) {
    size_t p = 0;
    while (p + 5u <= param_len) {
      const uint8_t id = params[p++];
      const uint16_t addr = dxl::readLe16(params + p);
      p += 2;
      const uint16_t len = dxl::readLe16(params + p);
      p += 2;
      if (p + len > param_len) return;
      if (id == kPeripheralDxlId) {
        if (ctInRange(addr, len)) {
          memcpy(&gControlTable[addr], params + p, len);
          applyControlTableWrite(addr, len);
        }
        return;
      }
      p += len;
    }
  }

  void handlePacket(const uint8_t* packet, size_t total_len) {
    if (total_len < 10u) return;
    const uint8_t target_id = packet[4];
    const uint8_t instruction = packet[7];
    const uint8_t* params = packet + 8;
    const size_t param_len = total_len - 10u;

    const bool is_broadcast = (target_id == 0xFEu);
    if (target_id != kPeripheralDxlId && !is_broadcast) return;

    switch (instruction) {
      case 0x01:  // ping
        if (target_id == kPeripheralDxlId || is_broadcast) handlePing();
        break;
      case 0x02:  // read
        if (!is_broadcast) handleRead(params, param_len);
        break;
      case 0x03:  // write
        handleWrite(params, param_len, !is_broadcast);
        break;
      case 0x83:  // sync write
        if (is_broadcast) handleSyncWrite(params, param_len);
        break;
      case 0x93:  // bulk write
        if (is_broadcast) handleBulkWrite(params, param_len);
        break;
      default:
        if (!is_broadcast) sendSimpleStatus(0x02);
        break;
    }
  }

  HardwareSerial& port_;
  uint8_t rx_[kPeripheralDxlMaxPacket] = {};
  size_t rx_len_ = 0;
};

PeripheralDxlNode gPeripheralNode(Serial1);

void printHelp(Print& out) {
  out.println();
  out.println(F("Commands:"));
  out.println(F("  /eye/...            existing eye driver commands"));
  out.println(F("  /eyes/...           plural alias for /eye/..."));
  out.println(F("  /eyes/on | /eyes/off | /eyes/normal"));
  out.println(F("  /antenna/idle | /antenna/off | /antenna/excited | /antenna/happy"));
  out.println(F("  /antenna/scared | /antenna/anxious | /antenna/angry | /antenna/curious | /antenna/retract"));
  out.println(F("  /antenna/yes | /antenna/no | /antenna/scan"));
  out.println(F("  /antenna/left <-1..1> | /antenna/right <-1..1> | /antenna/both <-1..1>"));
  out.println(F("  /antenna/center | /antenna/status"));
  out.println(F("  /ears/...           alias for /antenna/..."));
  out.println(F("  /torch/on [level01] [timeout_ms] | /torch/off | /torch/level <0..1> | /torch/status"));
  out.println(F("  /audio/volume <0..1> | /audio/play/index <track1-based> [clip0-based] | /audio/status | /audio/stop"));
  out.println(F("  /audio/reload/antenna"));
  out.println(F("  /flash/status | /flash/list"));
  out.println(F("  /flash/save/config | /flash/load/config | /flash/erase/config"));
  out.println(F("  /flash/set <key> <text...> | /flash/get <key> | /flash/delete <key>"));
  out.println(F("  /flash/import <sd_path> <asset_name> | /flash/export <asset_name> <sd_path>"));
  out.println(F("  /flash/import/antenna [sd_path] | /flash/import/warbler [sd_path] | /flash/clear/antenna"));
  out.println(F("  /status"));
  out.println(F("  help"));
  out.println();
}

bool dispatchEyeCommand(const char* line, Print& out) {
  if (line == nullptr) return false;

  if (strncmp(line, "/eye", 4) == 0) {
    const bool handled = gEyes.handleOscCommand(line, &out);
    gCtl.eye_brightness = gEyes.left().brightness();
    gCtl.eye_effort = normToRaw1000(gEyes.left().narrow());
    gCtl.eye_mode = (uint8_t)gEyes.left().mode();
    mirrorWritableShadowToControlTable();
    return handled;
  }

  if (strncmp(line, "/eyes/", 6) == 0) {
    const char* rest = line + 6;
    if (strcmp(rest, "on") == 0) {
      setEyesModeCommand(1);
      out.println(F("OK"));
      return true;
    }
    if (strcmp(rest, "off") == 0) {
      setEyesModeCommand(2);
      out.println(F("OK"));
      return true;
    }
    if (strcmp(rest, "normal") == 0) {
      setEyesModeCommand(0);
      out.println(F("OK"));
      return true;
    }

    char mapped[kLineBufLen];
    snprintf(mapped, sizeof(mapped), "/eye/%s", rest);
    const bool handled = gEyes.handleOscCommand(mapped, &out);
    gCtl.eye_brightness = gEyes.left().brightness();
    gCtl.eye_effort = normToRaw1000(gEyes.left().narrow());
    gCtl.eye_mode = (uint8_t)gEyes.left().mode();
    mirrorWritableShadowToControlTable();
    return handled;
  }

  return false;
}

bool parseAntennaAxisCommand(const char* line, const char* prefix, float& value_out) {
  if (strncmp(line, prefix, strlen(prefix)) != 0) return false;
  if (line[strlen(prefix)] != ' ') return false;
  value_out = strtof(line + strlen(prefix) + 1, nullptr);
  value_out = clampf(value_out, -1.0f, 1.0f);
  return true;
}

bool dispatchAntennaCommand(const char* original_line, Print& out) {
  if (original_line == nullptr) return false;

  char line[kLineBufLen];
  strncpy(line, original_line, sizeof(line) - 1);
  line[sizeof(line) - 1] = 0;

  if (strncmp(line, "/ears/", 6) == 0) {
    char mapped[kLineBufLen];
    snprintf(mapped, sizeof(mapped), "/antenna/%s", line + 6);
    strncpy(line, mapped, sizeof(line) - 1);
    line[sizeof(line) - 1] = 0;
  }

  if (strncmp(line, "/antenna", 8) != 0) return false;

  if (strcmp(line, "/antenna/center") == 0) {
    gAntenna.clearManual();
    setAntennaModeCommand((uint8_t)AntennaShow::kIdle);
    setAntennaManualBothCommand(0);
    out.println(F("OK"));
    return true;
  }

  if (strcmp(line, "/antenna/status") == 0) {
    printStatus(out);
    return true;
  }

  if (strcmp(line, "/antenna/off") == 0 ||
      strcmp(line, "/antenna/idle") == 0 ||
      strcmp(line, "/antenna/excited") == 0 ||
      strcmp(line, "/antenna/happy") == 0 ||
      strcmp(line, "/antenna/scared") == 0 ||
      strcmp(line, "/antenna/anxious") == 0 ||
      strcmp(line, "/antenna/angry") == 0 ||
      strcmp(line, "/antenna/curious") == 0 ||
      strcmp(line, "/antenna/retract") == 0) {
    const char* state = strrchr(line, '/');
    if (state != nullptr) {
      ++state;
      setAntennaModeCommand((uint8_t)AntennaShow::parseBaseMode(state));
      out.println(F("OK"));
      return true;
    }
  }

  if (strcmp(line, "/antenna/yes") == 0 || strcmp(line, "/antenna/no") == 0 || strcmp(line, "/antenna/scan") == 0) {
    const char* clip = strrchr(line, '/');
    if (clip != nullptr) {
      ++clip;
      triggerAntennaClipCommand((uint8_t)AntennaShow::parseClip(clip));
      out.println(F("OK"));
      return true;
    }
  }

  float value = 0.0f;
  if (parseAntennaAxisCommand(line, "/antenna/left", value)) {
    setAntennaManualLeftCommand(normToSignedRaw1000(value));
    out.println(F("OK"));
    return true;
  }
  if (parseAntennaAxisCommand(line, "/antenna/right", value)) {
    setAntennaManualRightCommand(normToSignedRaw1000(value));
    out.println(F("OK"));
    return true;
  }
  if (parseAntennaAxisCommand(line, "/antenna/both", value)) {
    setAntennaManualBothCommand(normToSignedRaw1000(value));
    out.println(F("OK"));
    return true;
  }

  out.println(F("ERR unknown antenna command"));
  return true;
}

bool dispatchTorchCommand(const char* line, Print& out) {
  if (line == nullptr) return false;
  if (strncmp(line, "/torch", 6) != 0) return false;

  if (strcmp(line, "/torch/off") == 0) {
    setTorchCommandRaw(0, gCtl.flashlight_timeout_ms, false);
    out.println(gTorch.ready() ? F("OK") : F("ERR torch unavailable"));
    return true;
  }

  if (strcmp(line, "/torch/status") == 0) {
    printStatus(out);
    return true;
  }

  if (strncmp(line, "/torch/level ", 13) == 0) {
    const float level = clampf(strtof(line + 13, nullptr), 0.0f, 1.0f);
    setTorchCommandRaw(normToRaw1000(level), gCtl.flashlight_timeout_ms, level > 0.0f);
    out.println(gTorch.ready() ? F("OK") : F("ERR torch unavailable"));
    return true;
  }

  if ((strcmp(line, "/torch/on") == 0) || (strncmp(line, "/torch/on ", 10) == 0)) {
    float level = 1.0f;
    uint32_t timeout_ms = kTorchDefaultTimeoutMs;

    if (line[9] == ' ') {
      char buffer[kLineBufLen];
      strncpy(buffer, line + 10, sizeof(buffer) - 1);
      buffer[sizeof(buffer) - 1] = 0;

      char* save = nullptr;
      char* tok = strtok_r(buffer, " \t", &save);
      if (tok != nullptr) {
        level = clampf(strtof(tok, nullptr), 0.0f, 1.0f);
        tok = strtok_r(nullptr, " \t", &save);
        if (tok != nullptr) {
          const long parsed = strtol(tok, nullptr, 10);
          if (parsed >= 0) timeout_ms = (uint32_t)parsed;
        }
      }
    }

    setTorchCommandRaw(normToRaw1000(level), timeout_ms, true);
    out.println(gTorch.ready() ? F("OK") : F("ERR torch unavailable"));
    return true;
  }

  out.println(F("ERR unknown torch command"));
  return true;
}

bool dispatchAudioCommand(const char* line, Print& out) {
  if (line == nullptr) return false;
  if (strncmp(line, "/audio", 6) != 0) return false;

  if (strcmp(line, "/audio/status") == 0) {
    printAudioStatus(out);
    return true;
  }

  if (strcmp(line, "/audio/stop") == 0) {
    gWarblerPodPlayer.stop();
    gAuxClip.stop();
    gAudio.active_aux_index = 0;
    gAudio.active_aux_arg0 = 0;
    out.println(F("OK"));
    return true;
  }

  if (strcmp(line, "/audio/reload/antenna") == 0) {
    invalidateCachedAudioAssets();
    const bool ok = ensureAntennaLoopAssetLoaded(millis());
    out.println(ok ? F("OK") : F("ERR no antenna loop asset found"));
    return true;
  }

  if (strncmp(line, "/audio/volume ", 14) == 0) {
    const float v = clampf(strtof(line + 14, nullptr), 0.0f, 1.0f);
    setMasterVolumeCommandRaw(normToRaw1000(v));
    out.println(F("OK"));
    return true;
  }

  if (strncmp(line, "/audio/play/index ", 18) == 0) {
    char buffer[kLineBufLen];
    strncpy(buffer, line + 18, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = 0;

    char* save = nullptr;
    char* tok = strtok_r(buffer, " 	", &save);
    uint16_t index = 0;
    uint32_t arg0 = 0;
    if (tok != nullptr) {
      long parsed = strtol(tok, nullptr, 10);
      if (parsed > 0) index = (uint16_t)parsed;
      tok = strtok_r(nullptr, " 	", &save);
      if (tok != nullptr) {
        long parsed_arg = strtol(tok, nullptr, 10);
        if (parsed_arg >= 0) arg0 = (uint32_t)parsed_arg;
      }
    }

    if (index == 0u) {
      out.println(F("ERR bad sound index"));
    } else if (playIndexedSound(index, arg0)) {
      gCtl.playsound_arg0 = arg0;
      ctSetU32(ADDR_PLAYSOUND_ARG0, gCtl.playsound_arg0);
      out.println(F("OK"));
    } else {
      out.println(F("ERR sound not found in flash or SD"));
    }
    return true;
  }

  out.println(F("ERR unknown audio command"));
  return true;
}

bool dispatchFlashCommand(const char* line, Print& out) {
  if (line == nullptr) return false;
  if (strncmp(line, "/flash", 6) != 0) return false;

  if (strcmp(line, "/flash/status") == 0) {
    printFlashStatus(out);
    return true;
  }

  if (strcmp(line, "/flash/list") == 0) {
    gFlashStore.list(out);
    return true;
  }

  if (strcmp(line, "/flash/save/config") == 0) {
    gFlashLastResult = gFlashStore.ready()
        ? (saveRuntimeConfigToFlash() ? kFlashResultOk : kFlashResultIoError)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR save failed"));
    return true;
  }

  if (strcmp(line, "/flash/load/config") == 0) {
    gFlashLastResult = gFlashStore.ready()
        ? (loadRuntimeConfigFromFlash() ? kFlashResultOk : kFlashResultBadData)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR load failed"));
    return true;
  }

  if (strcmp(line, "/flash/erase/config") == 0) {
    gFlashLastResult = gFlashStore.ready()
        ? (eraseRuntimeConfigInFlash() ? kFlashResultOk : kFlashResultIoError)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR erase failed"));
    return true;
  }

  if (strcmp(line, "/flash/clear/antenna") == 0) {
    gFlashLastResult = executeFlashOp(kFlashOpClearAntennaLoopCache);
    out.println(F("OK"));
    return true;
  }

  if (strcmp(line, "/flash/import/antenna") == 0) {
    gFlashLastResult = gFlashStore.ready()
        ? (importDefaultAntennaLoopAssetToFlash() ? kFlashResultOk : kFlashResultNotFound)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR import failed"));
    return true;
  }

  if (strncmp(line, "/flash/import/antenna ", 22) == 0) {
    const char* sd_path = line + 22;
    while (*sd_path == ' ' || *sd_path == '	') ++sd_path;
    gFlashLastResult = gFlashStore.ready()
        ? (gFlashStore.copySdFileToAsset(sd_path, "antenna_loop.wav") ? kFlashResultOk : kFlashResultNotFound)
        : kFlashResultNoFlash;
    if (gFlashLastResult == kFlashResultOk) invalidateCachedAudioAssets();
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR import failed"));
    return true;
  }

  if (strcmp(line, "/flash/import/warbler") == 0) {
    gFlashLastResult = gFlashStore.ready()
        ? (maybeInstallWarblerPodFromSd() ? kFlashResultOk : kFlashResultNotFound)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR import failed"));
    return true;
  }

  if (strncmp(line, "/flash/import/warbler ", 22) == 0) {
    const char* sd_path = line + 22;
    while (*sd_path == ' ' || *sd_path == '\t') ++sd_path;
    gFlashLastResult = gFlashStore.ready()
        ? (importWarblerPodAssetFromSd(sd_path, true) ? kFlashResultOk : kFlashResultNotFound)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR import failed"));
    return true;
  }

  if (strncmp(line, "/flash/set ", 11) == 0) {
    const char* args = line + 11;
    while (*args == ' ' || *args == '	') ++args;
    if (*args == 0) {
      out.println(F("ERR usage: /flash/set <key> <text...>"));
      return true;
    }

    const char* space = args;
    while (*space != 0 && *space != ' ' && *space != '	') ++space;
    if (*space == 0) {
      out.println(F("ERR usage: /flash/set <key> <text...>"));
      return true;
    }

    char key[48];
    const size_t key_len = (size_t)(space - args);
    if (key_len == 0u || key_len >= sizeof(key)) {
      out.println(F("ERR bad key"));
      return true;
    }
    memcpy(key, args, key_len);
    key[key_len] = 0;

    while (*space == ' ' || *space == '	') ++space;
    gFlashLastResult = gFlashStore.ready()
        ? (gFlashStore.writeTextKey(key, space) ? kFlashResultOk : kFlashResultIoError)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR write failed"));
    return true;
  }

  if (strncmp(line, "/flash/get ", 11) == 0) {
    const char* key = line + 11;
    while (*key == ' ' || *key == '	') ++key;
    gFlashLastResult = gFlashStore.ready()
        ? (gFlashStore.printTextKey(key, out) ? kFlashResultOk : kFlashResultNotFound)
        : kFlashResultNoFlash;
    if (gFlashLastResult != kFlashResultOk) out.println(F("ERR key not found"));
    return true;
  }

  if (strncmp(line, "/flash/delete ", 14) == 0) {
    const char* key = line + 14;
    while (*key == ' ' || *key == '	') ++key;
    gFlashLastResult = gFlashStore.ready()
        ? (gFlashStore.deleteTextKey(key) ? kFlashResultOk : kFlashResultNotFound)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR delete failed"));
    return true;
  }

  if (strncmp(line, "/flash/import ", 14) == 0) {
    char buffer[kLineBufLen];
    strncpy(buffer, line + 14, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = 0;

    char* save = nullptr;
    char* sd_path = strtok_r(buffer, " 	", &save);
    char* asset_name = strtok_r(nullptr, " 	", &save);
    if (sd_path == nullptr || asset_name == nullptr) {
      out.println(F("ERR usage: /flash/import <sd_path> <asset_name>"));
      return true;
    }

    gFlashLastResult = gFlashStore.ready()
        ? (gFlashStore.copySdFileToAsset(sd_path, asset_name) ? kFlashResultOk : kFlashResultNotFound)
        : kFlashResultNoFlash;
    if (gFlashLastResult == kFlashResultOk) {
      invalidateCachedAudioAssets();
      if (strcmp(asset_name, "warbler.pod") == 0) {
        gWarblerPodPlayer.stop();
        gWarblerPodCatalog.unload();
        WarblerPodCatalog verify_catalog;
        if (!verify_catalog.loadPath(kFlashWarblerPodAsset)) {
          gFlashStore.removePath(kFlashWarblerPodAsset);
          gFlashLastResult = kFlashResultBadData;
        }
      }
    }
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR import failed"));
    return true;
  }

  if (strncmp(line, "/flash/export ", 14) == 0) {
    char buffer[kLineBufLen];
    strncpy(buffer, line + 14, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = 0;

    char* save = nullptr;
    char* asset_name = strtok_r(buffer, " 	", &save);
    char* sd_path = strtok_r(nullptr, " 	", &save);
    if (asset_name == nullptr || sd_path == nullptr) {
      out.println(F("ERR usage: /flash/export <asset_name> <sd_path>"));
      return true;
    }

    gFlashLastResult = gFlashStore.ready()
        ? (gFlashStore.copyAssetToSd(asset_name, sd_path) ? kFlashResultOk : kFlashResultNotFound)
        : kFlashResultNoFlash;
    out.println((gFlashLastResult == kFlashResultOk) ? F("OK") : F("ERR export failed"));
    return true;
  }

  out.println(F("ERR unknown flash command"));
  return true;
}

void handleLine(const char* line, Print& out) {
  if (line == nullptr || line[0] == 0) return;

  if (strcmp(line, "help") == 0 || strcmp(line, "/help") == 0) {
    printHelp(out);
    return;
  }

  if (strcmp(line, "status") == 0 || strcmp(line, "/status") == 0) {
    printStatus(out);
    return;
  }

  if (dispatchEyeCommand(line, out)) return;
  if (dispatchAntennaCommand(line, out)) return;
  if (dispatchTorchCommand(line, out)) return;
  if (dispatchAudioCommand(line, out)) return;
  if (dispatchFlashCommand(line, out)) return;

  out.print(F("ERR unknown command: "));
  out.println(line);
}

template <typename PortT>
void pollConsole(PortT& port, char* buffer, size_t& len) {
  while (port.available() > 0) {
    const int raw = port.read();
    if (raw < 0) return;
    const char c = (char)raw;

    if (c == '\r' || c == '\n') {
      if (len > 0) {
        buffer[len] = 0;
        handleLine(buffer, port);
        len = 0;
      }
      continue;
    }

    if (c == 8 || c == 127) {
      if (len > 0) --len;
      continue;
    }

    if (len < (kLineBufLen - 1)) {
      buffer[len++] = c;
    }
  }
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(LOGO_LEFT_EYE, OUTPUT);
  pinMode(LOGO_RIGHT_EYE, OUTPUT);
  digitalWrite(LOGO_LEFT_EYE, LOW);
  digitalWrite(LOGO_RIGHT_EYE, LOW);

  Serial.begin(kConsoleBaud);
#if defined(USB_DUAL_SERIAL_AUDIO)
  SerialUSB1.begin(kConsoleBaud);
#endif
  delay(250);

  randomSeed((uint32_t)(micros() ^ millis() ^ 0x5A39C17Du));

  memset(gLampCacheStorage, 0, sizeof(gLampCacheStorage));
  memset(gAuxCacheStorage, 0, sizeof(gAuxCacheStorage));
  memset(gAntennaLoopStorage, 0, sizeof(gAntennaLoopStorage));

  configureUsbAudio();
  const bool flash_ready = gFlashStore.begin();
  const bool sd_ready = initSdAudio();
  const bool warbler_pod_installed = maybeInstallWarblerPodFromSd();
  const bool warbler_pod_ready = ensureWarblerPodCatalogLoaded();

  gEyes.begin();
  gEyes.setLeftOrientation(eye_driver::LedMatrixEye::ORIENTATION_CW);
  gEyes.setRightOrientation(eye_driver::LedMatrixEye::ORIENTATION_CCW);

  gAntenna.begin(millis());
  initAntennaBus();
  gPeripheralNode.begin(kPeripheralDxlBaud);

  gTorch.begin();

  initializeControlTable();

  bool restored_cfg = false;
  if (flash_ready && gFlashStore.configExists()) {
    restored_cfg = loadRuntimeConfigFromFlash();
    if (!restored_cfg) {
      gFlashLastResult = kFlashResultBadData;
    }
  }

  if (!restored_cfg) {
    setEyesModeCommand(gCtl.eye_mode);
    setEyesEffortCommand(gCtl.eye_effort);
    setEyesBrightnessCommand(gCtl.eye_brightness);
    setMasterVolumeCommandRaw(gCtl.volume_raw);
    setAntennaModeCommand(gCtl.antenna_mode);
    setTorchCommandRaw(gCtl.flashlight_raw, gCtl.flashlight_timeout_ms, false);
  }

  const bool antenna_loop_ready = ensureAntennaLoopAssetLoaded(millis());
  updateReadOnlyControlTable(millis());

  printlnBoth(F("QH4 eyes + ears + torch + USB audio + DXL peripheral"));
  printlnBoth(F("USB audio: stereo USB passthrough <-> primary I2S pins 7/20/21"));
  printlnBoth(F("Reactive audio: content bus feeds AudioFrequencyBitmap; AUX/antenna stay on non-reactive SFX bus"));
  printlnBoth(F("Antenna motors: Serial7 TTL @ 4M baud, IDs 5 and 6, bulk-write updates"));
  printlnBoth(F("Peripheral DXL node: Serial1 RS485 @ 4M baud, ID 10"));

  if (flash_ready) {
    Serial.print(F("Flash FS: ready "));
    Serial.print((unsigned long)gFlashStore.usedKilobytes());
    Serial.print(F("/"));
    Serial.print((unsigned long)gFlashStore.totalKilobytes());
    Serial.println(F(" KB"));
#if defined(USB_DUAL_SERIAL_AUDIO)
    SerialUSB1.print(F("Flash FS: ready "));
    SerialUSB1.print((unsigned long)gFlashStore.usedKilobytes());
    SerialUSB1.print(F("/"));
    SerialUSB1.print((unsigned long)gFlashStore.totalKilobytes());
    SerialUSB1.println(F(" KB"));
#endif
  } else {
    printlnBoth(F("Flash FS: unavailable or not mounted"));
  }

  if (restored_cfg) {
    printlnBoth(F("Flash config: restored at boot"));
  }

  if (sd_ready) {
    printlnBoth(F("SD audio: ready; lamp/sound WAVs are PSRAM-cached on first use"));
  } else {
    printlnBoth(F("SD audio: missing; flash assets and synthesized fallback still work"));
  }

  if (warbler_pod_installed) {
    printlnBoth(F("Warbler POD: installed from SD and removed from SD"));
  }
  if (warbler_pod_ready) {
    Serial.print(F("Warbler POD: "));
    Serial.print(storageSourceName(gWarblerPodCatalog.loadedSource()));
    Serial.print(F(" "));
    Serial.print(gWarblerPodCatalog.loadedPath());
    Serial.print(F(" tracks="));
    Serial.println((unsigned)gWarblerPodCatalog.numTracks());
#if defined(USB_DUAL_SERIAL_AUDIO)
    SerialUSB1.print(F("Warbler POD: "));
    SerialUSB1.print(storageSourceName(gWarblerPodCatalog.loadedSource()));
    SerialUSB1.print(F(" "));
    SerialUSB1.print(gWarblerPodCatalog.loadedPath());
    SerialUSB1.print(F(" tracks="));
    SerialUSB1.println((unsigned)gWarblerPodCatalog.numTracks());
#endif
  } else {
    printlnBoth(F("Warbler POD: not found; AUX sound will fall back to loose WAV assets"));
  }

  if (antenna_loop_ready) {
    Serial.print(F("Antenna loop WAV: "));
    Serial.print(storageSourceName(gAntennaLoopAsset.loadedSource()));
    Serial.print(F(" "));
    Serial.println(gAntennaLoopAsset.loadedPath());
#if defined(USB_DUAL_SERIAL_AUDIO)
    SerialUSB1.print(F("Antenna loop WAV: "));
    SerialUSB1.print(storageSourceName(gAntennaLoopAsset.loadedSource()));
    SerialUSB1.print(F(" "));
    SerialUSB1.println(gAntennaLoopAsset.loadedPath());
#endif
  } else {
    printlnBoth(F("Antenna loop WAV: not found; falling back to synthesized whine"));
  }

  if (gTorch.ready()) {
    Serial.print(F("Torch MCP4728 at 0x"));
    if (gTorch.address() < 0x10) Serial.print('0');
    Serial.println(gTorch.address(), HEX);
#if defined(USB_DUAL_SERIAL_AUDIO)
    SerialUSB1.print(F("Torch MCP4728 at 0x"));
    if (gTorch.address() < 0x10) SerialUSB1.print('0');
    SerialUSB1.println(gTorch.address(), HEX);
#endif
  } else {
    printlnBoth(F("Torch MCP4728 not found; /torch/* and ADDR_FLASHLIGHT will report unavailable"));
  }

  printFlashStatus(Serial);
  printAudioStatus(Serial);
  printHelp(Serial);
#if defined(USB_DUAL_SERIAL_AUDIO)
  printFlashStatus(SerialUSB1);
  printAudioStatus(SerialUSB1);
  printHelp(SerialUSB1);
#endif
}

void loop() {
  const uint32_t now_ms = millis();

  pollConsole(Serial, gSerialLine, gSerialLineLen);
#if defined(USB_DUAL_SERIAL_AUDIO)
  pollConsole(SerialUSB1, gSerialUsb1Line, gSerialUsb1LineLen);
#endif

  gPeripheralNode.service();
  serviceEyes(now_ms);
  serviceAntenna(now_ms);
  gTorch.service(now_ms);
  serviceClipPlayers();
  serviceAntennaWhine(now_ms);
  updateReadOnlyControlTable(now_ms);
  serviceHeartbeat(now_ms);
}
