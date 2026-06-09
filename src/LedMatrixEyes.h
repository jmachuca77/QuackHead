#pragma once

#include <Arduino.h>
#include <FastLED.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559f
#endif

namespace eye_driver {

static constexpr uint8_t  kMatrixWidth  = 12;
static constexpr uint8_t  kMatrixHeight = 12;
static constexpr uint16_t kNumLeds      = 121;
static constexpr uint16_t kInvalidLed   = 0xFFFF;

static constexpr uint8_t  kDefaultBrightness         = 50;
static constexpr uint32_t kDefaultBlinkMinDelayMs    = 1000;
static constexpr uint32_t kDefaultBlinkMaxDelayMs    = 4000;
static constexpr uint32_t kDefaultBlinkMinCloseMs    = 180;
static constexpr uint32_t kDefaultBlinkMaxCloseMs    = 200;
static constexpr uint32_t kDefaultAnimIntervalMs     = 50;

// With a 12-row eye, masking 6 rows per side blacks out the entire matrix.
// Reserve one row-pair of headroom so /eye/effort 1.0 leaves a narrow 2-row slit
// and the blink animation can still momentarily close the eye all the way.
static constexpr uint8_t kFullyClosedMaskRowsPerSide = (uint8_t)((kMatrixHeight + 1u) / 2u);
static constexpr uint8_t kMinVisibleRowsAtMaxEffort = 2;
static constexpr uint8_t kMaxEffortMaskRowsPerSide =
    (kMatrixHeight > kMinVisibleRowsAtMaxEffort)
        ? (uint8_t)((kMatrixHeight - kMinVisibleRowsAtMaxEffort) / 2u)
        : 0;

// Number of real LEDs in each visual row, top to bottom.
static const uint8_t kRowLen[kMatrixHeight] = {
  4, 8, 10, 10, 12, 12, 12, 12, 10, 10, 8, 4
};

// Starting LED index of each row in the physical chain.
static const uint16_t kRowStart[kMatrixHeight] = {
   0,   // row 0
   4,   // row 1
  12,   // row 2
  22,   // row 3
  32,   // row 4
  44,   // row 5
  56,   // row 6
  68,   // row 7
  80,   // row 8
  90,   // row 9
 100,   // row 10
 108    // row 11
};

// Physical rows zig-zag.
static const bool kRowReversed[kMatrixHeight] = {
  false, true, false, true, false, true,
  false, true, false, true, false, true
};

static inline float clampf(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static inline int clampi(int value, int lo, int hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static inline uint32_t clampu32(uint32_t value, uint32_t lo, uint32_t hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static inline uint32_t maxu32(uint32_t a, uint32_t b) {
  return (a > b) ? a : b;
}

// Map visual (x,y) onto packed LED index.
// Returns kInvalidLed when (x,y) is one of the empty corner cells.
static inline uint16_t XY(uint8_t x, uint8_t y) {
  if (x >= kMatrixWidth || y >= kMatrixHeight) return kInvalidLed;

  const uint8_t rowLen  = kRowLen[y];
  const uint8_t xOffset = (kMatrixWidth - rowLen) / 2;

  if (x < xOffset || x >= (uint8_t)(xOffset + rowLen)) {
    return kInvalidLed;
  }

  uint8_t localX = x - xOffset;
  if (kRowReversed[y]) {
    localX = rowLen - 1 - localX;
  }

  return kRowStart[y] + localX;
}

class LedMatrixEye {
public:
  enum Mode : uint8_t {
    MODE_NORMAL = 0,
    MODE_ON     = 1,
    MODE_OFF    = 2
  };

  // Physical board mount orientation relative to the canonical eye image.
  // The renderer compensates automatically so the eye still appears upright.
  enum Orientation : uint8_t {
    ORIENTATION_NORMAL = 0,
    ORIENTATION_CW     = 1,
    ORIENTATION_180    = 2,
    ORIENTATION_CCW    = 3
  };

  LedMatrixEye() {
    reset();
  }

  void reset() {
    mode_       = MODE_NORMAL;
    onColor_    = CRGB::White;
    brightness_ = kDefaultBrightness;
    narrow_     = 0.0f;

    minBlinkDelayMs_ = kDefaultBlinkMinDelayMs;
    maxBlinkDelayMs_ = kDefaultBlinkMaxDelayMs;
    minCloseMs_      = kDefaultBlinkMinCloseMs;
    maxCloseMs_      = kDefaultBlinkMaxCloseMs;
    intervalMs_      = kDefaultAnimIntervalMs;

    blinkState_  = 0;
    blinkRows_   = 0;
    nextTimeMs_  = 0;
    holdUntilMs_ = 0;

    swirlEnabled_ = true;
    swirlPhase_   = 0.0f;
    swirlSpeed_   = 1.0f;
    swirlArms_    = 2.0f;
    swirlTight_   = 6.0f;
    swirlAmount_  = 0.6f;

    edgeHintRed_   = 20;
    edgeHintWidth_ = 0.55f;

    lastAnimMs_ = 0;
  }

  void copyAnimationFrom(const LedMatrixEye& other) {
    blinkState_  = other.blinkState_;
    blinkRows_   = other.blinkRows_;
    nextTimeMs_  = other.nextTimeMs_;
    holdUntilMs_ = other.holdUntilMs_;
    swirlPhase_  = other.swirlPhase_;
    lastAnimMs_  = other.lastAnimMs_;
  }

  void copyAnimationSettingsFrom(const LedMatrixEye& other) {
    minBlinkDelayMs_ = other.minBlinkDelayMs_;
    maxBlinkDelayMs_ = other.maxBlinkDelayMs_;
    minCloseMs_      = other.minCloseMs_;
    maxCloseMs_      = other.maxCloseMs_;
    intervalMs_      = other.intervalMs_;

    swirlEnabled_ = other.swirlEnabled_;
    swirlSpeed_   = other.swirlSpeed_;
    swirlArms_    = other.swirlArms_;
    swirlTight_   = other.swirlTight_;
    swirlAmount_  = other.swirlAmount_;
  }

  static const char* orientationName(Orientation orientation) {
    switch (orientation) {
      case ORIENTATION_NORMAL: return "normal";
      case ORIENTATION_CW:     return "right90";
      case ORIENTATION_180:    return "180";
      case ORIENTATION_CCW:    return "left90";
      default:                 return "unknown";
    }
  }

  static bool parseOrientation(const char* value, Orientation& out) {
    if (value == nullptr) return false;

    if (strcmp(value, "normal") == 0 || strcmp(value, "0") == 0 || strcmp(value, "none") == 0 || strcmp(value, "upright") == 0) {
      out = ORIENTATION_NORMAL;
      return true;
    }

    if (strcmp(value, "cw") == 0 || strcmp(value, "cw90") == 0 || strcmp(value, "clockwise") == 0 ||
        strcmp(value, "right") == 0 || strcmp(value, "right90") == 0 || strcmp(value, "90") == 0) {
      out = ORIENTATION_CW;
      return true;
    }

    if (strcmp(value, "180") == 0 || strcmp(value, "flip") == 0 ||
        strcmp(value, "upside-down") == 0 || strcmp(value, "upsidedown") == 0) {
      out = ORIENTATION_180;
      return true;
    }

    if (strcmp(value, "ccw") == 0 || strcmp(value, "ccw90") == 0 || strcmp(value, "counterclockwise") == 0 ||
        strcmp(value, "left") == 0 || strcmp(value, "left90") == 0 || strcmp(value, "270") == 0 || strcmp(value, "-90") == 0) {
      out = ORIENTATION_CCW;
      return true;
    }

    return false;
  }

  // Convert a logical eye pixel into the physical board coordinate.
  // mirrorX is applied in logical space first, then board orientation compensation.
  static void mapLogicalToPhysical(
      uint8_t logicalX,
      uint8_t logicalY,
      Orientation mountOrientation,
      bool mirrorX,
      uint8_t& physicalX,
      uint8_t& physicalY) {
    uint8_t x = logicalX;
    const uint8_t y = logicalY;

    if (mirrorX) {
      x = (uint8_t)((kMatrixWidth - 1) - x);
    }

    switch (mountOrientation) {
      case ORIENTATION_NORMAL:
        physicalX = x;
        physicalY = y;
        break;

      case ORIENTATION_CW:
        // Board is mounted rotated clockwise, so output is rotated counter-clockwise.
        physicalX = y;
        physicalY = (uint8_t)((kMatrixHeight - 1) - x);
        break;

      case ORIENTATION_180:
        physicalX = (uint8_t)((kMatrixWidth - 1) - x);
        physicalY = (uint8_t)((kMatrixHeight - 1) - y);
        break;

      case ORIENTATION_CCW:
      default:
        // Board is mounted rotated counter-clockwise, so output is rotated clockwise.
        physicalX = (uint8_t)((kMatrixWidth - 1) - y);
        physicalY = x;
        break;
    }
  }

  void tick(uint32_t nowMs, bool forceAnimation = false) {
    const bool animateSwirl = forceAnimation ? swirlEnabled_ : (mode_ != MODE_OFF && swirlEnabled_);

    if (animateSwirl) {
      if (lastAnimMs_ == 0) lastAnimMs_ = nowMs;
      const float dt = (nowMs - lastAnimMs_) * 0.001f;
      lastAnimMs_ = nowMs;

      swirlPhase_ += swirlSpeed_ * dt;
      while (swirlPhase_ >= TWO_PI) swirlPhase_ -= TWO_PI;
      while (swirlPhase_ < 0.0f)   swirlPhase_ += TWO_PI;
    } else {
      lastAnimMs_ = nowMs;
    }

    const bool allowBlink = forceAnimation || (mode_ == MODE_NORMAL);
    if (!allowBlink) {
      blinkState_  = 0;
      blinkRows_   = 0;
      nextTimeMs_  = 0;
      holdUntilMs_ = 0;
      return;
    }

    if (nextTimeMs_ > nowMs) return;

    if (blinkState_ == 0) {
      const uint32_t delayMs = randRange(minBlinkDelayMs_, maxBlinkDelayMs_);
      nextTimeMs_  = nowMs + delayMs;
      blinkState_  = 1;
      blinkRows_   = 0;
      holdUntilMs_ = 0;
      return;
    }

    if (blinkState_ == 1) {
      if (blinkRows_ < kFullyClosedMaskRowsPerSide) {
        ++blinkRows_;
        nextTimeMs_ = nowMs + maxu32(1, intervalMs_);
        return;
      }

      const uint32_t closeMs = randRange(minCloseMs_, maxCloseMs_);
      holdUntilMs_ = nowMs + closeMs;
      nextTimeMs_  = holdUntilMs_;
      blinkState_  = 2;
      return;
    }

    if (blinkState_ == 2) {
      if (nowMs < holdUntilMs_) {
        nextTimeMs_ = holdUntilMs_;
        return;
      }

      if (blinkRows_ > 0) {
        --blinkRows_;
        nextTimeMs_ = nowMs + maxu32(1, intervalMs_);
        return;
      }

      blinkState_ = 0;
      nextTimeMs_ = nowMs + maxu32(1, intervalMs_);
    }
  }

  void renderTo(CRGB* out, Orientation mountOrientation = ORIENTATION_NORMAL, bool mirrorX = false) const {
    fill_solid(out, kNumLeds, CRGB::Black);

    if (mode_ == MODE_OFF) {
      return;
    }

    const float cx = (kMatrixWidth  - 1) * 0.5f;
    const float cy = (kMatrixHeight - 1) * 0.5f;
    const float r  = ((kMatrixWidth < kMatrixHeight) ? kMatrixWidth : kMatrixHeight) * 0.5f - 0.2f;
    const float ringW = (edgeHintWidth_ < 0.05f) ? 0.05f : edgeHintWidth_;

    for (int y = 0; y < kMatrixHeight; ++y) {
      for (int x = 0; x < kMatrixWidth; ++x) {
        const uint16_t logicalIdx = XY((uint8_t)x, (uint8_t)y);
        if (logicalIdx == kInvalidLed) continue;

        uint8_t physicalX = 0;
        uint8_t physicalY = 0;
        mapLogicalToPhysical((uint8_t)x, (uint8_t)y, mountOrientation, mirrorX, physicalX, physicalY);

        const uint16_t dstIdx = XY(physicalX, physicalY);
        if (dstIdx == kInvalidLed) continue;

        const float dx = x - cx;
        const float dy = y - cy;
        const float dist = sqrtf(dx * dx + dy * dy);

        CRGB pixel = CRGB::Black;

        if (dist <= r) {
          if (!swirlEnabled_) {
            pixel = onColor_;
          } else {
            const float rn     = dist / r;
            const float radial = powf(1.0f - rn, 1.1f);
            const float base   = 0.35f + 0.50f * radial;
            const float ang    = atan2f(dy, dx);
            const float wave   = sinf(ang * swirlArms_ + rn * swirlTight_ - swirlPhase_);
            const float amp    = swirlAmount_ * (0.35f + 0.65f * radial);

            float v = base + (wave * amp);
            v = clampf(v, 0.0f, 1.0f);

            pixel = onColor_;
            pixel.nscale8_video((uint8_t)lroundf(v * 255.0f));
          }
        } else if (edgeHintRed_ > 0) {
          const float outer = r + ringW;
          if (dist <= outer) {
            float t = (dist - r) / ringW;
            t = clampf(t, 0.0f, 1.0f);
            const float ring = 1.0f - t;
            pixel = CRGB((uint8_t)lroundf(edgeHintRed_ * ring), 0, 0);
          }
        }

        pixel.nscale8_video(brightness_);
        out[dstIdx] = pixel;
      }
    }

    const int baseRows = (int)effortMaskRows();
    const int blinkRows = clampi((int)blinkRows_, 0, (int)kFullyClosedMaskRowsPerSide);
    const int maskRows = (mode_ == MODE_ON) ? baseRows : ((baseRows > blinkRows) ? baseRows : blinkRows);

    if (maskRows <= 0) {
      return;
    }

    for (int y = 0; y < kMatrixHeight; ++y) {
      const bool masked = (y < maskRows) || (y >= (kMatrixHeight - maskRows));
      if (!masked) continue;

      for (int x = 0; x < kMatrixWidth; ++x) {
        const uint16_t logicalIdx = XY((uint8_t)x, (uint8_t)y);
        if (logicalIdx == kInvalidLed) continue;

        uint8_t physicalX = 0;
        uint8_t physicalY = 0;
        mapLogicalToPhysical((uint8_t)x, (uint8_t)y, mountOrientation, mirrorX, physicalX, physicalY);

        const uint16_t idx = XY(physicalX, physicalY);
        if (idx != kInvalidLed) {
          out[idx] = CRGB::Black;
        }
      }
    }
  }

  void setMode(Mode mode) { mode_ = mode; }
  Mode mode() const { return mode_; }

  void setBrightness(uint8_t brightness) { brightness_ = brightness; }
  uint8_t brightness() const { return brightness_; }

  void setColor(const CRGB& color) { onColor_ = color; }
  const CRGB& color() const { return onColor_; }

  void setNarrow(float value01) { narrow_ = clampf(value01, 0.0f, 1.0f); }
  float narrow() const { return narrow_; }
  uint8_t effortMaskRows() const { return narrowToMaskRows(narrow_); }
  uint8_t currentMaskRows() const {
    const uint8_t baseRows = effortMaskRows();
    const uint8_t blinkRows = (uint8_t)clampi((int)blinkRows_, 0, (int)kFullyClosedMaskRowsPerSide);
    return (mode_ == MODE_ON) ? baseRows : ((baseRows > blinkRows) ? baseRows : blinkRows);
  }

  void setBlinkPeriodMs(uint32_t periodMs) {
    const uint32_t minDelay = maxu32(50, (uint32_t)(periodMs * 0.75f));
    const uint32_t maxDelay = maxu32(minDelay, (uint32_t)(periodMs * 1.25f));
    minBlinkDelayMs_ = minDelay;
    maxBlinkDelayMs_ = maxDelay;
  }

  void setBlinkIntervalMs(uint32_t intervalMs) {
    intervalMs_ = maxu32(1, intervalMs);
  }

  void setBlinkCloseMs(uint32_t closeMs) {
    minCloseMs_ = closeMs;
    maxCloseMs_ = closeMs;
  }

  void setSwirlEnabled(bool enabled) { swirlEnabled_ = enabled; }
  bool swirlEnabled() const { return swirlEnabled_; }

  void setSwirlSpeed(float speed) { swirlSpeed_ = clampf(speed, -10.0f, 10.0f); }
  float swirlSpeed() const { return swirlSpeed_; }

  void setSwirlAmount(float amount01) {
    const float v = clampf(amount01, 0.0f, 1.0f);
    swirlAmount_ = v * 0.25f;
  }

  void setEdgeHint(uint8_t red) { edgeHintRed_ = red; }
  uint8_t edgeHint() const { return edgeHintRed_; }

  void setEdgeWidth(float width) {
    edgeHintWidth_ = clampf(width, 0.05f, 2.0f);
  }
  float edgeWidth() const { return edgeHintWidth_; }

private:
  static uint8_t narrowToMaskRows(float value01) {
    const int rows = (int)lroundf(clampf(value01, 0.0f, 1.0f) * (float)kMaxEffortMaskRowsPerSide);
    return (uint8_t)clampi(rows, 0, (int)kMaxEffortMaskRowsPerSide);
  }

  static uint32_t randRange(uint32_t a, uint32_t b) {
    if (a > b) {
      const uint32_t t = a;
      a = b;
      b = t;
    }
    if (a == b) return a;

    const long lo = (long)a;
    const long hiExclusive = (long)(b + 1U);
    return (uint32_t)random(lo, hiExclusive);
  }

  Mode mode_;
  CRGB onColor_;
  uint8_t brightness_;
  float narrow_;

  uint32_t minBlinkDelayMs_;
  uint32_t maxBlinkDelayMs_;
  uint32_t minCloseMs_;
  uint32_t maxCloseMs_;
  uint32_t intervalMs_;

  uint8_t blinkState_;
  uint8_t blinkRows_;
  uint32_t nextTimeMs_;
  uint32_t holdUntilMs_;

  bool  swirlEnabled_;
  float swirlPhase_;
  float swirlSpeed_;
  float swirlArms_;
  float swirlTight_;
  float swirlAmount_;

  uint8_t edgeHintRed_;
  float edgeHintWidth_;

  uint32_t lastAnimMs_;
};

template <uint8_t LEFT_PIN = 27, uint8_t RIGHT_PIN = 26>
class DualLedMatrixEyes {
public:
  using Orientation = LedMatrixEye::Orientation;

  DualLedMatrixEyes()
    : syncEnabled_(true),
      mirrorLeft_(false),
      mirrorRight_(true),
      leftOrientation_(LedMatrixEye::ORIENTATION_CW),
      rightOrientation_(LedMatrixEye::ORIENTATION_CCW),
      begun_(false) {
  }

  void begin() {
    if (begun_) return;

    static bool seeded = false;
    if (!seeded) {
      randomSeed((uint32_t)(micros() ^ millis() ^ 0x5A39C17Du));
      seeded = true;
    }

    FastLED.addLeds<WS2812B, LEFT_PIN, RGB>(leftLeds_, kNumLeds);
    FastLED.addLeds<WS2812B, RIGHT_PIN, RGB>(rightLeds_, kNumLeds);

    fill_solid(leftLeds_, kNumLeds, CRGB::Black);
    fill_solid(rightLeds_, kNumLeds, CRGB::Black);
    FastLED.show();

    leftEye_.reset();
    rightEye_.reset();
    syncEye_.reset();

    begun_ = true;
  }

  void tick(uint32_t nowMs = millis()) {
    if (syncEnabled_) {
      syncEye_.tick(nowMs, true);
      leftEye_.copyAnimationFrom(syncEye_);
      rightEye_.copyAnimationFrom(syncEye_);
    } else {
      leftEye_.tick(nowMs);
      rightEye_.tick(nowMs);
    }
  }

  void render() {
    leftEye_.renderTo(leftLeds_, leftOrientation_, mirrorLeft_);
    rightEye_.renderTo(rightLeds_, rightOrientation_, mirrorRight_);
  }

  void show() {
    FastLED.show();
  }

  void service(uint32_t nowMs = millis()) {
    tick(nowMs);
    render();
    show();
  }

  LedMatrixEye& left() { return leftEye_; }
  const LedMatrixEye& left() const { return leftEye_; }

  LedMatrixEye& right() { return rightEye_; }
  const LedMatrixEye& right() const { return rightEye_; }

  void setSynchronized(bool enabled) {
    if (syncEnabled_ == enabled) return;
    syncEnabled_ = enabled;

    if (syncEnabled_) {
      syncEye_.copyAnimationSettingsFrom(leftEye_);
      syncEye_.copyAnimationFrom(leftEye_);
    }
  }

  bool synchronized() const { return syncEnabled_; }

  void setMirrorLeft(bool enabled) { mirrorLeft_ = enabled; }
  void setMirrorRight(bool enabled) { mirrorRight_ = enabled; }
  bool mirrorLeft() const { return mirrorLeft_; }
  bool mirrorRight() const { return mirrorRight_; }

  void setLeftOrientation(Orientation orientation) { leftOrientation_ = orientation; }
  void setRightOrientation(Orientation orientation) { rightOrientation_ = orientation; }
  Orientation leftOrientation() const { return leftOrientation_; }
  Orientation rightOrientation() const { return rightOrientation_; }

  const CRGB* leftPixels() const { return leftLeds_; }
  const CRGB* rightPixels() const { return rightLeds_; }

  bool handleOscCommand(const String& line, Print* reply = nullptr) {
    return handleOscCommand(line.c_str(), reply);
  }

  bool handleOscCommand(const char* line, Print* reply = nullptr) {
    if (line == nullptr) return false;

    char buffer[192];
    size_t n = strlen(line);
    if (n >= sizeof(buffer)) n = sizeof(buffer) - 1;
    memcpy(buffer, line, n);
    buffer[n] = 0;

    const int kMaxTok = 8;
    char* tok[kMaxTok] = {0};
    int nt = 0;

    char* p = strtok(buffer, " \t\r\n");
    while (p != nullptr && nt < kMaxTok) {
      tok[nt++] = p;
      p = strtok(nullptr, " \t\r\n");
    }

    if (nt <= 0) return false;

    const char* addr = tok[0];
    if (strncmp(addr, "/eye", 4) != 0 || (addr[4] != 0 && addr[4] != '/')) {
      return false;
    }

    if (strcmp(addr, "/eye/sync") == 0) {
      if (nt < 2) return replyErr(reply, "usage: /eye/sync on|off");
      if (strcmp(tok[1], "on") == 0) {
        setSynchronized(true);
      } else if (strcmp(tok[1], "off") == 0) {
        setSynchronized(false);
      } else {
        return replyErr(reply, "sync must be on|off");
      }
      return replyOk(reply);
    }

    if (strcmp(addr, "/eye/status") == 0) {
      if (reply != nullptr) {
        reply->print(F("sync="));
        reply->print(syncEnabled_ ? F("on") : F("off"));
        reply->print(F(" leftMirror="));
        reply->print(mirrorLeft_ ? F("on") : F("off"));
        reply->print(F(" rightMirror="));
        reply->print(mirrorRight_ ? F("on") : F("off"));
        reply->print(F(" leftOrientation="));
        reply->print(LedMatrixEye::orientationName(leftOrientation_));
        reply->print(F(" rightOrientation="));
        reply->println(LedMatrixEye::orientationName(rightOrientation_));
      }
      return true;
    }

    Target target = TARGET_NONE;
    const char* command = nullptr;
    if (!decodeAddress(addr, target, command)) {
      return replyErr(reply, "unknown eye command");
    }

    if (strcmp(command, "mirror") == 0) {
      if (nt < 2) return replyErr(reply, "usage: /eye/<left|right|all>/mirror on|off");
      if (!isOnOff(tok[1])) return replyErr(reply, "mirror must be on|off");
      const bool value = parseOnOff(tok[1]);

      if (target == TARGET_LEFT || target == TARGET_BOTH) {
        mirrorLeft_ = value;
      }
      if (target == TARGET_RIGHT || target == TARGET_BOTH) {
        mirrorRight_ = value;
      }
      return replyOk(reply);
    }

    if (isOrientationCommand(command)) {
      if (nt < 2) return replyErr(reply, "usage: /eye/<left|right|all>/orientation normal|right90|left90|180");

      Orientation orientation = LedMatrixEye::ORIENTATION_NORMAL;
      if (!LedMatrixEye::parseOrientation(tok[1], orientation)) {
        return replyErr(reply, "orientation must be normal|right90|left90|180");
      }

      applyOrientationToTarget(target, orientation);
      return replyOk(reply);
    }

    const CommandResult result = applyCommandToTarget(target, command, nt - 1, &tok[1]);
    if (!result.handled) {
      return replyErr(reply, "unknown eye command");
    }
    if (!result.ok) {
      return replyErr(reply, result.err);
    }
    return replyOk(reply);
  }

private:
  enum Target : uint8_t {
    TARGET_NONE  = 0,
    TARGET_BOTH  = 1,
    TARGET_LEFT  = 2,
    TARGET_RIGHT = 3
  };

  struct CommandResult {
    bool handled;
    bool ok;
    const char* err;
  };

  static bool replyOk(Print* reply) {
    if (reply != nullptr) reply->println(F("OK"));
    return true;
  }

  static bool replyErr(Print* reply, const char* err) {
    if (reply != nullptr) {
      reply->print(F("ERR "));
      reply->println(err);
    }
    return true;
  }

  static bool decodeAddress(const char* addr, Target& target, const char*& command) {
    target = TARGET_NONE;
    command = nullptr;

    if (strncmp(addr, "/eye/left/", 10) == 0) {
      target = TARGET_LEFT;
      command = addr + 10;
      return true;
    }

    if (strncmp(addr, "/eye/right/", 11) == 0) {
      target = TARGET_RIGHT;
      command = addr + 11;
      return true;
    }

    if (strncmp(addr, "/eye/all/", 9) == 0) {
      target = TARGET_BOTH;
      command = addr + 9;
      return true;
    }

    if (strncmp(addr, "/eye/", 5) == 0) {
      target = TARGET_BOTH;
      command = addr + 5;
      return true;
    }

    return false;
  }

  static bool isOnOff(const char* s) {
    return (strcmp(s, "on") == 0) || (strcmp(s, "off") == 0);
  }

  static bool parseOnOff(const char* s) {
    return strcmp(s, "on") == 0;
  }

  static bool isOrientationCommand(const char* command) {
    return strcmp(command, "orientation") == 0 ||
           strcmp(command, "rotation") == 0 ||
           strcmp(command, "mount") == 0 ||
           strcmp(command, "map") == 0;
  }

  void applyOrientationToTarget(Target target, Orientation orientation) {
    if (target == TARGET_LEFT || target == TARGET_BOTH) {
      leftOrientation_ = orientation;
    }
    if (target == TARGET_RIGHT || target == TARGET_BOTH) {
      rightOrientation_ = orientation;
    }
  }

  CommandResult applyCommandToTarget(Target target, const char* command, int argc, char* argv[]) {
    if (target == TARGET_LEFT) {
      return applyEyeCommand(leftEye_, command, argc, argv);
    }

    if (target == TARGET_RIGHT) {
      return applyEyeCommand(rightEye_, command, argc, argv);
    }

    if (target == TARGET_BOTH) {
      const CommandResult leftResult  = applyEyeCommand(leftEye_, command, argc, argv);
      if (!leftResult.handled || !leftResult.ok) return leftResult;

      const CommandResult rightResult = applyEyeCommand(rightEye_, command, argc, argv);
      if (!rightResult.handled || !rightResult.ok) return rightResult;

      const CommandResult syncResult  = applyEyeCommand(syncEye_, command, argc, argv);
      if (!syncResult.handled || !syncResult.ok) return syncResult;

      return leftResult;
    }

    return {false, false, "bad target"};
  }

  static long parseLong(const char* s) {
    return strtol(s, nullptr, 10);
  }

  static CommandResult applyEyeCommand(LedMatrixEye& eye, const char* command, int argc, char* argv[]) {
    if (strcmp(command, "mode") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../mode normal|on|off"};
      if (strcmp(argv[0], "normal") == 0) eye.setMode(LedMatrixEye::MODE_NORMAL);
      else if (strcmp(argv[0], "on") == 0) eye.setMode(LedMatrixEye::MODE_ON);
      else if (strcmp(argv[0], "off") == 0) eye.setMode(LedMatrixEye::MODE_OFF);
      else return {true, false, "mode must be normal|on|off"};
      return {true, true, nullptr};
    }

    if (strcmp(command, "brightness") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../brightness 0..255"};
      eye.setBrightness((uint8_t)clampi(atoi(argv[0]), 0, 255));
      return {true, true, nullptr};
    }

    if (strcmp(command, "color") == 0) {
      if (argc < 3) return {true, false, "usage: /eye/.../color R G B"};
      const int r = clampi(atoi(argv[0]), 0, 255);
      const int g = clampi(atoi(argv[1]), 0, 255);
      const int b = clampi(atoi(argv[2]), 0, 255);
      eye.setColor(CRGB((uint8_t)r, (uint8_t)g, (uint8_t)b));
      return {true, true, nullptr};
    }

    if (strcmp(command, "edgehint") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../edgehint 0..255"};
      eye.setEdgeHint((uint8_t)clampi(atoi(argv[0]), 0, 255));
      return {true, true, nullptr};
    }

    if (strcmp(command, "edgewidth") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../edgewidth pixels"};
      eye.setEdgeWidth(strtof(argv[0], nullptr));
      return {true, true, nullptr};
    }

    if (strcmp(command, "effort") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../effort 0..1"};
      eye.setNarrow(strtof(argv[0], nullptr));
      return {true, true, nullptr};
    }

    if (strcmp(command, "blink/period") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../blink/period ms"};
      long value = parseLong(argv[0]);
      if (value < 0) value = 0;
      eye.setBlinkPeriodMs((uint32_t)value);
      return {true, true, nullptr};
    }

    if (strcmp(command, "blink/interval") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../blink/interval ms"};
      long value = parseLong(argv[0]);
      if (value < 1) value = 1;
      eye.setBlinkIntervalMs((uint32_t)value);
      return {true, true, nullptr};
    }

    if (strcmp(command, "blink/close") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../blink/close ms"};
      long value = parseLong(argv[0]);
      if (value < 0) value = 0;
      eye.setBlinkCloseMs((uint32_t)value);
      return {true, true, nullptr};
    }

    if (strcmp(command, "reset") == 0) {
      eye.reset();
      return {true, true, nullptr};
    }

    if (strcmp(command, "swirl") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../swirl on|off"};
      if (strcmp(argv[0], "on") == 0) eye.setSwirlEnabled(true);
      else if (strcmp(argv[0], "off") == 0) eye.setSwirlEnabled(false);
      else return {true, false, "swirl must be on|off"};
      return {true, true, nullptr};
    }

    if (strcmp(command, "swirl/speed") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../swirl/speed radians_per_sec"};
      eye.setSwirlSpeed(strtof(argv[0], nullptr));
      return {true, true, nullptr};
    }

    if (strcmp(command, "swirl/amount") == 0) {
      if (argc < 1) return {true, false, "usage: /eye/.../swirl/amount 0..1"};
      eye.setSwirlAmount(strtof(argv[0], nullptr));
      return {true, true, nullptr};
    }

    return {false, false, nullptr};
  }

  CRGB leftLeds_[kNumLeds];
  CRGB rightLeds_[kNumLeds];

  LedMatrixEye leftEye_;
  LedMatrixEye rightEye_;
  LedMatrixEye syncEye_;

  bool syncEnabled_;
  bool mirrorLeft_;
  bool mirrorRight_;
  Orientation leftOrientation_;
  Orientation rightOrientation_;
  bool begun_;
};

} // namespace eye_driver
