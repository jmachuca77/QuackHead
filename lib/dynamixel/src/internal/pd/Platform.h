// Platform shim for QuackHead - provides pd::platform::currentTimeMillis()
// required by the ROBOTIS Dynamixel master library.
#pragma once

#include <Arduino.h>

namespace pd {
namespace platform {

inline uint32_t currentTimeMillis() {
  return millis();
}

}  // namespace platform
}  // namespace pd
