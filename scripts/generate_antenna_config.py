"""
QuackHead config generator - generates AntennaConfig.h from YAML.
Similar approach to PuddleDuck's puddleduck_config.py.
"""
import os
import sys
import math

Import("env")

try:
    import yaml
except ImportError:
    env.Execute("$PYTHONEXE -m pip install pyyaml")
    import yaml


def deg_constructor(loader, node):
    value = loader.construct_scalar(node)
    return float(value)


def rad_constructor(loader, node):
    value = loader.construct_scalar(node)
    return math.degrees(float(value))


def get_loader():
    loader = yaml.SafeLoader
    loader.add_constructor('!deg', deg_constructor)
    loader.add_constructor('!rad', rad_constructor)
    return loader


def generate_antenna_config(yaml_path, output_path):
    with open(yaml_path, 'r') as f:
        data = yaml.load(f, Loader=get_loader())

    antenna = data.get('antenna', {})
    bus = antenna.get('bus', {})
    joints = antenna.get('joints', [])
    update_interval_ms = antenna.get('update_interval_ms', 20)
    keepalive_ms = antenna.get('keepalive_ms', 500)

    # Generate joint config structs
    joint_entries = []
    for j in joints:
        mode_map = {
            'Position': 'DYNAMIXEL::OP_POSITION',
            'Velocity': 'DYNAMIXEL::OP_VELOCITY',
            'Current': 'DYNAMIXEL::OP_CURRENT',
            'PWM': 'DYNAMIXEL::OP_PWM',
            'ExtendedPosition': 'DYNAMIXEL::OP_EXTENDED_POSITION',
            'CurrentBasedPosition': 'DYNAMIXEL::OP_CURRENT_BASED_POSITION',
        }
        mode = mode_map.get(j.get('mode', 'Position'), 'DYNAMIXEL::OP_POSITION')

        entry = (
            f"    {{\n"
            f"      .name = \"{j['name']}\",\n"
            f"      .id = {j['id']},\n"
            f"      .mode = {mode},\n"
            f"      .center = {j.get('center', 2048)},\n"
            f"      .sign = {j.get('sign', 1)},\n"
            f"      .travel = {j.get('travel', 520)},\n"
            f"      .min_counts = {j.get('min_counts', 0)},\n"
            f"      .max_counts = {j.get('max_counts', 4095)},\n"
            f"      .kp = {j.get('kp', 800)},\n"
            f"      .ki = {j.get('ki', 0)},\n"
            f"      .kd = {j.get('kd', 0)},\n"
            f"      .profile_acceleration = {j.get('profile_acceleration', 50)},\n"
            f"      .profile_velocity = {j.get('profile_velocity', 120)},\n"
            f"      .return_delay_time = {j.get('return_delay_time', 0)},\n"
            f"    }}"
        )
        joint_entries.append(entry)

    joints_str = ",\n".join(joint_entries)

    header = f"""\
// Auto-generated from {os.path.basename(yaml_path)} -- do not edit manually.
#pragma once

#include <stdint.h>
#include "DynamixelInternal.h"

namespace antenna_config {{

struct JointConfig {{
  const char* name;
  uint8_t id;
  DYNAMIXEL::DxlOperatingMode mode;
  int32_t center;
  int8_t sign;
  int32_t travel;
  int32_t min_counts;
  int32_t max_counts;
  int32_t kp;
  int32_t ki;
  int32_t kd;
  uint32_t profile_acceleration;
  uint32_t profile_velocity;
  uint8_t return_delay_time;
}};

struct BusConfig {{
  uint32_t baud;
  uint8_t txen_pin;
}};

struct AntennaConfig {{
  BusConfig bus;
  JointConfig joints[{len(joints)}];
  uint8_t joint_count;
  uint32_t update_interval_ms;
  uint32_t keepalive_ms;
}};

inline const AntennaConfig& getDefault() {{
  static const AntennaConfig cfg = {{
    .bus = {{
      .baud = {bus.get('baud', 4000000)},
      .txen_pin = {bus.get('txen_pin', 32)},
    }},
    .joints = {{
{joints_str}
    }},
    .joint_count = {len(joints)},
    .update_interval_ms = {update_interval_ms},
    .keepalive_ms = {keepalive_ms},
  }};
  return cfg;
}}

}}  // namespace antenna_config
"""

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        f.write(header)
    print(f"Generated {output_path}")


# --- PlatformIO build hook ---
project_dir = env.subst('$PROJECT_DIR')
build_dir = env.subst('$BUILD_DIR')

# Look for config override via environment variable or use default
config_name = os.environ.get('QUACKHEAD_CONFIG', 'default')
yaml_path = os.path.join(project_dir, 'configs', f'{config_name}.yaml')

if not os.path.isfile(yaml_path):
    print(f"WARNING: Config file not found: {yaml_path}, using configs/default.yaml")
    yaml_path = os.path.join(project_dir, 'configs', 'default.yaml')

if not os.path.isfile(yaml_path):
    print(f"ERROR: No config file found at {yaml_path}")
    sys.exit(1)

output_path = os.path.join(build_dir, 'AntennaConfig.h')
generate_antenna_config(yaml_path, output_path)

# Add build directory to include path so main.cpp can find AntennaConfig.h
env.Append(CPPPATH=[build_dir])

# Re-generate when yaml changes
env.Depends(os.path.join("$BUILD_DIR", "src", "main.cpp.o"), yaml_path)
