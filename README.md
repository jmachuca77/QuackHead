# QH4 Teensy control table, flash storage, and audio notes

This build does four control-plane things at once:

1. Drives XC330-T288 ears on `Serial7` at `4,000,000` baud using DYNAMIXEL bulk-write packets.
2. Exposes the Teensy itself as a custom DYNAMIXEL Protocol 2.0 peripheral on `Serial1` / RS485 at `4,000,000` baud with ID `10`.
3. Keeps the existing newline-delimited OSC-like serial command interface on USB serial.
4. Adds a LittleFS-backed external flash store for persisted config, small key/value text, and audio assets.

## Custom DYNAMIXEL node on UART1

Fixed node identity:

- ID: `10`
- Model number: `0x5148`
- Firmware version: `2`
- Protocol: DYNAMIXEL 2.0
- Implemented instructions:
  - `PING`
  - `READ`
  - `WRITE`
  - `SYNC_WRITE`
  - `BULK_WRITE`

`READ`/`WRITE` are the normal request/response path.
`SYNC_WRITE` and `BULK_WRITE` let a host include the Teensy peripheral in group commands with other DYNAMIXEL devices.

## Writable control registers

All values are little-endian.

- `0x40 / 64`  `ADDR_FLASHLIGHT` `uint16`
  - `0..1000`
  - `0` turns torch off
  - nonzero sets white torch level and turns it on
- `0x42 / 66`  `ADDR_FLASHLIGHT_TIMEOUT_MS` `uint32`
  - torch auto-off timeout
- `0x46 / 70`  `ADDR_SETVOLUME` `uint16`
  - overall digital audio volume `0..1000`
- `0x48 / 72`  `ADDR_PLAYSOUND_INDEX` `uint16`
  - command register
  - writing nonzero attempts to play `sound<index>.wav`
  - auto-clears back to `0`
- `0x4A / 74`  `ADDR_PLAYSOUND_ARG0` `uint32`
  - optional argument used by indexed sound lookup
  - checked first as `sound<index>_<arg0>.wav`
- `0x4E / 78`  `ADDR_ANTENNA_MODE` `uint8`
  - `0=off 1=idle 2=excited 3=happy 4=scared 5=anxious 6=angry 7=curious 8=retract`
- `0x4F / 79`  `ADDR_ANTENNA_CLIP` `uint8`
  - command register
  - `1=yes 2=no 3=scan`
  - auto-clears back to `0`
- `0x50 / 80`  `ADDR_ANTENNA_LEFT` `int16`
  - manual left override `-1000..1000`
- `0x52 / 82`  `ADDR_ANTENNA_RIGHT` `int16`
  - manual right override `-1000..1000`
- `0x54 / 84`  `ADDR_ANTENNA_BOTH` `int16`
  - manual both override `-1000..1000`
- `0x56 / 86`  `ADDR_EYE_MODE` `uint8`
  - `0=normal 1=on 2=off`
- `0x58 / 88`  `ADDR_EYE_EFFORT` `uint16`
  - eye narrowing `0..1000`
- `0x5A / 90`  `ADDR_EYE_BRIGHTNESS` `uint8`
  - `0..255`
- `0x70 / 112` `ADDR_FLASH_OP` `uint8`
  - command register, auto-clears back to `0`
  - `0=none`
  - `1=save current runtime config to flash`
  - `2=load runtime config from flash`
  - `3=erase saved runtime config`
  - `4=import default antenna loop WAV from SD into flash as /assets/antenna_loop.wav`
  - `5=clear cached antenna loop from PSRAM and force re-open next time`

## Read-only status registers

- `0x5C / 92`  `ADDR_STATUS_FLAGS` `uint32`
  - bit0 torch DAC ready
  - bit1 torch currently on
  - bit2 SD ready
  - bit3 lamp clip active
  - bit4 aux clip active
  - bit5 antenna bus initialized
  - bit6 dual USB serial + audio build enabled
  - bit7 flash FS ready
  - bit8 antenna motion loop active
- `0x60 / 96`  `ADDR_TORCH_REMAINING_MS` `uint32`
- `0x64 / 100` `ADDR_PRESENT_ANTENNA_LEFT` `int32`
- `0x68 / 104` `ADDR_PRESENT_ANTENNA_RIGHT` `int32`
- `0x6C / 108` `ADDR_ACTIVE_SOUND_INDEX` `uint16`
- `0x6E / 110` `ADDR_AUDIO_FLAGS` `uint8`
  - bit0 lamp clip cached in PSRAM
  - bit1 aux clip cached in PSRAM
  - bit2 antenna loop cached in PSRAM
  - bit3 antenna loop asset came from flash
- `0x71 / 113` `ADDR_FLASH_FLAGS` `uint8`
  - bit0 flash FS ready
  - bit1 saved config exists
  - bit2 antenna loop cached in PSRAM
  - bit3 antenna loop source is flash
  - bit4 `/assets/antenna_loop.wav` exists in flash
- `0x74 / 116` `ADDR_FLASH_USED_KB` `uint32`
- `0x78 / 120` `ADDR_FLASH_TOTAL_KB` `uint32`
- `0x7C / 124` `ADDR_FLASH_RESULT` `uint8`
  - `0=ok`
  - `1=no flash`
  - `2=not found`
  - `3=io error`
  - `4=bad op`
  - `5=bad data`

## External flash behavior

The firmware mounts the bottom-side QSPI flash with LittleFS and uses it for:

- `/cfg/runtime.bin`
  - persisted volume, torch timeout, eye mode/effort/brightness, antenna base mode
- `/kv/<key>.txt`
  - simple text key/value blobs for quick storage/retrieval from serial
- `/assets/...`
  - audio assets, including `/assets/antenna_loop.wav`
  - lamp and indexed sounds can also be stored here

## Audio behavior

- USB stereo passthrough stays on the primary I2S data pin.
- Antenna motion sound now prefers a looped WAV cached into PSRAM.
  - left ear motion feeds the left channel
  - right ear motion feeds the right channel
  - when no loop WAV is available, the sketch falls back to the synthesized whine
- Torch activation tries to play one of:
  - `/assets/lamp<seconds>.wav`
  - `/assets/lamp.wav`
  - `/sounds/lamp<seconds>.wav`
  - `/lamp<seconds>.wav`
  - `/sounds/lamp.wav`
  - `/lamp.wav`
- Indexed sounds try, in order:
  - `/assets/sound<index>_<arg0>.wav`
  - `/sounds/sound<index>_<arg0>.wav`
  - `/sound<index>_<arg0>.wav`
  - `/assets/sound<index>.wav`
  - `/sounds/sound<index>.wav`
  - `/sound<index>.wav`

## WAV format expected by the PSRAM cache

The cache loader currently expects:

- RIFF/WAVE PCM
- 16-bit little-endian samples
- 44.1 kHz
- mono or stereo

Stereo files are downmixed to mono before playback.

## OSC-like serial commands

### Existing high-level controls

- `/antenna/excited`, `/antenna/happy`, `/ears/scared`, etc.
- `/torch/on [level] [timeout_ms]`
- `/torch/off`
- `/audio/volume 0.0..1.0`
- `/audio/play/index <n> [arg0]`
- `/audio/reload/antenna`
- `/status`

### New flash commands

- `/flash/status`
- `/flash/list`
- `/flash/save/config`
- `/flash/load/config`
- `/flash/erase/config`
- `/flash/set <key> <text...>`
- `/flash/get <key>`
- `/flash/delete <key>`
- `/flash/import <sd_path> <asset_name>`
- `/flash/export <asset_name> <sd_path>`
- `/flash/import/antenna [sd_path]`
- `/flash/clear/antenna`
