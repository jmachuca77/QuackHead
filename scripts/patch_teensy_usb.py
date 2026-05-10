from pathlib import Path

Import("env")

MARKER = "#elif defined(QUACKHEAD)"
BLOCK = r'''
#elif defined(QUACKHEAD)
  #define VENDOR_ID                0x16C0
  #define PRODUCT_ID               0x048B
  #define MANUFACTURER_NAME        {'S','k','e','l','b','o','t','s'}
  #define MANUFACTURER_NAME_LEN    8
  #define PRODUCT_NAME             {'Q','u','a','c','k','H','e','a','d'}
  #define PRODUCT_NAME_LEN         9
  #define EP0_SIZE                 64
  #define NUM_ENDPOINTS            7
  #define NUM_INTERFACE            7
  #define CDC_IAD_DESCRIPTOR       1
  #define CDC_STATUS_INTERFACE     0
  #define CDC_DATA_INTERFACE       1
  #define CDC_ACM_ENDPOINT         2
  #define CDC_RX_ENDPOINT          3
  #define CDC_TX_ENDPOINT          3
  #define CDC_ACM_SIZE             16
  #define CDC_RX_SIZE_480          512
  #define CDC_TX_SIZE_480          512
  #define CDC_RX_SIZE_12           64
  #define CDC_TX_SIZE_12           64
  #define CDC2_STATUS_INTERFACE    2
  #define CDC2_DATA_INTERFACE      3
  #define CDC2_ACM_ENDPOINT        4
  #define CDC2_RX_ENDPOINT         5
  #define CDC2_TX_ENDPOINT         5
  #define AUDIO_INTERFACE          4
  #define AUDIO_TX_ENDPOINT        6
  #define AUDIO_TX_SIZE            180
  #define AUDIO_RX_ENDPOINT        6
  #define AUDIO_RX_SIZE            180
  #define AUDIO_SYNC_ENDPOINT      7
  #define ENDPOINT2_CONFIG         ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_INTERRUPT
  #define ENDPOINT3_CONFIG         ENDPOINT_RECEIVE_BULK + ENDPOINT_TRANSMIT_BULK
  #define ENDPOINT4_CONFIG         ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_INTERRUPT
  #define ENDPOINT5_CONFIG         ENDPOINT_RECEIVE_BULK + ENDPOINT_TRANSMIT_BULK
  #define ENDPOINT6_CONFIG         ENDPOINT_RECEIVE_ISOCHRONOUS + ENDPOINT_TRANSMIT_ISOCHRONOUS
  #define ENDPOINT7_CONFIG         ENDPOINT_RECEIVE_UNUSED + ENDPOINT_TRANSMIT_ISOCHRONOUS
'''.strip("\n")


def patch_usb_desc() -> None:
    framework_dir = Path(env.PioPlatform().get_package_dir("framework-arduinoteensy"))
    usb_desc = framework_dir / "cores" / "teensy4" / "usb_desc.h"

    if not usb_desc.exists():
        raise FileNotFoundError(f"Could not find {usb_desc}")

    original = usb_desc.read_text(encoding="utf-8")
    if MARKER in original:
        print("[patch_teensy_usb] QUACKHEAD already present")
        return

    anchor = "#elif defined(USB_AUDIO)"
    if anchor not in original:
        raise RuntimeError("USB_AUDIO block not found in usb_desc.h; core layout changed")

    patched = original.replace(anchor, BLOCK + "\n\n" + anchor, 1)
    usb_desc.write_text(patched, encoding="utf-8")
    print(f"[patch_teensy_usb] Patched {usb_desc}")


patch_usb_desc()