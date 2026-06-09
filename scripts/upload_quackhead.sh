#!/bin/bash

# --- defaults ---
FIRMWARE=""
UPLOAD_USER="${SSH_HEAD_UPLOAD_USER:-duck}"
UPLOAD_HOST="${SSH_HEAD_UPLOAD_HOST:-bdx-head.local}"

# --- parse flags ---
while [[ $# -gt 0 ]]; do
  case "$1" in
    -firmware)
      FIRMWARE="$2"
      shift 2
      ;;
    -user)
      UPLOAD_USER="$2"
      shift 2
      ;;
    -duckling)
      UPLOAD_HOST="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 [-firmware <hex-file>] [-user <user[:port]>] [-duckling <host[:port]>]"
      exit 1
      ;;
  esac
done

# --- normalize user[:port] ---
UPLOAD_USER_DEFAULT="${UPLOAD_USER##*:}"
UPLOAD_USER="${UPLOAD_USER%%:*}"
if [ -z "$UPLOAD_USER" ]; then
  UPLOAD_USER="$UPLOAD_USER_DEFAULT"
fi

# --- normalize host[:port] ---
UPLOAD_HOST_DEFAULT="${UPLOAD_HOST##*:}"
UPLOAD_HOST="${UPLOAD_HOST%%:*}"
if [ -z "$UPLOAD_HOST" ]; then
  UPLOAD_HOST="$UPLOAD_HOST_DEFAULT"
fi

echo "Uploading to ${UPLOAD_USER}@${UPLOAD_HOST}"
SSH_TARGET="${UPLOAD_USER}@${UPLOAD_HOST}"

ssh "${SSH_TARGET}" mkdir -p firmware

if [ -n "$FIRMWARE" ]; then
  echo "-> scp firmware: $FIRMWARE"
  scp "$FIRMWARE" "${SSH_TARGET}":firmware/quackhead_firmware.hex
else
  echo "-> no firmware file specified, skipping firmware upload"
  exit 0
fi

# build updater arguments
update_args=( -firmware firmware/quackhead_firmware.hex )

# run the updater
echo "-> update_quackhead ${update_args[*]}"
ssh "${SSH_TARGET}" /usr/local/bin/update_quackhead "${update_args[@]}"
