#!/bin/bash

# ESP8266 Flash Script
# Usage: espflash.sh <ip_address|usb_port> <bin_file>
# Example OTA:    espflash.sh 192.168.1.73 firmware.bin
# Example USB:    espflash.sh /dev/ttyUSB0 firmware.bin

PYTHON_PATH="/home/ryan/.arduino15/packages/esp8266/tools/python3/3.7.2-post1/python3"
ESPOTA_PATH="/home/ryan/.arduino15/packages/esp8266/hardware/esp8266/3.1.2/tools/espota.py"
UPLOAD_PATH="/home/ryan/.arduino15/packages/esp8266/hardware/esp8266/3.1.2/tools/upload.py"

# Check parameters
if [ $# -lt 2 ]; then
    echo "Usage: $0 <ip_address|usb_port> <bin_file>"
    echo ""
    echo "OTA mode:  $0 192.168.1.73 firmware.bin"
    echo "USB mode:  $0 /dev/ttyUSB0 firmware.bin"
    exit 1
fi

TARGET="$1"
BIN_FILE="$2"

# Check if bin file exists
if [ ! -f "$BIN_FILE" ]; then
    echo "Error: Bin file not found: $BIN_FILE"
    exit 1
fi

# Get absolute path for bin file
BIN_FILE="$(realpath "$BIN_FILE")"

# Detect mode based on first parameter
if [[ "$TARGET" == /dev/* ]]; then
    # USB/Serial mode
    echo "=== USB Flash Mode ==="
    echo "Port:    $TARGET"
    echo "File:    $BIN_FILE"
    echo ""

    $PYTHON_PATH -I "$UPLOAD_PATH" \
        --chip esp8266 \
        --port "$TARGET" \
        --baud 921600 \
        --before default_reset \
        --after hard_reset \
        write_flash 0x0 "$BIN_FILE"
else
    # OTA mode (assume IP address)
    echo "=== OTA Flash Mode ==="
    echo "IP:      $TARGET"
    echo "Port:    8266"
    echo "File:    $BIN_FILE"
    echo ""

    $PYTHON_PATH -I "$ESPOTA_PATH" \
        -i "$TARGET" \
        -p 8266 \
        --auth= \
        -f "$BIN_FILE"
fi
