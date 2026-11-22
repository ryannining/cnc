#!/bin/bash

# ESP32 Flash Script
# Usage: esp32flash.sh <ip_address|usb_port> <bin_file|build_path>
# Example OTA:    esp32flash.sh 192.168.1.73 /tmp/arduino_build_xxx/cnc.ino.bin
# Example USB:    esp32flash.sh /dev/ttyACM0 /tmp/arduino_build_xxx/cnc.ino.bin
# Example USB:    esp32flash.sh /dev/ttyUSB0 /tmp/arduino_build_xxx

PYTHON_PATH="/home/ryan/.arduino15/packages/esp8266/tools/python3/3.7.2-post1/python3"
ESPOTA_PATH="/home/ryan/.arduino15/packages/esp8266/hardware/esp8266/3.1.2/tools/espota.py"
ESPTOOL_PATH="/home/ryan/.arduino15/packages/esp32/tools/esptool_py/5.1.0/esptool"
BOOT_APP0_PATH="/home/ryan/.arduino15/packages/esp32/hardware/esp32/3.3.6/tools/partitions/boot_app0.bin"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

# Check parameters
if [ $# -lt 2 ]; then
    echo "Usage: $0 <ip_address|usb_port> <bin_file|build_path>"
    echo ""
    echo "OTA mode:  $0 192.168.1.73 /tmp/arduino_build_xxx/cnc.ino.bin"
    echo "USB mode:  $0 /dev/ttyACM0 /tmp/arduino_build_xxx/cnc.ino.bin"
    echo "USB mode:  $0 /dev/ttyUSB0 /tmp/arduino_build_xxx"
    exit 1
fi

TARGET="$1"
BIN_PATH="$2"

# Detect mode based on first parameter
if [[ "$TARGET" == /dev/* ]]; then
    # USB/Serial mode
    echo -e "${BOLD}${BLUE}=== ESP32 USB Flash Mode ===${NC}"
    echo -e "  ${CYAN}Port:${NC} $TARGET"
    
    # Check if it's a build directory or bin file
    if [[ -d "$BIN_PATH" ]]; then
        BUILD_PATH="$BIN_PATH"
    elif [[ "$BIN_PATH" == *.bin ]]; then
        BUILD_PATH=$(dirname "$BIN_PATH")
    else
        echo -e "${RED}Error: Invalid path - must be .bin file or build directory${NC}"
        exit 1
    fi
    
    # Find project name
    BIN_FILE=$(ls "$BUILD_PATH"/*.ino.bin 2>/dev/null | head -1)
    if [ -z "$BIN_FILE" ]; then
        echo -e "${RED}Error: No .ino.bin file found in $BUILD_PATH${NC}"
        exit 1
    fi
    
    PROJECT_NAME=$(basename "$BIN_FILE" .ino.bin)
    BOOTLOADER="$BUILD_PATH/${PROJECT_NAME}.ino.bootloader.bin"
    PARTITIONS="$BUILD_PATH/${PROJECT_NAME}.ino.partitions.bin"
    
    # Check required files
    if [ ! -f "$BOOTLOADER" ]; then
        echo -e "${RED}Error: Bootloader not found: $BOOTLOADER${NC}"
        exit 1
    fi
    if [ ! -f "$PARTITIONS" ]; then
        echo -e "${RED}Error: Partitions not found: $PARTITIONS${NC}"
        exit 1
    fi
    
    echo -e "  ${CYAN}Project:${NC} $PROJECT_NAME"
    echo -e "  ${CYAN}Files:${NC}"
    echo -e "    ${GRAY}Bootloader:$(basename $BOOTLOADER)${NC}"
    echo -e "    ${GRAY}Partitions: $(basename $PARTITIONS)${NC}"
    echo -e "    ${GRAY}Firmware:   $(basename $BIN_FILE)${NC}"
    echo ""
    
    # Detect chip type from build path or default to esp32
    CHIP="esp32"
    if [[ "$BUILD_PATH" == *"c3"* ]] || ls "$BUILD_PATH"/*.map 2>/dev/null | xargs grep -l "esp32c3" >/dev/null 2>&1; then
        CHIP="esp32c3"
    fi
    
    $ESPTOOL_PATH --chip $CHIP --port "$TARGET" --baud 921600 \
        --before default-reset --after hard-reset \
        write-flash -z --flash-mode keep --flash-freq keep --flash-size keep \
        0x0 "$BOOTLOADER" \
        0x8000 "$PARTITIONS" \
        0xe000 "$BOOT_APP0_PATH" \
        0x10000 "$BIN_FILE"
else
    # OTA mode (IP address)
    echo -e "${BOLD}${BLUE}=== ESP32 OTA Flash Mode ===${NC}"
    
    # Find bin file
    if [[ -d "$BIN_PATH" ]]; then
        BIN_FILE=$(ls "$BIN_PATH"/*.ino.bin 2>/dev/null | head -1)
        if [ -z "$BIN_FILE" ]; then
            echo -e "${RED}Error: No .ino.bin file found in $BIN_PATH${NC}"
            exit 1
        fi
    else
        BIN_FILE="$BIN_PATH"
    fi
    
    if [ ! -f "$BIN_FILE" ]; then
        echo -e "${RED}Error: Bin file not found: $BIN_FILE${NC}"
        exit 1
    fi
    
    echo -e "  ${CYAN}IP:${NC}   $TARGET"
    echo -e "  ${CYAN}Port:${NC} 8266"
    echo -e "  ${CYAN}File:${NC} $BIN_FILE"
    echo ""
    
    $PYTHON_PATH -I "$ESPOTA_PATH" \
        -i "$TARGET" \
        -p 8266 \
        --auth= \
        -f "$BIN_FILE"
fi
