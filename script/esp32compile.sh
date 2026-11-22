#!/bin/bash

# ESP32 Compile Script
# Usage: esp32compile.sh [board] [target] <project.ino>
# Example compile only:  esp32compile.sh project.ino
# Example with board:    esp32compile.sh lolin_c3_mini project.ino
# Example compile + OTA: esp32compile.sh lolin_c3_mini 192.168.1.73 project.ino
# Example compile + USB: esp32compile.sh lolin_c3_mini /dev/ttyACM0 project.ino

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GRAY='\033[0;90m'
BOLD='\033[1m'
NC='\033[0m'

ARDUINO_BUILDER="/usr/share/arduino/arduino-builder"
HARDWARE_PATHS="-hardware /usr/share/arduino/hardware -hardware /home/ryan/.arduino15/packages"
TOOLS_PATHS="-tools /usr/share/arduino/tools-builder -tools /home/ryan/.arduino15/packages"
LIBRARIES="-libraries /home/ryan/Arduino/libraries"

# Default board
DEFAULT_BOARD="lolin_c3_mini"
ESP32_VERSION="3.3.6"

ESPFLASH_SCRIPT="/home/ryan/esp32flash.sh"

# Function to format compile output
format_output() {
    local input="$1"
    
    echo "$input" | while IFS= read -r line; do
        # Skip empty lines
        [[ -z "$line" ]] && continue
        
        # Skip progress bars
        [[ "$line" == *"Writing at"* ]] && continue
        [[ "$line" == *"==="* ]] && continue
        
        # Skip long compiler commands
        if [[ "$line" == *"/riscv32-esp-elf-g++"* ]] || [[ "$line" == *"/xtensa-esp32-elf-g++"* ]]; then
            src=$(echo "$line" | grep -oP '\b\w+\.cpp(?=\s+-o|\s*$)' | head -1)
            [[ -z "$src" ]] && src=$(echo "$line" | grep -oP '\w+\.cpp\.o' | sed 's/\.o$//')
            if [[ -n "$src" && "$src" != "ctags_target_for_gcc_minus_e.cpp" ]]; then
                echo -e "${GRAY}  Compiling: ${NC}${src}"
            fi
            continue
        fi
        
        # Skip env bash commands
        if [[ "$line" == *"/usr/bin/env bash"* ]]; then
            continue
        fi
        
        # Skip esptool commands (show summary instead)
        if [[ "$line" == *"esptool"* ]]; then
            if [[ "$line" == *"Creating ESP32"*"image"* ]]; then
                echo -e "${GREEN}✓ Creating firmware image${NC}"
            elif [[ "$line" == *"Successfully created"* ]]; then
                continue
            fi
            continue
        fi
        
        # Skip arduino-ctags
        if [[ "$line" == *"arduino-ctags"* ]]; then
            continue
        fi
        
        # Show library resolution nicely
        if [[ "$line" == *"Alternatives for"* ]]; then
            lib=$(echo "$line" | sed 's/Alternatives for \(.*\)/\1/')
            echo -e "${GRAY}  Finding: ${lib}${NC}"
            continue
        fi
        
        if [[ "$line" == *"ResolveLibrary"* ]] || [[ "$line" == *"candidates:"* ]]; then
            continue
        fi
        
        # Show "Using cached" as cache hit
        if [[ "$line" == *"Using cached"* ]]; then
            continue
        fi
        
        if [[ "$line" == *"Using previously compiled"* ]]; then
            continue
        fi
        
        # Show "Using library" nicely
        if [[ "$line" == *"Using library"* ]]; then
            lib=$(echo "$line" | sed 's/Using library \([^ ]*\).*/\1/')
            echo -e "${GRAY}  Library: ${lib}${NC}"
            continue
        fi
        
        # Show warnings in yellow
        if [[ "$line" == *"Warning"* ]] || [[ "$line" == *"warning:"* ]]; then
            echo -e "${YELLOW}⚠ ${line}${NC}"
            continue
        fi
        
        # Show errors in red
        if [[ "$line" == *"Error"* ]] || [[ "$line" == *"error:"* ]]; then
            echo -e "${RED}✗ ${line}${NC}"
            continue
        fi
        
        # Show creating bootloader
        if [[ "$line" == *"Merged"*"ELF sections"* ]]; then
            continue
        fi
        
        # Show memory usage
        if [[ "$line" == *"Sketch uses"* ]]; then
            pct=$(echo "$line" | grep -oP '\d+(?=%)')
            echo -e "\n${CYAN}${BOLD}📊 Program Space:${NC}"
            if [[ -n "$pct" ]]; then
                if (( pct > 90 )); then
                    echo -e "${RED}${line}${NC}"
                elif (( pct > 75 )); then
                    echo -e "${YELLOW}${line}${NC}"
                else
                    echo -e "${GREEN}${line}${NC}"
                fi
            else
                echo "$line"
            fi
            continue
        fi
        
        if [[ "$line" == *"Global variables use"* ]]; then
            echo -e "${CYAN}${BOLD}📦 RAM Usage:${NC}"
            echo -e "${GREEN}${line}${NC}"
            continue
        fi
        
        # Show phases
        if [[ "$line" == *"Detecting libraries"* ]]; then
            echo -e "${GRAY}  Detecting libraries...${NC}"
            continue
        fi
        
        if [[ "$line" == *"Generating function prototypes"* ]]; then
            echo -e "${GRAY}  Generating prototypes...${NC}"
            continue
        fi
        
        if [[ "$line" == *"Compiling sketch"* ]]; then
            echo -e "\n${BOLD}📦 Compiling sketch...${NC}"
            continue
        fi
        
        if [[ "$line" == *"Compiling libraries"* ]]; then
            continue
        fi
        
        if [[ "$line" == *"Compiling library"* ]]; then
            continue
        fi
        
        if [[ "$line" == *"Compiling core"* ]]; then
            echo -e "${GRAY}  Compiling core...${NC}"
            continue
        fi
        
        if [[ "$line" == *"Linking everything"* ]]; then
            echo -e "${GRAY}  Linking...${NC}"
            continue
        fi
        
        if [[ "$line" == *"Using precompiled core"* ]]; then
            continue
        fi
        
        # Show board/core info
        if [[ "$line" == *"Using board"* ]] || [[ "$line" == *"Using core"* ]]; then
            continue
        fi
        
        # Skip very long linker commands
        if [[ ${#line} -gt 300 ]]; then
            continue
        fi
    done
}

# Parse board options for FQBN
get_fqbn() {
    local board="$1"
    case "$board" in
        "esp32"|"esp32_devkit")
            echo "esp32:esp32:esp32:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
            ;;
        "lolin_c3_mini"|"c3")
            echo "esp32:esp32:lolin_c3_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=160,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
            ;;
        "esp32c3"|"c3_devkit")
            echo "esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=default,CPUFreq=160,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
            ;;
        "esp32s2"|"s2")
            echo "esp32:esp32:esp32s2:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
            ;;
        "esp32s3"|"s3")
            echo "esp32:esp32:esp32s3:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
            ;;
        "lolin_s2_mini"|"s2_mini")
            echo "esp32:esp32:lolin_s2_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
            ;;
        "lolin_s3_mini"|"s3_mini")
            echo "esp32:esp32:lolin_s3_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
            ;;
        *)
            # Check if it's a custom FQBN
            if [[ "$board" == *"esp32:esp32:"* ]]; then
                echo "$board"
            else
                echo "esp32:esp32:$board:CDCOnBoot=default,PartitionScheme=default,CPUFreq=160,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
            fi
            ;;
    esac
}

# Check parameters
if [ $# -lt 1 ]; then
    echo "Usage: $0 [board] [target] <project.ino>"
    echo ""
    echo "Compile only:              $0 project.ino"
    echo "With board:                $0 lolin_c3_mini project.ino"
    echo "Compile + OTA:             $0 lolin_c3_mini 192.168.1.73 project.ino"
    echo "Compile + USB:             $0 lolin_c3_mini /dev/ttyACM0 project.ino"
    echo ""
    echo "Available boards: esp32, lolin_c3_mini, esp32c3, esp32s2, esp32s3, lolin_s2_mini, lolin_s3_mini"
    echo "Or use full FQBN: esp32:esp32:esp32c3:..."
    exit 1
fi

# Parse arguments
TARGET=""
BOARD="$DEFAULT_BOARD"

for arg in "$@"; do
    if [[ "$arg" == *.ino ]]; then
        INO_FILE="$arg"
    elif [[ "$arg" == /dev/* ]]; then
        TARGET="$arg"
    elif [[ "$arg" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        TARGET="$arg"
    elif [[ "$arg" == *"esp32:"* ]] || [[ "$arg" != *.ino ]] && [[ "$arg" != /dev/* ]] && [[ ! "$arg" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        BOARD="$arg"
    fi
done

if [ -z "$INO_FILE" ]; then
    echo -e "${RED}Error: Missing .ino file parameter${NC}"
    exit 1
fi

# Check if ino file exists
if [ ! -f "$INO_FILE" ]; then
    echo -e "${RED}Error: INO file not found: $INO_FILE${NC}"
    exit 1
fi

# Get absolute path
INO_FILE="$(realpath "$INO_FILE")"

# Generate consistent build path from project path + board
PROJECT_HASH=$(echo "$INO_FILE:$BOARD" | md5sum | cut -c1-8)
BUILD_PATH="/tmp/arduino_build_${PROJECT_HASH}"
CACHE_PATH="/tmp/arduino_cache_${PROJECT_HASH}"

# Extract project name
PROJECT_NAME=$(basename "$INO_FILE" .ino)

# Get FQBN
FQBN=$(get_fqbn "$BOARD")

echo -e "${BOLD}${BLUE}=== ESP32 Compile ===${NC}"
echo -e "  ${CYAN}Project:${NC} $PROJECT_NAME"
echo -e "  ${CYAN}Board:${NC}   $BOARD"
echo -e "  ${CYAN}File:${NC}    $INO_FILE"
[[ -n "$TARGET" ]] && echo -e "  ${CYAN}Target:${NC}  $TARGET"
echo ""

# Create build directory
mkdir -p "$BUILD_PATH"
mkdir -p "$CACHE_PATH"

# Build common arguments
COMMON_ARGS="$HARDWARE_PATHS $TOOLS_PATHS $LIBRARIES -fqbn=$FQBN -ide-version=10819 -build-path $BUILD_PATH -warnings=none -build-cache $CACHE_PATH -prefs=build.warn_data_percentage=75"

# Tool paths - ESP32 3.3.6
TOOL_PREFS="-prefs=runtime.tools.esp32s2-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32s2-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32s2-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32s2-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32h2-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32h2-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32h2-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32h2-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32c6-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32c6-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32c6-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32c6-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp-x32.path=/home/ryan/.arduino15/packages/esp32/tools/esp-x32/2511"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp-x32-2511.path=/home/ryan/.arduino15/packages/esp32/tools/esp-x32/2511"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.riscv32-esp-elf-gdb.path=/home/ryan/.arduino15/packages/esp32/tools/riscv32-esp-elf-gdb/16.3_20250913"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.riscv32-esp-elf-gdb-16.3_20250913.path=/home/ryan/.arduino15/packages/esp32/tools/riscv32-esp-elf-gdb/16.3_20250913"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.dfu-util.path=/home/ryan/.arduino15/packages/arduino/tools/dfu-util/0.11.0-arduino5"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.dfu-util-0.11.0-arduino5.path=/home/ryan/.arduino15/packages/arduino/tools/dfu-util/0.11.0-arduino5"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.mklittlefs.path=/home/ryan/.arduino15/packages/esp32/tools/mklittlefs/4.0.2-db0513a"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.mklittlefs-4.0.2-db0513a.path=/home/ryan/.arduino15/packages/esp32/tools/mklittlefs/4.0.2-db0513a"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32s3-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32s3-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32s3-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32s3-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32c3-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32c3-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32c3-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32c3-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32p4_es-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32p4_es-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32p4_es-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32p4_es-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esptool_py.path=/home/ryan/.arduino15/packages/esp32/tools/esptool_py/5.1.0"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esptool_py-5.1.0.path=/home/ryan/.arduino15/packages/esp32/tools/esptool_py/5.1.0"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.openocd-esp32.path=/home/ryan/.arduino15/packages/esp32/tools/openocd-esp32/v0.12.0-esp32-20251215"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.openocd-esp32-v0.12.0-esp32-20251215.path=/home/ryan/.arduino15/packages/esp32/tools/openocd-esp32/v0.12.0-esp32-20251215"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.mkspiffs.path=/home/ryan/.arduino15/packages/esp32/tools/mkspiffs/0.2.3"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.mkspiffs-0.2.3.path=/home/ryan/.arduino15/packages/esp32/tools/mkspiffs/0.2.3"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32p4-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32p4-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32p4-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32p4-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32c5-libs.path=/home/ryan/.arduino15/packages/esp32/tools/esp32c5-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp32c5-libs-$ESP32_VERSION.path=/home/ryan/.arduino15/packages/esp32/tools/esp32c5-libs/$ESP32_VERSION"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.xtensa-esp-elf-gdb.path=/home/ryan/.arduino15/packages/esp32/tools/xtensa-esp-elf-gdb/16.3_20250913"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.xtensa-esp-elf-gdb-16.3_20250913.path=/home/ryan/.arduino15/packages/esp32/tools/xtensa-esp-elf-gdb/16.3_20250913"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp-rv32.path=/home/ryan/.arduino15/packages/esp32/tools/esp-rv32/2511"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.esp-rv32-2511.path=/home/ryan/.arduino15/packages/esp32/tools/esp-rv32/2511"

FULL_ARGS="$COMMON_ARGS $TOOL_PREFS -verbose $INO_FILE"

# Run dump-prefs (silent)
$ARDUINO_BUILDER -dump-prefs -logger=machine $FULL_ARGS >/dev/null 2>&1

# Run compile with formatted output
echo -e "${BOLD}--- Compiling ---${NC}"
OUTPUT=$($ARDUINO_BUILDER -compile -logger=machine $FULL_ARGS 2>&1)
format_output "$OUTPUT"

# Check for success
if echo "$OUTPUT" | grep -q "Successfully created ESP32"; then
    # Find bin file
    BIN_FILE=$(ls "$BUILD_PATH"/*.ino.bin 2>/dev/null | head -1)
    MERGED_BIN=$(ls "$BUILD_PATH"/*.merged.bin 2>/dev/null | head -1)
    
    if [ -n "$BIN_FILE" ]; then
        BIN_SIZE=$(ls -lh "$BIN_FILE" 2>/dev/null | awk '{print $5}')
        
        echo ""
        echo -e "${BOLD}${GREEN}=== Compile Success ===${NC}"
        echo -e "  ${CYAN}BIN:${NC}    $BIN_FILE (${BIN_SIZE})"
        [ -n "$MERGED_BIN" ] && echo -e "  ${CYAN}Merged:${NC} $MERGED_BIN"
        
        # Flash if target provided
        if [ -n "$TARGET" ]; then
            echo ""
            echo -e "${BOLD}${BLUE}=== Flashing to $TARGET ===${NC}"
            $ESPFLASH_SCRIPT "$TARGET" "$BUILD_PATH"
        fi
    else
        echo ""
        echo -e "${BOLD}${GREEN}=== Compile Success ===${NC}"
        echo -e "  ${CYAN}Build:${NC} $BUILD_PATH"
    fi
else
    echo ""
    echo -e "${BOLD}${RED}=== Compile Failed ===${NC}"
    # Show last 20 lines of output for debugging
    echo "$OUTPUT" | tail -20
    exit 1
fi
