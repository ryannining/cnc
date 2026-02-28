#!/bin/bash

# ESP8266 Compile Script
# Usage: espcompile.sh [target] <project.ino>
# Example compile only:  espcompile.sh project.ino
# Example compile + OTA: espcompile.sh 192.168.1.73 project.ino
# Example compile + USB: espcompile.sh /dev/ttyUSB0 project.ino

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
GRAY='\033[0;90m'
BOLD='\033[1m'
NC='\033[0m' # No Color

ARDUINO_BUILDER="/usr/share/arduino/arduino-builder"
HARDWARE_PATHS="-hardware /usr/share/arduino/hardware -hardware /home/ryan/.arduino15/packages"
TOOLS_PATHS="-tools /usr/share/arduino/tools-builder -tools /home/ryan/.arduino15/packages"
LIBRARIES="-libraries /home/ryan/Arduino/libraries"
FQBN="esp8266:esp8266:d1_mini:xtal=160,vt=flash,exception=disabled,stacksmash=disabled,ssl=basic,mmu=4816,non32xfer=fast,eesz=4M3M,ip=lm2n,dbg=Disabled,lvl=None____,wipe=none,baud=921600"

ESPFLASH_SCRIPT="/home/ryan/espflash.sh"

# Function to format compile output
format_output() {
    local input="$1"
    local build_path="$2"

    echo "$input" | while IFS= read -r line; do
        # Skip empty lines and progress
        [[ -z "$line" ]] && continue
        [[ "$line" == *"===info"* ]] && continue

        # Skip long gcc/g++ commands (just show compiling indicator)
        if [[ "$line" == *"/xtensa-lx106-elf-g++"* ]] || [[ "$line" == *"/xtensa-lx106-elf-gcc"* ]]; then
            # Extract source file being compiled
            src=$(echo "$line" | grep -oP '\b\w+\.cpp\.o\b' | sed 's/\.o$//' | head -1)
            if [[ -n "$src" && "$src" != "ctags_target_for_gcc_minus_e.cpp" ]]; then
                echo -e "${GRAY}  Compiling: ${NC}${src}"
            fi
            continue
        fi

        # Skip python tool commands
        if [[ "$line" == *"python3 -I"* ]] || [[ "$line" == *"python3 -X"* ]]; then
            continue
        fi

        # Skip ctags
        if [[ "$line" == *"arduino-ctags"* ]]; then
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

        # Show creating BIN file
        if [[ "$line" == *"Creating BIN file"* ]]; then
            bin_name=$(basename "$(echo "$line" | grep -oP '"[^"]+\.bin"')")
            echo -e "${GREEN}✓ Creating: ${BOLD}${bin_name}${NC}"
            continue
        fi

        # Show memory usage nicely
        if [[ "$line" == *"Variables and constants in RAM"* ]]; then
            echo -e "\n${CYAN}${BOLD}📊 Memory Usage:${NC}"
            continue
        fi

        if [[ "$line" == *"Instruction RAM"* ]]; then
            echo -e "${CYAN}${BOLD}📦 IRAM Usage:${NC}"
            continue
        fi

        if [[ "$line" == *"Code in flash"* ]]; then
            echo -e "${CYAN}${BOLD}💾 Flash Usage:${NC}"
            continue
        fi

        # Show segment table nicely
        if [[ "$line" == *"SEGMENT"* ]] || [[ "$line" =~ ^[║╠╚].* ]]; then
            # Colorize the table
            echo -e "${GRAY}${line}${NC}"
            continue
        fi

        # Show file usage percentages with color based on level
        if [[ "$line" == *"used"*"bytes"* ]]; then
            pct=$(echo "$line" | grep -oP '\d+(?=%)')
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

        # Show info messages
        if [[ "$line" == *"Note:"* ]] || [[ "$line" == *"Assume"* ]] || [[ "$line" == *"default_encoding"* ]]; then
            echo -e "${GRAY}  ${line}${NC}"
            continue
        fi

        # Skip very long linker commands, show summary instead
        if [[ "$line" == *"-Wl,"* ]] && [[ ${#line} -gt 200 ]]; then
            echo -e "${GRAY}  Linking...${NC}"
            continue
        fi
    done
}

# Check parameters
if [ $# -lt 1 ]; then
    echo "Usage: $0 [target] <project.ino>"
    echo ""
    echo "Compile only:   $0 project.ino"
    echo "Compile + OTA:  $0 192.168.1.73 project.ino"
    echo "Compile + USB:  $0 /dev/ttyUSB0 project.ino"
    exit 1
fi

# Determine if target is provided
if [[ "$1" == *.ino ]]; then
    # No target, just compile
    TARGET=""
    INO_FILE="$1"
else
    # Target provided
    TARGET="$1"
    INO_FILE="$2"

    if [ -z "$INO_FILE" ]; then
        echo -e "${RED}Error: Missing .ino file parameter${NC}"
        exit 1
    fi
fi

# Check if ino file exists
if [ ! -f "$INO_FILE" ]; then
    echo -e "${RED}Error: INO file not found: $INO_FILE${NC}"
    exit 1
fi

# Get absolute path
INO_FILE="$(realpath "$INO_FILE")"

# Generate consistent build path from project path
PROJECT_HASH=$(echo "$INO_FILE" | md5sum | cut -c1-8)
BUILD_PATH="/tmp/arduino_build_${PROJECT_HASH}"
CACHE_PATH="/tmp/arduino_cache_${PROJECT_HASH}"

# Extract project name
PROJECT_NAME=$(basename "$INO_FILE" .ino)

echo -e "${BOLD}${BLUE}=== ESP8266 Compile ===${NC}"
echo -e "  ${CYAN}Project:${NC} $PROJECT_NAME"
echo -e "  ${CYAN}File:${NC}    $INO_FILE"
[[ -n "$TARGET" ]] && echo -e "  ${CYAN}Target:${NC}  $TARGET"
echo ""

# Create build directory
mkdir -p "$BUILD_PATH"
mkdir -p "$CACHE_PATH"

# Build common arguments
COMMON_ARGS="$HARDWARE_PATHS $TOOLS_PATHS $LIBRARIES -fqbn=$FQBN -ide-version=10819 -build-path $BUILD_PATH -warnings=none -build-cache $CACHE_PATH -prefs=build.warn_data_percentage=75"

# Tool paths
TOOL_PREFS="-prefs=runtime.tools.mklittlefs.path=/home/ryan/.arduino15/packages/esp8266/tools/mklittlefs/3.1.0-gcc10.3-e5f9fec"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.mklittlefs-3.1.0-gcc10.3-e5f9fec.path=/home/ryan/.arduino15/packages/esp8266/tools/mklittlefs/3.1.0-gcc10.3-e5f9fec"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.python3.path=/home/ryan/.arduino15/packages/esp8266/tools/python3/3.7.2-post1"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.python3-3.7.2-post1.path=/home/ryan/.arduino15/packages/esp8266/tools/python3/3.7.2-post1"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.mkspiffs.path=/home/ryan/.arduino15/packages/esp8266/tools/mkspiffs/3.1.0-gcc10.3-e5f9fec"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.mkspiffs-3.1.0-gcc10.3-e5f9fec.path=/home/ryan/.arduino15/packages/esp8266/tools/mkspiffs/3.1.0-gcc10.3-e5f9fec"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.xtensa-lx106-elf-gcc.path=/home/ryan/.arduino15/packages/esp8266/tools/xtensa-lx106-elf-gcc/3.1.0-gcc10.3-e5f9fec"
TOOL_PREFS="$TOOL_PREFS -prefs=runtime.tools.xtensa-lx106-elf-gcc-3.1.0-gcc10.3-e5f9fec.path=/home/ryan/.arduino15/packages/esp8266/tools/xtensa-lx106-elf-gcc/3.1.0-gcc10.3-e5f9fec"

FULL_ARGS="$COMMON_ARGS $TOOL_PREFS -verbose $INO_FILE"

# Run dump-prefs (silent)
$ARDUINO_BUILDER -dump-prefs -logger=machine $FULL_ARGS >/dev/null 2>&1

# Run compile with formatted output
echo -e "${BOLD}--- Compiling ---${NC}"
OUTPUT=$($ARDUINO_BUILDER -compile -logger=machine $FULL_ARGS 2>&1)
format_output "$OUTPUT" "$BUILD_PATH"

# Check for success
if echo "$OUTPUT" | grep -q "Creating BIN file"; then
    # Extract bin file path
    BIN_FILE=$(echo "$OUTPUT" | grep "Creating BIN file" | sed 's/.*Creating BIN file "\([^"]*\)".*/\1/')
    BIN_SIZE=$(ls -lh "$BIN_FILE" 2>/dev/null | awk '{print $5}')

    echo ""
    echo -e "${BOLD}${GREEN}=== Compile Success ===${NC}"
    echo -e "  ${CYAN}BIN:${NC}  $BIN_FILE"
    echo -e "  ${CYAN}Size:${NC} ${BOLD}${BIN_SIZE}${NC}"

    # Flash if target provided
    if [ -n "$TARGET" ]; then
        echo ""
        echo -e "${BOLD}${BLUE}=== Flashing to $TARGET ===${NC}"
        $ESPFLASH_SCRIPT "$TARGET" "$BIN_FILE"
    fi
else
    echo ""
    echo -e "${BOLD}${RED}=== Compile Failed ===${NC}"
    exit 1
fi
