#!/bin/bash
# Build script for K3-klipper-mcu firmware
# Kobra S1 with GD32F303 (STM32F103 compatible)

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# OTA payload limit for [mcu_ota mcu].
# Can be overridden with: FW_MAX_SIZE=...
FW_MAX_SIZE="${FW_MAX_SIZE:-40960}"

# Parse arguments
CLEAN_BUILD=0
OPEN_SOURCE_BUILD=0
for arg in "$@"; do
    case "$arg" in
        --clean|-c)
            CLEAN_BUILD=1
            ;;
        --open-source|--open_source|open_source)
            OPEN_SOURCE_BUILD=1
            ;;
        *)
            echo -e "${RED}Error: unknown argument '$arg'${NC}"
            echo "Usage: ./build.sh [--clean] [--open-source]"
            exit 1
            ;;
    esac
done

echo -e "${GREEN}K3-klipper-mcu Build Script${NC}"
echo "================================"
if [ $OPEN_SOURCE_BUILD -eq 1 ]; then
    echo -e "${YELLOW}Build mode: open_source${NC}"
else
    echo -e "${YELLOW}Build mode: stock-compatible${NC}"
fi

# Select toolchain (Ubuntu package)
TOOLCHAIN_BIN="/usr/bin"

if [ ! -x "$TOOLCHAIN_BIN/arm-none-eabi-gcc" ]; then
    echo -e "${RED}Error: arm-none-eabi-gcc not found at $TOOLCHAIN_BIN${NC}"
    echo "Please run ./setup-ubuntu.sh to install the Ubuntu ARM toolchain package."
    exit 1
fi

# Add selected toolchain to PATH
export PATH="$TOOLCHAIN_BIN:$PATH"

# Use simple version format (just the tag, no timestamp/hostname)
export KLIPPER_VERSION_SIMPLE=1

# Verify compiler
echo -e "${YELLOW}Checking compiler version...${NC}"
arm-none-eabi-gcc --version | head -n 1

# Check if .config exists
if [ ! -f .config ]; then
    echo -e "${RED}Error: .config file not found${NC}"
    echo "Please run 'make menuconfig' to configure the build first."
    exit 1
fi

# Extra compiler flags for variant selection
MAKE_ARGS=()

# Clean build if requested
if [ $CLEAN_BUILD -eq 1 ]; then
    echo -e "${YELLOW}Cleaning previous build...${NC}"
    make "${MAKE_ARGS[@]}" clean
    echo -e "${YELLOW}Regenerating config from Kconfig defaults...${NC}"
    make "${MAKE_ARGS[@]}" olddefconfig
fi

# Sanity-check config for Kobra S1 OTA firmware.
# Set SKIP_CONFIG_SANITY=1 to bypass (not recommended).
if [ "${SKIP_CONFIG_SANITY:-0}" != "1" ]; then
    echo -e "${YELLOW}Running config sanity checks...${NC}"

    require_config_line() {
        local line="$1"
        local what="$2"
        if ! grep -q "^${line}$" .config; then
            echo -e "${RED}Error: invalid .config (${what})${NC}"
            echo "Expected: ${line}"
            echo -e "${YELLOW}Hint: stale .config may contain wrong MCU/OTA settings.${NC}"
            echo "Run: rm -f .config .config.old && make menuconfig"
            exit 1
        fi
    }

    require_config_line "CONFIG_MACH_STM32=y" "MCU architecture"
    require_config_line "CONFIG_MACH_STM32F103=y" "MCU model"
    require_config_line "CONFIG_MCU=\"stm32f103xe\"" "MCU part number"
    require_config_line "CONFIG_FLASH_APP1_ADDRESS=0x08008000" "APP1 flash address"
    require_config_line "CONFIG_FLASH_APP2_ADDRESS=0x08012000" "APP2 flash address"
    require_config_line "CONFIG_FLASH_FACTORY_ADDRESS=0x0801C000" "factory flash address"
    require_config_line "CONFIG_FLASH_OTA_DATA_ADDRESS=0x0801E000" "OTA metadata flash address"

    echo -e "${GREEN}✓ Config sanity checks passed${NC}"
fi

# Create board links
echo -e "${YELLOW}Creating board links...${NC}"
# Clean out directory if it has issues
if [ -d out/board ] && [ ! -L out/board ]; then
    rm -rf out/board
fi
make "${MAKE_ARGS[@]}" create-board-link

# Build firmware
echo -e "${YELLOW}Building firmware...${NC}"
make "${MAKE_ARGS[@]}" -j$(nproc)

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓ Build completed successfully!${NC}"

    echo -e "${YELLOW}Verifying OTA protocol symbols in firmware dictionary...${NC}"
    if ! python3 - <<'PY'
import json
import sys

required_cmds = {
    "config_ota",
    "ota_start",
    "ota_erase",
    "ota_transfer_response",
    "query_ota_local_info",
}
required_resps = {
    "ota_status",
    "ota_transfer",
    "ota_local_info",
}

with open("out/klipper.dict", "rb") as f:
    d = json.load(f)

cmds = {k.split()[0] for k in d.get("commands", {})}
resps = {k.split()[0] for k in d.get("responses", {})}

missing = [f"command:{n}" for n in sorted(required_cmds - cmds)]
missing += [f"response:{n}" for n in sorted(required_resps - resps)]

if missing:
    print("Missing OTA protocol entries: " + ", ".join(missing))
    sys.exit(1)
PY
    then
        echo -e "${RED}✗ OTA protocol sanity check failed.${NC}"
        echo -e "${YELLOW}Your build does not contain required OTA commands/responses.${NC}"
        echo -e "${YELLOW}Hint: regenerate config with:${NC} rm -f .config .config.old && make menuconfig"
        exit 1
    fi
    echo -e "${GREEN}✓ OTA protocol symbols verified${NC}"

    echo -e "${YELLOW}Checking message-id encoding compatibility...${NC}"
    if ! python3 - <<'PY'
import pathlib
import re
import sys

compile_time_request = pathlib.Path("out/compile_time_request.c")
if not compile_time_request.exists():
    print("Missing generated file: out/compile_time_request.c")
    sys.exit(1)

text = compile_time_request.read_text()
encoded_ids = [int(m.group(1)) for m in re.finditer(r"\.encoded_msgid=(\d+)", text)]
if not encoded_ids:
    print("Could not find any encoded message ids in out/compile_time_request.c")
    sys.exit(1)

max_encoded_msgid = max(encoded_ids)
if max_encoded_msgid >= 128:
    print(
        "Firmware dictionary exceeded one-byte message ids: "
        f"max_encoded_msgid={max_encoded_msgid}"
    )
    print("This would require two-byte msgid encoding on the wire.")
    sys.exit(1)

print(f"  max_encoded_msgid={max_encoded_msgid} (one-byte msgids only)")
PY
    then
        echo -e "${RED}✗ Message-id compatibility check failed.${NC}"
        echo -e "${YELLOW}This build would require two-byte wire msgids and may break older host implementations.${NC}"
        echo -e "${YELLOW}Reduce enabled commands/responses before flashing this firmware.${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Message-id compatibility verified${NC}"
    
    # Append CRC32 to binary for OTA compatibility
    # Rebuild .bin from .elf to ensure clean state, then append CRC
    echo -e "${YELLOW}Appending CRC32 for OTA...${NC}"
    arm-none-eabi-objcopy -O binary out/klipper.elf out/klipper.bin
    python3 -c "
import zlib
with open('out/klipper.bin', 'rb') as f:
    data = f.read()
crc = zlib.crc32(data) & 0xFFFFFFFF
with open('out/klipper.bin', 'ab') as f:
    f.write(f'{crc:08X}'.encode('ascii'))
    f.write(b'\n')
print(f'  CRC32: 0x{crc:08X}')
"

    # OTA package includes a 9-byte ASCII trailer (8 hex CRC + newline).
    # fw_max_size applies to the payload bytes only.
    BIN_SIZE=$(stat -c%s out/klipper.bin)
    if [ "$BIN_SIZE" -lt 9 ]; then
        echo -e "${RED}✗ Invalid firmware size: ${BIN_SIZE} bytes (expected at least 9-byte CRC trailer).${NC}"
        exit 1
    fi
    PAYLOAD_SIZE=$((BIN_SIZE - 9))
    if [ "$PAYLOAD_SIZE" -gt "$FW_MAX_SIZE" ]; then
        echo -e "${RED}✗ Firmware too large for MCU OTA.${NC}"
        echo -e "${RED}  payload_size=${PAYLOAD_SIZE} bytes, fw_max_size=${FW_MAX_SIZE} bytes${NC}"
        echo -e "${YELLOW}  total_bin_size=${BIN_SIZE} bytes (includes 9-byte CRC trailer)${NC}"
        echo -e "${YELLOW}  Reduce firmware size or increase fw_max_size in printer config (if safe).${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Firmware payload size OK: ${PAYLOAD_SIZE}/${FW_MAX_SIZE} bytes${NC}"

    echo -e "${YELLOW}Embedded MCU version...${NC}"
    python3 - <<'PY'
import json
with open('out/klipper.dict', 'rb') as f:
    print(f"  Version: {json.load(f)['version']}")
PY
    
    # Get version from Kconfig default or .config
    VERSION=$(grep -E "^CONFIG_FIRMWARE_VERSION=" .config 2>/dev/null | cut -d'"' -f2 || echo "v1.3.11")
    VERSION=${VERSION#v}  # Remove leading 'v' if present
    
    # Get build date (without time)
    BUILD_DATE=$(date +%Y%m%d)
    
    # Create output filename
    if [ $OPEN_SOURCE_BUILD -eq 1 ]; then
        OUTPUT_NAME="firmware_v${VERSION}_OS_${BUILD_DATE}"
    else
        OUTPUT_NAME="firmware_v${VERSION}_${BUILD_DATE}"
    fi
    
    # Copy output files with version/datetime
    cp out/klipper.elf "out/${OUTPUT_NAME}.elf"
    cp out/klipper.bin "out/${OUTPUT_NAME}.bin"
    
    echo ""
    echo "Output files:"
    ls -lh "out/${OUTPUT_NAME}.elf" "out/${OUTPUT_NAME}.bin"
    echo ""
    echo "Flash the .bin file to your Kobra S1 MCU"
else
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi
