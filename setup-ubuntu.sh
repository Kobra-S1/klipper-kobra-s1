#!/bin/bash
# Setup script for K3-klipper-mcu build environment on Ubuntu/Debian
# This script installs all dependencies and ARM GCC toolchain from Ubuntu packages

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}K3-klipper-mcu Build Environment Setup (Ubuntu/Debian)${NC}"
echo "======================================================="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then 
    echo -e "${RED}Error: Do not run this script as root${NC}"
    echo "The script will prompt for sudo password when needed."
    exit 1
fi

# Update package list
echo -e "${YELLOW}Updating package list...${NC}"
sudo apt-get update

# Install system dependencies
echo -e "${YELLOW}Installing system dependencies...${NC}"
sudo apt-get install -y \
    python3 \
    python3-pip \
    build-essential \
    make \
    libncurses-dev \
    git \
    libusb-dev \
    libusb-1.0-0-dev \
    python3-serial \
    gcc-arm-none-eabi \
    binutils-arm-none-eabi

echo -e "${GREEN}✓ System dependencies installed${NC}"
echo ""

SYSTEM_TOOLCHAIN_BIN="/usr/bin/arm-none-eabi-gcc"

# Verify toolchain
echo ""
echo -e "${YELLOW}Verifying toolchain...${NC}"
if [ ! -x "$SYSTEM_TOOLCHAIN_BIN" ]; then
    echo -e "${RED}Error: Toolchain installation failed${NC}"
    exit 1
fi
"$SYSTEM_TOOLCHAIN_BIN" --version | head -n 1
echo -e "${GREEN}✓ Toolchain verified${NC}"

# Setup PATH in bashrc if needed
echo ""
echo -e "${YELLOW}Setting up environment...${NC}"
echo -e "${BLUE}System toolchain installed via apt; no PATH update needed${NC}"

echo ""
echo -e "${GREEN}======================================================="
echo "Setup completed successfully!"
echo -e "=======================================================${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo -e "1. Reload your shell: ${BLUE}source ~/.bashrc${NC}"
echo -e "2. Configure the build: ${BLUE}make menuconfig${NC}"
echo -e "3. Build firmware: ${BLUE}bash build.sh${NC}"
echo ""
echo -e "${YELLOW}Toolchain location:${NC} /usr/bin/arm-none-eabi-gcc"
echo ""
