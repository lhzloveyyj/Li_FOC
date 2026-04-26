#!/bin/bash
# OpenOCD flash script for AT32F403ACCT7 + CMSIS-DAP

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build/Debug"
ELF_FILE="$BUILD_DIR/AT32F403ACCT7_WorkBench.elf"
OPENOCD_CFG="$SCRIPT_DIR/at32f403acct7-cmsis-dap.cfg"

if [ ! -f "$ELF_FILE" ]; then
    echo "ERROR: ELF file not found at $ELF_FILE"
    echo "Please build the project first."
    exit 1
fi

echo "Flashing $ELF_FILE via OpenOCD (CMSIS-DAP)..."
openocd -f "$OPENOCD_CFG" -c "program ${ELF_FILE} verify reset exit"
