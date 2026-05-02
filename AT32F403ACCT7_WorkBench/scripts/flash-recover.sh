#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/.."

ELF="${1:-build/Debug/AT32F403ACCT7_WorkBench.elf}"

openocd \
  -f openocd/at32f403acct7-cmsis-dap-recover.cfg \
  -c "init; reset halt; program ${ELF} verify; reset run; shutdown"
