#!/usr/bin/env sh
set -eu

cd "$(dirname "$0")/.."

openocd \
  -f openocd/at32f403acct7-cmsis-dap.cfg \
  -c "init; reset halt; flash probe 0; flash banks; shutdown"
