#!/usr/bin/env bash
# macOS launcher for the FactoryLM arm console.
# Double-click this file in Finder, or run it from Terminal.
# Keep the Terminal window open while the console is in use.

set -u

ARM_ROOT="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
ARM_BRIDGE="$ARM_ROOT/Software/arm-console/arm-bridge.py"
ARM_URL="http://127.0.0.1:8770/"
ARM_PYTHON="${ARM_PYTHON:-python3}"

printf '\n%s\n' '============================================================'
printf '%s\n' ' FACTORYLM ARM GUI — macOS'
printf '%s\n\n' '============================================================'

if [ ! -f "$ARM_BRIDGE" ]; then
  printf '%s\n' "PROBLEM: the bridge program is missing: $ARM_BRIDGE"
  read -r -p 'Press Return to close... ' _
  exit 1
fi

if ! command -v "$ARM_PYTHON" >/dev/null 2>&1; then
  printf '%s\n' 'PROBLEM: Python 3 was not found.'
  printf '%s\n' 'Install Python 3, or use the no-install route:'
  printf '%s\n' 'open Software/arm-console/arm-console.html in Chrome or Edge.'
  read -r -p 'Press Return to close... ' _
  exit 1
fi

if ! "$ARM_PYTHON" -c 'import serial' >/dev/null 2>&1; then
  printf '%s\n' 'PROBLEM: pyserial is not installed for this Python.'
  printf '%s\n' "Install it with: $ARM_PYTHON -m pip install --user pyserial"
  printf '%s\n' 'Then double-click this launcher again.'
  printf '%s\n' 'Alternatively, open arm-console.html directly in Chrome or Edge.'
  read -r -p 'Press Return to close... ' _
  exit 1
fi

if ! command -v open >/dev/null 2>&1; then
  printf '%s\n' 'PROBLEM: macOS open command is unavailable.'
  read -r -p 'Press Return to close... ' _
  exit 1
fi

# Wait until the bridge accepts connections, then open the canonical URL. The
# bridge itself prints a one-time token and redirects the bare URL to it.
(
  if command -v curl >/dev/null 2>&1; then
    attempt=0
    while [ "$attempt" -lt 60 ]; do
      if curl -fsS --max-time 1 "$ARM_URL" >/dev/null 2>&1; then break; fi
      attempt=$((attempt + 1))
      sleep 0.1
    done
  else
    sleep 2
  fi
  open "$ARM_URL"
) &

printf '%s\n' 'Starting the local bridge. Your default browser will open shortly.'
printf '%s\n\n' 'Keep this Terminal window open. Press Ctrl-C here to stop the bridge.'
exec "$ARM_PYTHON" "$ARM_BRIDGE"
