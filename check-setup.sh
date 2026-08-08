#!/usr/bin/env bash
# macOS/Linux setup verifier. Safe: it does not upload firmware or enable motion.

set -u

ARM_ROOT="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
ARM_PYTHON="${ARM_PYTHON:-python3}"
ARM_CLI="${ARDUINO_CLI:-}"
PASS=0
FAIL=0

report() {
  if [ "$2" = yes ]; then
    PASS=$((PASS + 1))
    printf '[ OK   ] %-34s %s\n' "$1" "$3"
  else
    FAIL=$((FAIL + 1))
    printf '[ FAIL ] %-34s %s\n' "$1" "$3"
  fi
}

note() {
  printf '[ NOTE ] %-34s %s\n' "$1" "$2"
}

printf '\n%s\n' '========================================================'
printf '%s\n' ' EMRE KALEM ROBOT ARM — macOS SETUP CHECK'
printf '%s\n\n' '========================================================'

if command -v "$ARM_PYTHON" >/dev/null 2>&1; then
  report 'Python 3' yes "$($ARM_PYTHON --version 2>&1)"
  if "$ARM_PYTHON" -c 'import serial' >/dev/null 2>&1; then
    report 'pyserial' yes 'installed'
  else
    report 'pyserial' no "missing — $ARM_PYTHON -m pip install --user pyserial"
  fi
else
  report 'Python 3' no "not found — set ARM_PYTHON or install Python 3"
fi

if [ -z "$ARM_CLI" ]; then ARM_CLI="$(command -v arduino-cli 2>/dev/null || true)"; fi
if [ -n "$ARM_CLI" ] && [ -x "$ARM_CLI" ]; then
  report 'Arduino CLI' yes "$($ARM_CLI version 2>/dev/null | head -1)"
  if "$ARM_CLI" core list 2>/dev/null | grep -q 'arduino:avr'; then
    report 'Arduino AVR Boards' yes 'installed'
  else
    report 'Arduino AVR Boards' no 'missing — arduino-cli core install arduino:avr'
  fi
else
  note 'Arduino CLI' 'not found (only required to compile/upload firmware)'
fi

for rel in \
  'Software/arm-console/arm-console.html' \
  'Software/arm-console/arm-bridge.py' \
  'Software/arm-console/joint-limits.csv' \
  'Software/factorylm_arm_controller/factorylm_arm_controller.ino'; do
  if [ -f "$ARM_ROOT/$rel" ]; then
    report "$rel" yes 'present'
  else
    report "$rel" no 'MISSING'
  fi
done

if command -v "$ARM_PYTHON" >/dev/null 2>&1; then
  ports="$($ARM_PYTHON - <<'PY'
try:
    from serial.tools import list_ports
except ImportError:
    raise SystemExit(0)
for port in list_ports.comports():
    print("  %s  [%s]" % (port.device, port.description or "serial device"))
PY
)"
  if [ -n "$ports" ]; then
    printf '\nDetected serial devices:\n%s\n' "$ports"
  else
    printf '\nNo serial device detected. Plug in the Arduino with a DATA USB cable before connecting.\n'
  fi
fi

printf '\nPASS: %d    FAIL: %d\n' "$PASS" "$FAIL"
printf '%s\n' 'No firmware was uploaded and no servo command was sent.'
if [ "$FAIL" -eq 0 ]; then exit 0; else exit 1; fi
