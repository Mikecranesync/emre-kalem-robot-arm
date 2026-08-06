#!/usr/bin/env bash
# Runs the arm console's ?selftest=1 maths in headless Chrome and gates on the
# sentinel it renders.
#
# Passes only if the rendered <pre id="selftest"> block contains SELFTEST_PASS.
#
# Two traps this deliberately avoids:
#
#   1. Chrome's EXIT CODE is not the gate. A page that throws before rendering
#      anything still exits 0 -- exactly the failure this guards against.
#
#   2. Grepping the WHOLE DOM is not the gate either. --dump-dom includes the
#      <script> source, and that source contains the literal strings
#      "SELFTEST_PASS" and "SELFTEST_FAIL". A whole-DOM grep therefore matches
#      the code rather than the result, and reports a failure on a passing run.
#      So the <pre> block is extracted FIRST and only its contents are judged.
#
# No board, no bridge, no servo -- this runs against file://.
set -u

SCRIPT_DIR="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(CDPATH='' cd -- "$SCRIPT_DIR/../.." && pwd)"
ARM_PYTHON="${ARM_PYTHON:-python3}"
PAGE_FILE="${PAGE_FILE:-$REPO_ROOT/Software/arm-console/arm-console.html}"

if ! command -v "$ARM_PYTHON" >/dev/null 2>&1; then
  echo "HARNESS ERROR: Python not found: $ARM_PYTHON"
  echo "Set ARM_PYTHON=/path/to/python3 and re-run."
  exit 2
fi

if [ -z "${CHROME:-}" ]; then
  case "$(uname -s 2>/dev/null || true)" in
    Darwin*)
      for candidate in \
        "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
        "/Applications/Chromium.app/Contents/MacOS/Chromium"; do
        if [ -x "$candidate" ]; then CHROME="$candidate"; break; fi
      done
      ;;
    *)
      # The directory is "Google/Chrome", NOT "Google Chrome". Getting that
      # wrong silently costs the whole Windows path: nothing below matches
      # either (Git Bash has no google-chrome on PATH), so the harness exits 2
      # on a machine where Chrome is installed and working. Both per-machine
      # install locations plus the per-user one, because Chrome uses all three.
      for candidate in \
        "/c/Program Files/Google/Chrome/Application/chrome.exe" \
        "/c/Program Files (x86)/Google/Chrome/Application/chrome.exe" \
        "${LOCALAPPDATA:-/c/Users/$USERNAME/AppData/Local}/Google/Chrome/Application/chrome.exe"; do
        if [ -x "$candidate" ]; then CHROME="$candidate"; break; fi
      done
      ;;
  esac
fi

if [ -z "${CHROME:-}" ]; then
  for candidate in google-chrome chromium chromium-browser; do
    if command -v "$candidate" >/dev/null 2>&1; then CHROME="$(command -v "$candidate")"; break; fi
  done
fi

PAGE_URL="${PAGE_URL:-}"
if [ -z "$PAGE_URL" ]; then
  PAGE_URL="$("$ARM_PYTHON" - "$PAGE_FILE" <<'PY'
import sys
from pathlib import Path
from urllib.parse import quote

path = Path(sys.argv[1]).resolve().as_posix()
print("file://" + quote(path, safe="/:" ) + "?selftest=1")
PY
)"
fi

if [ -z "${CHROME:-}" ] || [ ! -x "$CHROME" ]; then
  echo "HARNESS ERROR: a headless Chrome/Chromium executable was not found."
  echo "On macOS, install Google Chrome or set CHROME=/path/to/Google\\ Chrome."
  echo "Set CHROME=/path/to/chrome and re-run."
  exit 2
fi

# Syntax-check the inline script first, when node is available. Without this a
# stray character anywhere in 2500 lines reports only as "the page threw before
# rendering", which is true but useless. node names the line.
if command -v node >/dev/null 2>&1; then
  # macOS mktemp -t treats its argument as a prefix and appends a random
  # suffix, which leaves Node with a file named *.js.<random>. Use a full
  # template so the temporary file keeps the .js suffix on every platform.
  TMPJS="$(mktemp "${TMPDIR:-/tmp}/armconsole.XXXXXX.js" 2>/dev/null || true)"
  if [ -z "$TMPJS" ]; then TMPJS="${TMPDIR:-/tmp}/armconsole.$$.js"; fi
  "$ARM_PYTHON" -c "
import io,re,sys
s = io.open(sys.argv[1], encoding='utf-8').read()
b = re.findall(r'<script>(.*?)</script>', s, re.S)
io.open(sys.argv[2], 'w', encoding='utf-8', newline='\n').write(b[-1] if b else '')
" "$PAGE_FILE" "$TMPJS" 2>/dev/null
  if [ -s "$TMPJS" ] && ! node --check "$TMPJS" >/dev/null 2>&1; then
    echo "SELFTEST FAILED: the console's inline script does not parse."
    node --check "$TMPJS" 2>&1 | head -12
    rm -f "$TMPJS"
    exit 1
  fi
  rm -f "$TMPJS"
fi

# --virtual-time-budget makes the dump deterministic: it fires after the page's
# timers settle rather than at an arbitrary moment.
DOM="$("$CHROME" --headless=new --disable-gpu --virtual-time-budget=5000 \
        --dump-dom "$PAGE_URL" 2>/dev/null)"

if [ -z "$DOM" ]; then
  echo "HARNESS ERROR: Chrome produced no DOM at all."
  exit 2
fi

# Extract only the rendered sentinel block.
BLOCK="$(printf '%s' "$DOM" | awk '
  /<pre id="selftest">/ { inblk = 1; sub(/.*<pre id="selftest">/, "") }
  inblk {
    if (match($0, /<\/pre>/)) { print substr($0, 1, RSTART - 1); exit }
    print
  }
')"

if [ -z "$BLOCK" ]; then
  echo "SELFTEST DID NOT RUN — no <pre id=\"selftest\"> in the rendered page."
  echo "The page probably threw before rendering. Open it and check the console:"
  echo "  $PAGE_URL"
  exit 1
fi

case "$BLOCK" in
  SELFTEST_PASS*)
    echo "SELFTEST_PASS"
    exit 0
    ;;
  *)
    echo "SELFTEST FAILED:"
    printf '%s\n' "$BLOCK"
    exit 1
    ;;
esac
