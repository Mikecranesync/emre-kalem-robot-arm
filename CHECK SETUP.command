#!/usr/bin/env bash
# Finder launcher for the portable setup check.
set -u
ARM_ROOT="$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)"
"$ARM_ROOT/check-setup.sh"
status=$?
printf '\nSetup check finished with status %d.\n' "$status"
read -r -p 'Press Return to close... ' _
exit "$status"
