#!/usr/bin/env bash
# Pull arm-poses.csv back from the machine running the bridge into this repo.
#
# WHY THIS EXISTS. The bridge writes poses to the file sitting next to it, and on
# the Pi that is a DEPLOY COPY (~/arm/console/), not a git working tree. Without
# this step a pose taught at the bench lives on one machine and is invisible to
# git, to review, and to anyone else working on the arm - which is the same
# failure joint-limits.csv already names: "a lock that lives only in a Downloads
# folder is invisible to anything that edits this file". Teaching a pose and
# losing it is worse than not having the feature, because you believe it is saved.
#
# Run it after a teach session, look at the diff, then commit.
#
#   Software/arm-console/sync-poses.sh            # from the ssh host "arm"
#   Software/arm-console/sync-poses.sh other-host
#
# The reverse direction (repo -> bench) is a plain scp and is deliberately NOT
# automated here: overwriting the bench copy would destroy poses taught since the
# last sync, and this file is append-only history by design.
set -euo pipefail

HOST="${1:-arm}"
REMOTE="${REMOTE:-\$HOME/arm/console/arm-poses.csv}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL="$HERE/arm-poses.csv"

tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT

echo "fetching arm-poses.csv from $HOST ..."
# shellcheck disable=SC2029  # $REMOTE is expanded on the far side on purpose
ssh "$HOST" "cat $REMOTE" > "$tmp"

if [ ! -s "$tmp" ]; then
  echo "REFUSING: the bench copy is empty or unreadable. Nothing changed." >&2
  exit 1
fi

# A shrinking pose file means rows were lost, not added. This file is append-only
# history; refuse rather than let a bad sync quietly delete taught poses.
bench_rows=$(grep -vc '^\s*#' "$tmp" || true)
local_rows=$(grep -vc '^\s*#' "$LOCAL" 2>/dev/null || echo 0)
if [ "$bench_rows" -lt "$local_rows" ]; then
  echo "REFUSING: bench copy has $bench_rows data rows, repo has $local_rows." >&2
  echo "Rows would be LOST. Diff them by hand:  diff <(ssh $HOST cat $REMOTE) $LOCAL" >&2
  exit 1
fi

if cmp -s "$tmp" "$LOCAL" 2>/dev/null; then
  echo "already in sync - no change."
  exit 0
fi

cp "$tmp" "$LOCAL"
echo "updated $LOCAL"
echo
git -C "$HERE" --no-pager diff --stat -- "$LOCAL" || true
echo
echo "Review the diff, then commit. Poses are append-only: an older row stays true"
echo "as of its date, so expect additions and not edits."
