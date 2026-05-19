#!/bin/bash
set -euo pipefail

LOG_DIR="$HOME/arv_v1_logs"
RETENTION_HOURS=24

[ -d "$LOG_DIR" ] || exit 0

# Protect the currently active session (target of 'latest' symlink)
ACTIVE=""
if [ -L "$LOG_DIR/latest" ]; then
  ACTIVE=$(readlink -f "$LOG_DIR/latest" 2>/dev/null || true)
fi

# Delete session directories older than retention period
find "$LOG_DIR" -maxdepth 1 -mindepth 1 -type d -mmin +$((RETENTION_HOURS * 60)) | while read -r dir; do
  resolved=$(readlink -f "$dir")
  [ "$resolved" = "$ACTIVE" ] && continue
  echo "Removing old session: $dir"
  rm -rf "$dir"
done

# Rotate systemd.log if > 1MB (keep tail 512KB)
SYSLOG="$LOG_DIR/systemd.log"
if [ -f "$SYSLOG" ] && [ "$(stat -c%s "$SYSLOG" 2>/dev/null || echo 0)" -gt 1048576 ]; then
  tail -c 524288 "$SYSLOG" > "${SYSLOG}.tmp" && mv "${SYSLOG}.tmp" "$SYSLOG"
  echo "Rotated systemd.log"
fi
