#!/usr/bin/env bash
# Backs up the dARM control scripts to a timestamped folder on the
# Pi's own SD card. Safe to run any time - read-only with respect to
# the robot, just copies files.
set -euo pipefail

SRC_DIR="$HOME/dARM/odrive_tools"
BACKUP_ROOT="$HOME/dARM/backups"
STAMP=$(date +%Y-%m-%d_%H%M%S)
DEST="$BACKUP_ROOT/backup_$STAMP"

mkdir -p "$DEST"

rsync -a \
  --exclude '.venv' \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude 'backups' \
  --exclude 'gamecontroller_test_log.txt' \
  "$SRC_DIR"/ "$DEST"/

echo "Backed up $SRC_DIR to $DEST"
echo ""
echo "=== Existing backups ==="
ls -1 "$BACKUP_ROOT"
