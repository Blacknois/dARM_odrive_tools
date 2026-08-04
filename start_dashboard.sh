#!/bin/bash
# Starts pos_stream_server.py (feeds the Mac-side visualizer) if it isn't
# already running, then hands off to the existing preflight startdARM
# wrapper exactly as if you'd typed startdARM yourself.
cd ~/dARM/odrive_tools || exit 1

if pgrep -f pos_stream_server.py > /dev/null; then
  echo "pos_stream_server.py already running."
else
  echo "Starting pos_stream_server.py in the background..."
  source .venv/bin/activate
  nohup python3 pos_stream_server.py > pos_stream.log 2>&1 &
  disown
  sleep 1
  echo "pos_stream_server.py started (log: pos_stream.log)."
fi

echo ""
bash ~/dARM/odrive_tools/preflight_startdARM.sh
