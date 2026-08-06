"""
pos_stream_server.py

Read-only companion to watch_limits.py: continuously reads live joint
positions for all discovered nodes and serves them as JSON over a plain
local HTTP endpoint, so the darm_visualizer.html page (or anything else
on the local network) can poll for live positions.

- Does NOT arm, disarm, or send any motion command - reads only.
- Uses only the Python standard library (http.server, threading, json)
  plus the project's existing src/configure.py and src/can_utils.py -
  no new pip installs needed.
- Safe to run alongside gamecontroller.py in a separate SSH window,
  same as watch_limits.py.
- Re-discovers nodes periodically (every REDISCOVER_INTERVAL seconds)
  rather than once at startup, so a node that wasn't up yet when this
  server started, or that drops and reconnects (e.g. during a rebuild
  where nodes get power-cycled), gets picked up automatically instead
  of silently never appearing/updating again.
- Survives can0 actually going down (not just bus-off): a CAN-level
  read/discovery error no longer kills the polling thread silently -
  it's caught, the bus connection is reopened, and polling resumes once
  the interface is healthy again. Previously an unhandled
  CanOperationError ("Network is down") would kill the background
  thread while the HTTP server kept running and serving frozen,
  never-updating data with no visible sign anything was wrong.
- Each snapshot includes "_healthy": true/false so a client can tell
  whether the last poll cycle actually succeeded, not just that the
  server process is alive.

Usage (on the Pi, from ~/dARM/odrive_tools/):
    source .venv/bin/activate
    python3 tools/pos_stream_server.py

Then from the visualizer page, enter this Pi's address (e.g.
pidarm:8080 or 192.168.x.x:8080) and click "Connect to live robot".

Find the Pi's local IP if needed with:  hostname -I
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # so this can find src/ when run from tools/
import json
import threading
import time
import can
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from src.configure import load_endpoints, read_config
from src.can_utils import discover_node_ids

MAX_PLAUSIBLE_DELTA = 2.0    # ignore single-poll jumps bigger than this (glitch filter)
POLL_INTERVAL = 0.1          # seconds between CAN reads
REDISCOVER_INTERVAL = 5.0    # seconds between re-running node discovery
BUS_REOPEN_DELAY = 1.0       # seconds to wait before reopening the bus after a CAN error
HTTP_PORT = 8080

latest = {}
latest_lock = threading.Lock()


def open_bus():
    return can.interface.Bus("can0", interface="socketcan")


def poll_loop():
    bus = open_bus()
    node_ids = []
    last_good = {}
    endpoints = load_endpoints()["endpoints"]
    pos_ep = endpoints["axis0.pos_estimate"]
    last_rediscover = 0.0  # force an immediate discovery on first loop pass

    print(f"pos_stream_server: serving http://0.0.0.0:{HTTP_PORT}/positions")
    print(f"pos_stream_server: re-discovering nodes every {REDISCOVER_INTERVAL}s")
    print("Read-only - does not touch arming or motion. Ctrl+C to stop.\n")

    try:
        while True:
            try:
                if time.time() - last_rediscover >= REDISCOVER_INTERVAL:
                    fresh_ids = sorted(discover_node_ids(bus))
                    if fresh_ids != node_ids:
                        print(f"pos_stream_server: node list changed {node_ids} -> {fresh_ids}")
                        node_ids = fresh_ids
                        for nid in node_ids:
                            if nid not in last_good:
                                last_good[nid] = None
                    last_rediscover = time.time()

                snapshot = {}
                for nid in node_ids:
                    pos = read_config(bus, nid, pos_ep["id"], pos_ep["type"])
                    if pos is not None and last_good[nid] is not None:
                        if abs(pos - last_good[nid]) > MAX_PLAUSIBLE_DELTA:
                            pos = None  # treat as a glitch, keep the last good value
                    if pos is not None:
                        last_good[nid] = pos
                    snapshot[str(nid)] = last_good[nid]

                with latest_lock:
                    latest.clear()
                    latest.update(snapshot)
                    latest["_updated"] = time.time()
                    latest["_healthy"] = True

                time.sleep(POLL_INTERVAL)

            except (can.CanError, OSError) as e:
                # can0 went down (bus-off dropping the interface, unplugged,
                # etc.) - the existing socket is likely dead. Mark the
                # snapshot unhealthy so a client can tell, close and reopen
                # the bus, force a fresh discovery next pass, and keep going
                # instead of letting this thread die silently.
                print(f"pos_stream_server: CAN error ({e}) - reopening bus in {BUS_REOPEN_DELAY}s...")
                with latest_lock:
                    latest["_updated"] = time.time()
                    latest["_healthy"] = False
                try:
                    bus.shutdown()
                except Exception:
                    pass
                time.sleep(BUS_REOPEN_DELAY)
                try:
                    bus = open_bus()
                except Exception as reopen_err:
                    print(f"pos_stream_server: failed to reopen bus ({reopen_err}), will retry")
                node_ids = []
                last_rediscover = 0.0
    finally:
        bus.shutdown()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != "/positions":
            self.send_response(404)
            self.end_headers()
            return
        with latest_lock:
            body = json.dumps(latest).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")  # allow the local viewer page to fetch this
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass  # keep the terminal quiet - don't print a line for every poll


def main():
    t = threading.Thread(target=poll_loop, daemon=True)
    t.start()
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\npos_stream_server stopped.")


if __name__ == "__main__":
    main()
