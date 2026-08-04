#!/usr/bin/env python3
"""
Nothing in this project ever coordinated who gets to talk to the CAN bus
at a given moment - the main control loop, the watchdog check, and the
metrics/UI thread all call bus.recv()/bus.send() independently, with no
lock between them. That means one thread's recv() can pick up the reply
meant for a different thread's request, so the original requester finds
nothing waiting and has to retry - a very plausible explanation for the
0.17-0.4s stalls still showing up after shortening the per-read timeout.

Fix: a single threading.Lock() ("can_lock"), created once in can_utils.py.
send_can_message() (the only place that sends) now holds it for the
duration of the send. This is NOT a retry or recovery loop - it's a
plain "wait your turn," at most as long as one send takes.
"""

path = "can_utils.py"
src = open(path).read()
original_src = src

def apply_edit(name, old, new):
    global src
    count = src.count(old)
    assert count == 1, f"[{name}] expected 1 match, found {count} - aborting, nothing written."
    src = src.replace(old, new)
    print(f"[{name}] OK")

apply_edit(
    "A: add threading import + shared can_lock",
    '''import struct
import time
import can

def extract_node_id(arbitration_id):''',
    '''import struct
import threading
import time
import can

can_lock = threading.Lock()  # serializes all bus.send/bus.recv across threads
                              # (main loop, watchdog, metrics/UI thread, and any
                              # automated sequence) so one thread's response can't
                              # get stolen by another thread's recv() call.

def extract_node_id(arbitration_id):''',
)

apply_edit(
    "B: hold can_lock for the bus.send in send_can_message",
    '''        bus.send(message, timeout=0.1)  # bounded - a stuck/bus-off TX queue must not hang the caller (this is on the same path as the emergency disarm)
        return True''',
    '''        with can_lock:  # keep this send from landing in the middle of another thread's read
            bus.send(message, timeout=0.1)  # bounded - a stuck/bus-off TX queue must not hang the caller (this is on the same path as the emergency disarm)
        return True''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
