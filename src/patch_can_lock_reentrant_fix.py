#!/usr/bin/env python3
"""
Bug in the last patch: read_config() grabs can_lock, then WHILE STILL
HOLDING IT calls send_can_message(), which also tries to grab can_lock.
A plain threading.Lock() is not reentrant - a thread trying to take a
lock it already holds just waits forever for itself. That's the exact
hang just seen at "Reading actual current positions..." (confirmed by
the Ctrl+C traceback, which shows the second, nested acquire).

Fix: change can_lock from threading.Lock() to threading.RLock() - a
reentrant lock. Behaves identically for keeping two different threads
out of each other's way (which is the whole point), but the same
thread can safely acquire it again from a call it's already inside of.
One-word fix, verified locally with a test that reproduces the nested
acquire and confirms it now completes instead of hanging.
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
    "can_lock: Lock -> RLock (reentrant, fixes self-deadlock)",
    '''can_lock = threading.Lock()  # serializes all bus.send/bus.recv across threads''',
    '''can_lock = threading.RLock()  # serializes all bus.send/bus.recv across threads''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
