#!/usr/bin/env python3
"""
Companion to patch_can_bus_lock_utils.py. That patch protects every send;
this one protects the read side. read_config() now holds the same
can_lock for the whole span of one attempt - the stale-message flush,
the request, and the wait-for-reply loop - so no other thread can sneak
a send or a recv in in between and take the reply meant for this call.

Still exactly the same loop structure as before, just wrapped in a
"with" block (mutual exclusion, not a retry). Worst case one thread
holds the lock for ~timeout (0.03s) before the next thread gets a turn.
"""

path = "configure.py"
src = open(path).read()
original_src = src

def apply_edit(name, old, new):
    global src
    count = src.count(old)
    assert count == 1, f"[{name}] expected 1 match, found {count} - aborting, nothing written."
    src = src.replace(old, new)
    print(f"[{name}] OK")

apply_edit(
    "A: import shared can_lock",
    '''from src.can_utils import send_can_message, receive_can_message''',
    '''from src.can_utils import send_can_message, receive_can_message, can_lock''',
)

apply_edit(
    "B: hold can_lock for the whole read attempt (flush + request + wait)",
    '''    for attempt in range(retries):
        for _ in range(50):
            if bus.recv(timeout=0) is None:
                break

        send_can_message(bus, node_id, RXSDO, '<BHB', READ, endpoint_id, 0)

        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = bus.recv(timeout=0)
            if msg is None:
                continue
            if msg.arbitration_id != expected_arb_id:
                continue
            if len(msg.data) < needed:
                continue
            _, _, _, value = struct.unpack_from(fmt, msg.data)
            return value
    return None''',
    '''    for attempt in range(retries):
        with can_lock:  # hold the bus for this whole attempt so another thread
                         # can't send/recv in between and steal our response
            for _ in range(50):
                if bus.recv(timeout=0) is None:
                    break

            send_can_message(bus, node_id, RXSDO, '<BHB', READ, endpoint_id, 0)

            start_time = time.time()
            while time.time() - start_time < timeout:
                msg = bus.recv(timeout=0)
                if msg is None:
                    continue
                if msg.arbitration_id != expected_arb_id:
                    continue
                if len(msg.data) < needed:
                    continue
                _, _, _, value = struct.unpack_from(fmt, msg.data)
                return value
    return None''',
)

open(path, "w").write(src)
print(f"\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
