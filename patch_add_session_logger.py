#!/usr/bin/env python3
"""
The urwid UI redraws over the whole terminal constantly, so anything
printed gets overwritten and scrollback is unreliable - this is why so
much of yesterday's troubleshooting depended on catching a screenshot
at exactly the right moment.

Fix: wrap Python's built-in print() so every single print() call already
in this file (all ~50+ of them - [INFO], [WARNING], [CRITICAL], [SAFETY],
[SAFE_UP] etc.) ALSO gets appended, with a timestamp, to a plain text
log file (session_log.txt) in the same folder. This is the only change -
one location, purely additive. It does not change what shows on screen,
does not touch any control loop, arming, watchdog, or safety logic at
all. After any test run, `cat session_log.txt` (or just the tail of it)
gives a clean, complete, chronological record to paste back for review,
independent of whatever the screen looked like at any given moment.
"""

path = "gamecontroller.py"
src = open(path).read()
original_src = src

def apply_edit(name, old, new):
    global src
    count = src.count(old)
    assert count == 1, f"[{name}] expected 1 match, found {count} - aborting, nothing written."
    src = src.replace(old, new)
    print(f"[{name}] OK")

apply_edit(
    "add background session logger (wraps print(), additive only)",
    '''#!/usr/bin/env python3

import time
import threading
import signal
import urwid
import pygame
import can

from src.can_utils import discover_node_ids
from src.control import move_odrive_to_position, set_closed_loop_control, set_idle_mode
from src.metrics import get_metrics, METRIC_ENDPOINTS
from src.configure import load_endpoints, read_config, write_config

# ------------------------------------------------------------------------------''',
    '''#!/usr/bin/env python3

import time
import threading
import signal
import urwid
import pygame
import can
import builtins
from datetime import datetime

from src.can_utils import discover_node_ids
from src.control import move_odrive_to_position, set_closed_loop_control, set_idle_mode
from src.metrics import get_metrics, METRIC_ENDPOINTS
from src.configure import load_endpoints, read_config, write_config

# ------------------------------------------------------------------------------
# Background session logger - purely additive, does not touch any control,
# arming, or safety logic. urwid redraws over the whole terminal constantly,
# so scrollback is unreliable; this file is not. Every print() call already
# in this script also gets appended here, with a timestamp.
_session_log = open("session_log.txt", "a", buffering=1)
_session_log.write(f"\\n===== session started {datetime.now().isoformat(timespec='seconds')} =====\\n")
_real_print = builtins.print
def _logging_print(*args, **kwargs):
    _real_print(*args, **kwargs)
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    text = kwargs.get("sep", " ").join(str(a) for a in args)
    _session_log.write(f"[{ts}] {text}\\n")
builtins.print = _logging_print
# ------------------------------------------------------------------------------''',
)

open(path, "w").write(src)
print(f"\\nAll edits applied. {len(original_src.splitlines())} -> {len(src.splitlines())} lines.")
