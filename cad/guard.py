# SPDX-License-Identifier: CC-BY-NC-SA-4.0
"""Memory watchdog: import first.  Idle priority; self-kill above 1.5 GB working set (checked every 2 s). ctypes only (no psutil in the venv)."""
import ctypes, os, threading, time
from ctypes import wintypes
k32, psapi = ctypes.WinDLL("kernel32", use_last_error=True), ctypes.WinDLL("psapi", use_last_error=True)
h = k32.GetCurrentProcess()
k32.SetPriorityClass(h, 0x40)          # IDLE_PRIORITY_CLASS
class PMC(ctypes.Structure):
    _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD)] + [(n, ctypes.c_size_t) for n in
        ("PeakWorkingSetSize", "WorkingSetSize", "QuotaPeakPagedPoolUsage", "QuotaPagedPoolUsage", "QuotaPeakNonPagedPoolUsage", "QuotaNonPagedPoolUsage", "PagefileUsage", "PeakPagefileUsage")]
def ws():
    c = PMC(); c.cb = ctypes.sizeof(c); psapi.GetProcessMemoryInfo(h, ctypes.byref(c), c.cb); return c.WorkingSetSize
def _w():
    while True:
        if ws() > 1.5 * 2**30:
            print("GUARD: >1.5 GB, killing self", flush=True); os._exit(3)
        time.sleep(2)
threading.Thread(target=_w, daemon=True).start()
