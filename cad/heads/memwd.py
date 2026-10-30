# SPDX-License-Identifier: CC-BY-NC-SA-4.0
import os, threading, time, ctypes
from ctypes import wintypes
class PMC(ctypes.Structure):
    _fields_=[("cb",wintypes.DWORD),("PageFaultCount",wintypes.DWORD),("PeakWorkingSetSize",ctypes.c_size_t),("WorkingSetSize",ctypes.c_size_t),("QuotaPeakPagedPoolUsage",ctypes.c_size_t),("QuotaPagedPoolUsage",ctypes.c_size_t),("QuotaPeakNonPagedPoolUsage",ctypes.c_size_t),("QuotaNonPagedPoolUsage",ctypes.c_size_t),("PagefileUsage",ctypes.c_size_t),("PeakPagefileUsage",ctypes.c_size_t)]
def _rss():
    pmc=PMC(); pmc.cb=ctypes.sizeof(pmc)
    ctypes.windll.psapi.GetProcessMemoryInfo(ctypes.windll.kernel32.GetCurrentProcess(),ctypes.byref(pmc),pmc.cb)
    return pmc.WorkingSetSize
def start(limit=1.5e9):
    # idle priority
    try: ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(),0x40)
    except Exception: pass
    def run():
        while True:
            if _rss()>limit:
                print("MEM WATCHDOG KILL",_rss()/1e6,flush=True); os._exit(9)
            time.sleep(0.5)
    threading.Thread(target=run,daemon=True).start()
