"""Process resource capture for the bed's evidence files (peak RSS, threads, CPU time)."""

from __future__ import annotations

import os
import resource
import time


def snapshot() -> dict:
    ru = resource.getrusage(resource.RUSAGE_SELF)
    threads = None
    try:
        threads = int(open("/proc/self/status").read().split("Threads:")[1].split()[0])
    except (OSError, IndexError, ValueError):
        pass
    return {
        "peak_rss_mb": round(ru.ru_maxrss / 1024.0, 1),  # Linux reports KiB
        "cpu_user_s": round(ru.ru_utime, 1),
        "cpu_system_s": round(ru.ru_stime, 1),
        "threads": threads,
        "cpus_online": os.cpu_count(),
        "wall_clock": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    }
