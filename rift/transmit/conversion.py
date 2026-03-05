import io

import numpy as np

from rift.arraytypes import Matrix

from . import commands as commands


def read_positions(ser: io.IOBase) -> Matrix | None:
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        if line.startswith('[') and line.endswith(']'):
            try:
                values = np.fromstring(line[1:-1], sep=',')
                return -values.reshape(-1, 1)
            except Exception:
                print(f"[WARN] Could not parse: {line}")
                return None
        else:
            # Print other serial messages like warnings
            print(f"[OTHER] {line}")
            return
