from collections.abc import Iterable
from typing import Final

import numpy as np

from rift.arraytypes import Vector
import struct


STOP: Final = b"STOP\n"
bStop: Final = struct.pack("<B",1)
RESET: Final = b"RESET\n"
bRESET: Final = struct.pack("<B",0)


def VEL(v: Iterable[float], t: float | None = None) -> bytes:
    if t is None:
        # "VEL" implicitly sets t to 1.5
        return f"VEL:{','.join(str(int(vi)) for vi in v)}\n".encode()
    else:
        return f"VEL_DUR:{','.join(str(int(vi)) for vi in v)}:{t}\n".encode()
    
def bVEL(v: Iterable[float], t: float | None = None) -> bytes:
    if t is None:
        fmt = f"<B{len(v)*'i'}"
        print(fmt)
        return struct.pack(fmt,2,*v)
    else:
        fmt = f"<B{len(v)}if"
        return struct.pack(fmt,3,*v,t)


def POS(q: Iterable[int]) -> bytes:
    return f"POS:{','.join(map(str, q))}\n".encode()

def bPOS(q: Iterable[int]) -> bytes:
    fmt = f"<B{len(q)}i"
    return struct.pack(fmt,4,*q)

def get_smallest_dt(dq: Vector[np.number], max_speed: float = 1800) -> float:
    return np.max(np.abs(dq)) / max_speed
