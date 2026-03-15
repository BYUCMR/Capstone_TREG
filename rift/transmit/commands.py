from collections.abc import Iterable
from typing import Final

import numpy as np

from rift.arraytypes import Vector


STOP: Final = b"STOP\n"
RESET: Final = b"RESET\n"


def VEL(v: Iterable[float], t: float | None = None) -> bytes:
    if t is None:
        # "VEL" implicitly sets t to 1.5
        return f"VEL:{','.join(str(int(vi)) for vi in v)}\n".encode()
    else:
        return f"VEL_DUR:{','.join(str(int(vi)) for vi in v)}:{t}\n".encode()


def POS(q: Iterable[int]) -> bytes:
    return f"POS:{','.join(map(str, q))}\n".encode()


def get_smallest_dt(dq: Vector[np.number], max_speed: float = 1800) -> float:
    return np.max(np.abs(dq)) / max_speed
