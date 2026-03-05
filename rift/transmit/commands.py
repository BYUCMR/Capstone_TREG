from collections.abc import Iterable
from typing import Final


STOP: Final = b"STOP\n"
RESET: Final = b"RESET\n"


def VEL(v: Iterable[float], t: float | None = None) -> bytes:
    if t is None:
        # "VEL" implicitly sets t to 1.5
        return f"VEL:{','.join(map(str, v))}\n".encode()
    else:
        return f"VEL_DUR:{','.join(map(str, v))}:{t}\n".encode()


def POS(q: Iterable[float]) -> bytes:
    return f"POS:{','.join(map(str, q))}\n".encode()
