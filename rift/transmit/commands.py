import io
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Protocol


class Command(Protocol):
    def __bytes__(self) -> bytes: ...


@dataclass(slots=True, frozen=True)
class VEL:
    v: Iterable[float] = ()
    t: float = 1.5

    def __bytes__(self) -> bytes:
        return f"VEL_DUR:{','.join(map(str, self.v))}:{self.t}\n".encode()


@dataclass(slots=True, frozen=True)
class POS:
    q: Iterable[float] = ()

    def __bytes__(self) -> bytes:
        return f"POS:{','.join(map(str, self.q))}\n".encode()


@dataclass(slots=True, frozen=True)
class STOP:
    def __bytes__(self) -> bytes:
        return b"STOP\n"


@dataclass(slots=True, frozen=True)
class RESET:
    def __bytes__(self) -> bytes:
        return b"RESET\n"


def send(ser: io.IOBase, cmd: Command) -> bytes:
    b = bytes(cmd)
    ser.write(b)
    ser.flush()
    print(f"[SENT] {b.decode()}", end='')
    return ser.readline()
