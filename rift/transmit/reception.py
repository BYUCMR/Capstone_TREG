import re
from collections.abc import Callable, Iterable

import numpy as np

from rift.arraytypes import Vector


def read_q(
    lines: Iterable[bytes],
    log: Callable[[str], object] | None = None,
) -> Vector[np.intp] | None:
    q_match = None
    for line in lines:
        string = line.decode()
        q_match = re.fullmatch(r'\[(.*)\]\r\n', string)
        if q_match is None and log is not None:
            log(string)
    if q_match:
        q = np.fromstring(q_match[1], sep=',', dtype=np.intp)
    else:
        q = None
    return q
