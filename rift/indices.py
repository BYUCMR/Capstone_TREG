__all__ = [
    'L1',
    'L2',
    'L3',
    'R1',
    'R2',
    'R3',
    'PL1',
    'PL2',
    'PL3',
    'PR1',
    'PR2',
    'PR3',
]

from typing import Final

# Left feet / end-effectors
L1: Final = 0
L2: Final = 1
L3: Final = 2

# Right feet / end effectors
R1: Final = 3
R2: Final = 4
R3: Final = 5

# Left side of payload / body
PL1: Final = 6
PL2: Final = 7
PL3: Final = 8

# Right side of payload / body
PR1: Final = 9
PR2: Final = 10
PR3: Final = 11

# Slices
FEET: Final = slice(L1, R3+1)
PAYLOAD: Final = slice(PL1, PR3+1)
