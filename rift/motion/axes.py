from typing import Final

import numpy as np

X: Final = np.array((1., 0., 0.))
Y: Final = np.array((0., 1., 0.))
Z: Final = np.array((0., 0., 1.))
X.setflags(write=False)
Y.setflags(write=False)
Z.setflags(write=False)
