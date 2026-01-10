import numpy as np

type Vector = np.ndarray[tuple[int], np.dtype[np.floating]]
type Matrix = np.ndarray[tuple[int, int], np.dtype[np.floating]]
type MatrixStack = np.ndarray[tuple[int, int, int], np.dtype[np.floating]]
