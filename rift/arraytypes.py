import numpy as np

type Vector[T: np.generic = np.floating] = np.ndarray[tuple[int], np.dtype[T]]
type Matrix[T: np.generic = np.floating] = np.ndarray[tuple[int, int], np.dtype[T]]
type MatrixStack[T: np.generic = np.floating] = np.ndarray[tuple[int, int, int], np.dtype[T]]
