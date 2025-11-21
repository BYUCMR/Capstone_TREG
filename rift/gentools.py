import functools
from collections.abc import Callable, Generator, Iterator
from typing import Never


def auto_initialize[**P, G: Generator[object, Never, object]](func: Callable[P, G]) -> Callable[P, G]:
    @functools.wraps(func)
    def wrapped(*args: P.args, **kwargs: P.kwargs) -> G:
        gen = func(*args, **kwargs)
        next(gen)
        return gen

    return wrapped


def expend(it: Iterator[object]) -> None:
    for _ in it:
        pass
