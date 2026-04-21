from typing import Protocol

from rift.arraytypes import Matrix


class StateFunction[StateT, ReturnT](Protocol):
    def at(self, state: StateT, /) -> ReturnT: ...


class HasPos(Protocol):
    @property
    def pos(self, /) -> Matrix: ...


class HasRigidity(Protocol):
    @property
    def rigidity(self, /) -> Matrix: ...


class HasDxToDq(Protocol):
    @property
    def dx_to_dq(self, /) -> Matrix: ...
