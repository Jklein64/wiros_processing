from __future__ import annotations

from typing import ClassVar

from .params import Params


class Algorithm:
    name: ClassVar[str]

    def __init__(self, params: Params):
        self.params = params

    @classmethod
    def from_string(cls, name):
        for subclass in cls.__subclasses__():
            if subclass.name == name:
                return subclass
        raise NameError(f'Could not find an algorithm with name "{name}"')


class FullSvd(Algorithm):
    name = "full_svd"


class FullMusic(Algorithm):
    name = "music"


class RxSvd(Algorithm):
    name = "rx_svd"


class Fft(Algorithm):
    name = "fft"


class Music1D(Algorithm):
    name = "aoa_music"


class Fft1D(Algorithm):
    name = "aoa_only"


class SpotFi(Algorithm):
    name = "spotfi"
