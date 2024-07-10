from __future__ import annotations

import functools
import weakref
from typing import ClassVar

import numpy as np
from aoa_node.aoa_params import Params
from common.constants import SUBCARRIER_FREQUENCIES, C


# see https://stackoverflow.com/a/68052994
def weak_lru(maxsize=128, typed=False):
    'LRU Cache decorator that keeps a weak reference to "self". Shared by the entire class.'

    def wrapper(func):
        @functools.lru_cache(maxsize, typed)
        def _func(_self, *args, **kwargs):
            return func(_self(), *args, **kwargs)

        @functools.wraps(func)
        def inner(self, *args, **kwargs):
            return _func(weakref.ref(self), *args, **kwargs)

        return inner

    return wrapper


class Algorithm:
    name: ClassVar[str]

    def __init__(self, params: Params, channel, bandwidth):
        self.params = params
        self.channel = channel
        self.bandwidth = bandwidth
        theta_min = self.params.theta_min
        theta_max = self.params.theta_max
        theta_count = self.params.theta_count
        self.theta_samples = np.linspace(theta_min, theta_max, theta_count)
        tau_min = self.params.tau_min
        tau_max = self.params.tau_max
        tau_count = self.params.tau_count
        self.tau_samples = np.linspace(tau_min, tau_max, tau_count)

    def evaluate(self, new_csi: np.ndarray):
        raise NotImplementedError()

    def aoa_steering_vector(self, theta_samples=None):
        return self._aoa_steering_vector(
            self.channel,
            self.bandwidth,
            self.theta_samples if theta_samples is None else theta_samples,
        )

    @weak_lru(maxsize=16)
    def _aoa_steering_vector(self, channel, bandwidth, theta_samples):
        # calculate wave number
        freqs = 5e9 + channel * 5e6 + SUBCARRIER_FREQUENCIES[bandwidth]
        k = 2 * np.pi * np.mean(freqs) / C
        # helper variables
        theta_samples = np.atleast_1d(theta_samples)
        dx, dy = self.params.rx_position.T
        # column j corresponds to steering vector for j'th theta sample
        A = np.repeat(np.expand_dims(theta_samples, axis=0), len(dx), axis=0)
        A = np.exp(-1.0j * k * (dx * np.cos(A) + dy * np.sin(A)))
        # A now has shape (n_rx, len(theta_samples))
        return A

    def tof_steering_vector(self, tau_samples=None):
        return self._tof_steering_vector(
            self.channel,
            self.bandwidth,
            self.tau_samples if tau_samples is None else tau_samples,
        )

    @weak_lru(maxsize=16)
    def _tof_steering_vector(self, channel, bandwidth, tau_samples):
        freqs = 5e9 + channel * 5e6 + SUBCARRIER_FREQUENCIES[bandwidth]
        omega = 2 * np.pi * freqs / C
        tau_samples = np.atleast_1d(tau_samples)
        # column j corresponds to steering vector for the j'th tau sample
        B = np.exp(-1.0j * np.outer(omega, tau_samples))
        # B now has shape (n_sub, len(tau_samples))
        return B

    @classmethod
    def from_string(cls, name):
        for subclass in cls.__subclasses__():
            if subclass.name == name:
                return subclass
        raise NameError(f'Could not find an algorithm with name "{name}"')

    def __eq__(self, other):
        # needed for lru_cache
        if other is None:
            return False
        return isinstance(other.__class__, self.__class__)

    def __hash__(self):
        # needed for lru_cache
        return self.__class__.name


class Svd(Algorithm):
    name = "full_svd"


class SvdReduced(Algorithm):
    name = "rx_svd"


class Music(Algorithm):
    name = "music"


class Music1D(Algorithm):
    name = "aoa_music"


class MatrixProduct(Algorithm):
    name = "fft"


class MatrixProduct1D(Algorithm):
    name = "aoa_only"


class SpotFi(Algorithm):
    name = "spotfi"
