from __future__ import annotations

from collections import deque
from typing import ClassVar

import numpy as np
import scipy as sp

from ..constants import SUBCARRIER_FREQUENCIES, C
from .aoa_params import AoaParams


class Algorithm:
    name: ClassVar[str]

    def __init__(self, params: AoaParams, channel, bandwidth):
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

    def csi_callback(self, new_csi: np.ndarray):
        raise NotImplementedError()

    def evaluate(self) -> tuple[float, np.ndarray]:
        raise NotImplementedError()

    def aoa_steering_vector(self, theta_samples=None):
        """
        Creates a steering vector for AoA with shape `(n_rx, len(theta_samples))`. If the `theta_samples` parameter is `None`, then this method uses samples determined by the parameters passed during initialization.
        """
        # calculate wave number
        freqs = 5e9 + self.channel * 5e6 + SUBCARRIER_FREQUENCIES[self.bandwidth * 1e6]
        k = 2 * np.pi * np.mean(freqs) / C
        # expand dims so broadcasting works later
        dx, dy = np.expand_dims(self.params.rx_position.T, axis=2)
        # column j corresponds to steering vector for j'th theta sample
        theta_samples = self.theta_samples if theta_samples is None else theta_samples
        theta_samples = np.atleast_1d(theta_samples)
        A = np.repeat(np.expand_dims(theta_samples, axis=0), len(dx), axis=0)
        A = np.exp(-1.0j * k * (dx * np.cos(A) + dy * np.sin(A)))
        # A now has shape (n_rx, len(theta_samples))
        return A

    def tof_steering_vector(self, tau_samples=None):
        """
        Creates a steering vector for ToF with shape `(n_sub, len(tau_samples))`. If the `tau_samples` parameter is `None`, then this method uses samples determined by the parameters passed during initialization.
        """
        freqs = 5e9 + self.channel * 5e6 + SUBCARRIER_FREQUENCIES[self.bandwidth * 1e6]
        omega = 2 * np.pi * freqs / C
        tau_samples = self.tau_samples if tau_samples is None else tau_samples
        tau_samples = np.atleast_1d(tau_samples)
        # column j corresponds to steering vector for the j'th tau sample
        B = np.exp(-1.0j * np.outer(omega, tau_samples))
        # B now has shape (n_sub, len(tau_samples))
        return B

    @classmethod
    def from_params(cls, params: AoaParams, channel, bandwidth):
        for subclass in cls.__subclasses__():
            if subclass.name == params.algo:
                return subclass(params, channel, bandwidth)
        raise NameError(f'Could not find an algorithm with name "{params.algo}"')


class CsiRoller:
    def __init__(self, keep_last=20):
        self._deque = deque()
        self._keep_last = keep_last
        # assumes shape is constant per-instance
        self.input_shape = None
        self.output_shape = None

    def add(self, new_csi):
        """
        Adds `new_csi` to the roller, removing old ones when there would be more than `keep_last` stored. Expects `new_csi` to have shape `(n_sub, n_rx, n_tx)`. Treats the transmitter dimension as separate measurements, so actually stores multiple arrays with shape `(n_sub, n_rx)` each method call.
        """
        n_sub, n_rx, n_tx = np.shape(new_csi)
        if self.input_shape is None:
            self.input_shape = (n_sub, n_rx, n_tx)

        for tx in range(n_tx):
            self._deque.append(new_csi[:, :, tx])

        while len(self._deque) > self._keep_last:
            self._deque.popleft()

        self.output_shape = (len(self._deque), n_sub, n_rx)

    def aggregate(self):
        """
        Returns the aggregate data, an array with shape `(n_data, n_sub, n_rx)`, where `n_data` is the number of data points saved, a value between zero and `keep_last` (inclusive).
        """
        return np.asarray(self._deque)


class Svd(Algorithm):
    name = "full_svd"

    def __init__(self, params: AoaParams, channel, bandwidth):
        super().__init__(params, channel, bandwidth)
        self.csi_roller = CsiRoller()

    def csi_callback(self, new_csi):
        self.csi_roller.add(new_csi)

    def evaluate(self):
        n_data, n_sub, n_rx = self.csi_roller.output_shape
        # create aggregate csi as largest right singular vector of stacked frames
        X = np.reshape(self.csi_roller.aggregate(), (n_data, n_sub * n_rx))
        _, _, vh = sp.sparse.linalg.svds(X, k=1, which="LM")
        aggregate_csi = np.reshape(vh, (n_sub, n_rx)).T  #  (n_rx, n_sub)
        # create profile and get AoA by comparing against steering vectors
        A = self.aoa_steering_vector()  # (n_rx, len(theta_samples))
        B = self.tof_steering_vector()  # (n_sub, len(tau_samples))
        profile = np.abs(A.conj().T @ aggregate_csi @ B)
        # find theta corresponding to highest peak in intensity
        theta_index = np.unravel_index(np.argmax(profile), profile.shape)[0]
        theta = self.theta_samples[theta_index]
        return (theta, profile)


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
