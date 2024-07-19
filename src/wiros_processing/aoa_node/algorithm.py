from __future__ import annotations

from typing import ClassVar

import numpy as np
from typing_extensions import override

from ..constants import SUBCARRIER_FREQUENCIES, C
from .aoa_params import AoaParams


class Algorithm:
    """Angle of Arrival (AoA) and Time of Flight (ToF) extraction algorithm.

    Note that many aspects of the algorithm depend on the channel and bandwidth of the
    provided CSI matrices. Create a new algorithm instance for each channel/bandwidth
    combination.

    Attributes:
        name: the name of the algorithm. Used to match string to class.
        theta_samples: theta_count values between theta_min and theta_max in radians
        tau_samples: tau_count values between tau_min and tau_max in nanoseconds
    """

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
        """Processes a new CSI matrix.

        AoA and ToF extraction are separated from this callback function so the
        algorithms can be run at a different rate from the data collection rate.

        Args:
            new_csi: Complex matrix with shape (n_sub, n_rx, n_tx).
        """
        raise NotImplementedError()

    def evaluate(self) -> tuple[float, np.ndarray]:
        """Extracts AoA and ToF information.

        Returns:
            A tuple containing the most likely AoA (in radians) and the 2d profile with
            shape (theta_count, tau_count) where larger values correspond to increased
            likelihood that the AoA/ToF combination is correct.
        """
        raise NotImplementedError()

    def aoa_steering_vector(self, theta_samples=None):
        """Creates an AoA steering vector based on this instance's params.

        Returns:
            An ndarray with shape (n_rx, theta_count)
        """
        # channel is 155, bandwidth is 80
        freqs = 5e9 + 155 * 5e6 + SUBCARRIER_FREQUENCIES[self.bandwidth * 1e6]
        # calculate wave number
        k = 2 * np.pi * np.mean(freqs) / C
        # expand dims so broadcasting works later
        dx, dy = np.expand_dims(self.params.rx_position.T, axis=2)
        # column j corresponds to steering vector for j'th theta sample
        theta_samples = np.atleast_1d(self.theta_samples)
        A = np.repeat(np.expand_dims(theta_samples, axis=0), len(dx), axis=0)
        A = np.exp(-1.0j * k * (dx * np.cos(A) + dy * np.sin(A)))
        # A now has shape (n_rx, len(theta_samples))
        return A
        # # calculate wave number
        # freqs = 5e9 + self.channel * 5e6 + SUBCARRIER_FREQUENCIES[self.bandwidth * 1e6]
        # k = 2 * np.pi * np.mean(freqs) / C
        # # expand dims so broadcasting works later
        # dx, dy = np.expand_dims(self.params.rx_position.T, axis=2)
        # # column j corresponds to steering vector for j'th theta sample
        # theta_samples = np.atleast_1d(self.theta_samples)
        # A = np.repeat(np.expand_dims(theta_samples, axis=0), len(dx), axis=0)
        # A = np.exp(-1.0j * k * (dx * np.cos(A) + dy * np.sin(A)))
        # # A now has shape (n_rx, theta_count)
        # return A

    def tof_steering_vector(self, tau_samples=None):
        """
        Creates a ToF steering vector based on this instance's params.

        Returns:
            An ndarray with shape (n_sub, tau_count)
        """
        # channel is 155, bandwidth is 80
        freqs = 5e9 + 155 * 5e6 + SUBCARRIER_FREQUENCIES[self.bandwidth * 1e6]
        # f_delta = freqs - freqs[0]
        omega = 2 * np.pi * freqs / C
        tau_samples = np.atleast_1d(self.tau_samples)
        # column j corresponds to steering vector for the j'th tau sample
        # B = np.exp(-1.0j * np.outer(2 * np.pi * f_delta, tau_samples))
        B = np.exp(-1.0j * np.outer(omega, tau_samples))
        # B now has shape (n_sub, len(tau_samples))
        return B
        # freqs = 5e9 + self.channel * 5e6 + SUBCARRIER_FREQUENCIES[self.bandwidth * 1e6]
        # omega = 2 * np.pi * freqs / C
        # tau_samples = np.atleast_1d(self.tau_samples)
        # # column j corresponds to steering vector for the j'th tau sample
        # B = np.exp(-1.0j * np.outer(omega, tau_samples))
        # # B now has shape (n_sub, len(tau_samples))
        # return B

    @classmethod
    def from_params(cls, params: AoaParams, channel, bandwidth):
        """Instantiates an algorithm with the given parameters, channel, and bandwidth.

        Args:
            params: aoa_node parameters object, which include algorithm name
            channel: the WiFi channel corresponding to the CSI matrices
            bandwidth: the WiFi bandwidth corresponding to the CSI matrices
        """
        for subclass in cls.__subclasses__():
            if subclass.name == params.algo:
                return subclass(params, channel, bandwidth)
        raise NameError(f'Could not find an algorithm with name "{params.algo}"')


class CircularBuffer:
    """Circular buffer with a fixed size that stores numpy arrays.

    The buffer is immediately initialized to the maximum size and filled with zeros.
    This might give unexpected behavior when performing operations like an average
    over the buffer contents before every element has been filled with data.
    """

    def __init__(self, maxlen: int):
        """Creates a circular buffer.

        Args:
            maxlen: maximum length. Only the last maxlen data elements are retained.
        """
        self.maxlen = maxlen
        self.buffer = None
        self.index = 0

    def push(self, new_data):
        """Pushes data into the circular buffer, removing old data as needed.

        The shape of the buffer is determined by the shape of the data the first time
        this method is called, and this class assumes that all subsequent data has
        the same shape.

        Args:
            new_data: the data to push to the circular buffer.
        """
        if self.buffer is None:
            shape = (*np.shape(new_data), self.maxlen)
            self.buffer = np.zeros(shape, dtype=np.complex128)
        self.buffer[..., self.index] = new_data
        self.index = (self.index + 1) % self.maxlen

    def asarray(self):
        """Retrieves a read-only view of the buffer contents.

        If the data put into the buffer had shape s, then the returned array will have
        shape (*s, maxlen). Each data point in the buffer is indexed using the last
        index of the buffer.
        """
        arr = self.buffer.view()
        arr.flags.writeable = False
        return arr


class Svd(Algorithm):
    """
    2d DFT algorithm using SVD to merge data across frames and transmitters.
    """

    name = "full_svd"

    def __init__(self, params: AoaParams, channel, bandwidth):
        super().__init__(params, channel, bandwidth)
        self.buffer = CircularBuffer(maxlen=params.keep_last)
        self.csi_shape = None
        self.csi = None

    @override
    def csi_callback(self, new_csi):
        self.csi = new_csi
        if self.csi_shape is None:
            self.csi_shape = np.shape(new_csi)
        # # treat each tx as if it was a separate measurement
        for tx in range(self.csi_shape[2]):
            self.buffer.push(np.reshape(new_csi[:, :, tx], (-1,), order="F"))

    @override
    def evaluate(self):
        n_sub, n_rx, _ = self.csi_shape
        A = self.aoa_steering_vector()  # (n_rx, theta_count)
        B = self.tof_steering_vector()  # (n_sub, tau_count)
        # X = self.buffer.asarray()  # (n_sub * n_rx, n_data)
        # # use first principal component of csi measurements
        # u, _, _ = np.linalg.svd(X)
        # csi = np.reshape(u[:, 0], (n_sub, n_rx), order="F")
        csi = self.csi[:, :, 0]
        # result will have peaks for correct theta/tau
        profile = np.abs(A.conj().T @ csi.conj().T @ B)
        # find theta corresponding to highest peak in intensity
        theta_index = np.unravel_index(np.argmax(profile), profile.shape)[0]
        theta = self.theta_samples[theta_index]
        return (theta, profile)


class SvdReduced(Algorithm):
    """
    Implementation of WAIS algorithm, a 1d DFT algorithm that avoids needing ToF data.
    """

    name = "rx_svd"

    def __init__(self, params: AoaParams, channel, bandwidth):
        super().__init__(params, channel, bandwidth)

    def csi_callback(self, new_csi): ...

    def evaluate(self): ...


class Music(Algorithm):
    name = "music"


class Music1D(Algorithm):
    name = "aoa_music"


class MatrixProduct(Algorithm):
    name = "fft"

    def __init__(self, params: AoaParams, channel, bandwidth):
        super().__init__(params, channel, bandwidth)

    def csi_callback(self, new_csi): ...

    def evaluate(self): ...


class MatrixProduct1D(Algorithm):
    name = "aoa_only"

    def __init__(self, params: AoaParams, channel, bandwidth):
        super().__init__(params, channel, bandwidth)

    def csi_callback(self, new_csi): ...

    # untested!
    def evaluate(self): ...


class SpotFi(Algorithm):
    name = "spotfi"
    name = "spotfi"
