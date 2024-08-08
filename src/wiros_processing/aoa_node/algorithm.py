from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar

import numpy as np
from typing_extensions import override

from ..constants import C_SPEED, F0, USABLE_SUBCARRIER_FREQUENCIES

if TYPE_CHECKING:
    # avoids circular import
    from .aoa_node import Params

SMALLEST_NORMAL = np.finfo(np.float64).smallest_normal


class Algorithm:
    """Angle of Arrival (AoA) and Time of Flight (ToF) extraction algorithm.

    Note that many aspects of the algorithm depend on the channel and bandwidth of the
    provided CSI matrices. Create a new algorithm instance for each channel/bandwidth
    combination. See the README for algorithm explanations. Variables are named to
    resemble the math.

    Attributes:
        name: the name of the algorithm. Used to match string to class.
        channel: the WiFi channel of the provided CSI matrices.
        bandwidth: the WiFi bandwidth of the provided CSI matrices.
        theta_samples: theta_count values between theta_min and theta_max in radians.
        tau_samples: tau_count values between tau_min and tau_max in nanoseconds.
    """

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

    def csi_callback(self, new_csi: np.ndarray):
        """Processes a new CSI matrix.

        AoA and ToF extraction are separated from this callback function so the
        algorithms can be run at a different rate from the data collection rate.

        Args:
            new_csi: Complex matrix with shape (n_sub, n_rx, n_tx).
        """
        raise NotImplementedError()

    def evaluate(self) -> np.ndarray:
        """Extracts AoA and ToF information.

        Returns:
            A 2d profile with shape (theta_count, tau_count) where larger values
            correspond to increased likelihood that the AoA/ToF combination is correct.
        """
        raise NotImplementedError()

    def aoa_steering_vector(self):
        """Creates an AoA steering vector based on this instance's params.

        Returns:
            An ndarray with shape (n_rx, theta_count).
        """
        # calculate wave number
        k = 2 * np.pi * F0 / C_SPEED
        # expand dims so broadcasting works later
        dx, dy = np.expand_dims(self.params.rx_position.T, axis=2)
        # column j corresponds to steering vector for j'th theta sample
        A = np.repeat(np.expand_dims(self.theta_samples, axis=0), len(dx), axis=0)
        A = np.exp(1j * k * (dx * np.cos(A) + dy * np.sin(A)))
        # A now has shape (n_rx, theta_count)
        return A

    def tof_steering_vector(self):
        """
        Creates a ToF steering vector based on this instance's params.

        Returns:
            An ndarray with shape (n_sub, tau_count).
        """
        omega = 2 * np.pi * USABLE_SUBCARRIER_FREQUENCIES / C_SPEED
        # column j corresponds to steering vector for the j'th tau sample
        B = np.exp(-1.0j * np.outer(omega, self.tau_samples))
        # B now has shape (n_sub, tau_count)
        return B

    @classmethod
    def from_params(cls, params: Params, channel, bandwidth):
        """Instantiates an algorithm with the given parameters, channel, and bandwidth.

        Args:
            params: aoa_node parameters object, which include algorithm name.
            channel: the WiFi channel corresponding to the CSI matrices.
            bandwidth: the WiFi bandwidth corresponding to the CSI matrices.
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

    Attributes:
        maxlen: the maximum number of items in the buffer.
        buffer: the raw data array backing the CircularBuffer.
        index: number such that buffer[..., index] is the most recent data point.
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

    name = "svd"

    def __init__(self, params: Params, channel, bandwidth):
        super().__init__(params, channel, bandwidth)
        self.buffer = CircularBuffer(maxlen=params.buffer_size)
        self.A = self.aoa_steering_vector()  # (n_rx, theta_count)
        self.B = self.tof_steering_vector()  # (n_sub, tau_count)
        self.n_sub = None
        self.n_rx = None

    @override
    def csi_callback(self, new_csi):
        n_sub, n_rx, n_tx = np.shape(new_csi)
        # store shape for later
        if self.n_sub is None:
            self.n_sub = n_sub
        if self.n_rx is None:
            self.n_rx = n_rx

        for tx in range(n_tx):
            # treat each tx as if it was a separate measurement
            self.buffer.push(np.reshape(new_csi[:, :, tx], (-1,), order="F"))

    @override
    def evaluate(self):
        C = self.buffer.asarray()  # (n_sub * n_rx, n_data)
        # use first principal component of flattened csi measurements
        u, _, _ = np.linalg.svd(C)
        X = np.reshape(u[:, 0], (self.n_sub, self.n_rx), order="F")
        # compute (theta_count, tau_count) profile
        return np.abs(self.A.conj().T @ X.T @ self.B)


class Wiros(Algorithm):
    """
    The RxSVD algorithm from WiROS.
    """

    name = "wiros"

    def __init__(self, params: Params, channel, bandwidth):
        super().__init__(params, channel, bandwidth)
        self.buffer = CircularBuffer(maxlen=params.buffer_size)
        self.A = self.aoa_steering_vector()  # (n_rx, theta_count)
        self.B = self.tof_steering_vector()  # (n_sub, tau_count)
        self.n_sub = None
        self.n_rx = None

    @override
    def csi_callback(self, new_csi: np.ndarray):
        n_sub, n_rx, n_tx = np.shape(new_csi)
        if self.n_sub is None:
            self.n_sub = n_sub
        if self.n_rx is None:
            self.n_rx = n_rx
        # treat each tx as a separate reading
        for tx in range(n_tx):
            self.buffer.push(new_csi[:, :, tx])

    @override
    def evaluate(self) -> np.ndarray:
        C = self.buffer.asarray()  # (n_sub, n_rx, buffer_size)
        # n_sub first principal components; one for each (n_rx, buffer_size) matrix
        u, _, _ = np.linalg.svd(C)  # (n_sub, n_rx, n_rx)
        X = u[:, :, 0]  # (n_sub, n_rx)
        # compute (theta_count, tau_count) profile
        return np.abs(self.A.conj().T @ X.T @ self.B)


class Music(Algorithm):
    """1D MUSIC algorithm. Only uses a single subcarrier."""

    name = "music"

    def __init__(self, params: Params, channel, bandwidth):
        super().__init__(params, channel, bandwidth)
        self.buffer = CircularBuffer(maxlen=params.buffer_size)
        self.A = self.aoa_steering_vector()  # (n_rx, theta_count)
        self.n_sub = None
        self.n_rx = None

    @override
    def csi_callback(self, new_csi):
        n_sub, n_rx, n_tx = np.shape(new_csi)
        if self.n_sub is None:
            self.n_sub = n_sub
        if self.n_rx is None:
            self.n_rx = n_rx
        for tx in range(n_tx):
            self.buffer.push(new_csi[:, :, tx])

    @override
    def evaluate(self):
        C = np.swapaxes(self.buffer.asarray(), 0, 1)  # (n_rx, n_sub, k)
        R = np.cov(np.reshape(C, (self.n_rx, -1), order="F"))
        # pick L eigenvectors of noise space
        e, v = np.linalg.eigh(R)
        indices = e < 0.1 * np.max(e)
        E = v[:, indices]  # (n_rx, L)
        # compute profile
        profile = np.zeros(self.params.theta_count)
        for i in range(self.params.theta_count):
            a = self.A[:, i]  # (n_rx)
            denominator = np.real(a.conj().T @ E @ E.conj().T @ a)
            profile[i] = 1 / np.maximum(denominator, SMALLEST_NORMAL)
        return np.reshape(np.atleast_2d(profile), (self.params.theta_count, 1))


class Conventional(Algorithm):
    """Conventional beamforming, also known as the FFT method."""

    name = "conventional"

    @override
    def csi_callback(self, new_csi):
        raise NotImplementedError()

    @override
    def evaluate(self):
        raise NotImplementedError()


class Capon(Algorithm):
    """Capon beamforming, also known as Minimum Variance Distortionless Response."""

    name = "capon"

    def __init__(self, params: Params, channel, bandwidth):
        super().__init__(params, channel, bandwidth)
        self.buffer = CircularBuffer(maxlen=params.buffer_size)
        self.A = self.aoa_steering_vector()  # (n_rx, theta_count)
        self.n_sub = None
        self.n_rx = None

    @override
    def csi_callback(self, new_csi):
        n_sub, n_rx, n_tx = np.shape(new_csi)
        if self.n_sub is None:
            self.n_sub = n_sub
        if self.n_rx is None:
            self.n_rx = n_rx
        for tx in range(n_tx):
            self.buffer.push(new_csi[:, :, tx])

    @override
    def evaluate(self):
        C = np.swapaxes(self.buffer.asarray(), 0, 1)  # (n_rx, n_sub, k)
        R = np.cov(np.reshape(C, (self.n_rx, -1), order="F"))
        # use least_squares solve for stability when computing profile
        profile = np.zeros(self.params.theta_count)
        for i in range(self.params.theta_count):
            a = self.A[:, i]  # (n_rx)
            x, *_ = np.linalg.lstsq(R, a, rcond=None)
            profile[i] = 1 / np.real(a.conj().T @ x)
        return np.reshape(np.atleast_2d(profile), (self.params.theta_count, 1))
