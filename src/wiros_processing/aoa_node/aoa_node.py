from __future__ import annotations

import numpy as np
import rospy
from rf_msgs.msg import Profile2d

from ..constants import SUBCARRIER_FREQUENCIES, SUBCARRIER_SPACING, C

COMPENSATION = np.load("/media/share/ros/bag/192.168.43.1-155.npy")


class CircularBuffer:
    def __init__(self, maxlen):
        self.maxlen = maxlen
        self.buffer = None
        self.index = 0

    def push(self, new_data):
        if self.buffer is None:
            shape = (*np.shape(new_data), self.maxlen)
            self.buffer = np.zeros(shape, dtype=np.complex128)
        self.buffer[..., self.index] = new_data
        self.index = (self.index + 1) % self.maxlen

    def asarray(self):
        arr = self.buffer.view()
        arr.flags.writeable = False
        return arr


class AoaNode:
    def __init__(self):
        self.params = AoaNode.Params()
        theta_min = self.params.theta_min
        theta_max = self.params.theta_max
        theta_count = self.params.theta_count
        tau_min = self.params.tau_min
        tau_max = self.params.tau_max
        tau_count = self.params.tau_count

        self.theta_samples = np.linspace(theta_min, theta_max, theta_count)
        self.tau_samples = np.linspace(tau_min, tau_max, tau_count)
        self.buffer = CircularBuffer(maxlen=20)
        self.profile_pub = rospy.Publisher("profile_2d", Profile2d, queue_size=10)

    def raw_csi_callback(self, raw_csi):
        # raw_csi is n_sub x n_rx x n_tx
        csi = np.copy(raw_csi)
        csi[:64] *= -1

        csi = csi[SUBCARRIER_SPACING[80e6] + 128]

        csi[117] = csi[118]

        for tx in range(csi.shape[2]):
            hpk = csi[:, :, tx]
            line = np.polyfit(
                SUBCARRIER_FREQUENCIES[80e6], np.unwrap(np.angle(hpk), axis=0), 1
            )
            tch = np.min(line[0, :])
            subc_angle = np.exp(-1.0j * tch * SUBCARRIER_FREQUENCIES[80e6])
            csi[:, :, tx] = hpk * subc_angle[:, np.newaxis]

        csi *= COMPENSATION
        return csi

    def csi_callback(self, msg):
        n_sub = msg.n_sub
        n_rows = msg.n_rows
        n_cols = msg.n_cols
        # extract csi matrix
        csi_real = np.reshape(msg.csi_real, (n_sub, n_rows, n_cols), order="F")
        csi_imag = np.reshape(msg.csi_imag, (n_sub, n_rows, n_cols), order="F")
        csi = csi_real + 1.0j * csi_imag.astype(np.complex128)
        csi = self.raw_csi_callback(csi)

        n_sub, n_rx, n_tx = np.shape(csi)
        A = self.aoa_steering_vector()  # (n_rx, len(theta_samples))
        B = self.tof_steering_vector()  # (n_sub, len(theta_samples))
        # # treat each tx as if it was a separate measurement
        for tx in range(n_tx):
            self.buffer.push(np.reshape(csi[:, :, tx], (-1,), order="F"))
        X = self.buffer.asarray()  # (n_sub * n_rx, n_data)
        # use first principal component of csi measurement autocorrelation
        u, _, _ = np.linalg.svd(X)
        csi = np.reshape(u[:, 0], (n_sub, n_rx), order="F")

        profile = np.abs(A.conj().T @ csi.conj().T @ B)
        profile_msg = Profile2d(
            header=rospy.Header(stamp=rospy.Time.now()),
            theta_count=self.params.theta_count,
            theta_min=self.params.theta_min,
            theta_max=self.params.theta_max,
            tau_count=self.params.tau_count,
            tau_min=self.params.tau_min,
            tau_max=self.params.tau_max,
            intensity=np.ravel(profile),
        )
        self.profile_pub.publish(profile_msg)

    def aoa_steering_vector(self, theta_samples=None):
        """
        Creates a steering vector for AoA with shape `(n_rx, len(theta_samples))`. If the `theta_samples` parameter is `None`, then this method uses samples determined by the parameters passed during initialization.
        """
        # channel is 155, bandwidth is 80
        freqs = 5e9 + 155 * 5e6 + SUBCARRIER_FREQUENCIES[80e6]
        # calculate wave number
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
        # channel is 155, bandwidth is 80
        freqs = 5e9 + 155 * 5e6 + SUBCARRIER_FREQUENCIES[80e6]
        # f_delta = freqs - freqs[0]
        omega = 2 * np.pi * freqs / C
        tau_samples = self.tau_samples if tau_samples is None else tau_samples
        tau_samples = np.atleast_1d(tau_samples)
        # column j corresponds to steering vector for the j'th tau sample
        # B = np.exp(-1.0j * np.outer(2 * np.pi * f_delta, tau_samples))
        B = np.exp(-1.0j * np.outer(omega, tau_samples))
        # B now has shape (n_sub, len(tau_samples))
        return B

    class Params:
        # algorithm parameters
        theta_min: float  # min value of theta/AoA samples (radians)
        theta_max: float  # max value of theta/AoA samples (radians)
        theta_count: int  # number of theta/AoA samples
        tau_min: float  # min value of tau/ToF samples (seconds)
        tau_max: float  # max value of tau/ToF samples (seconds)
        tau_count: int  # number of tau/ToF samples
        buffer_size: int  # number of elements to keep in the circular buffer
        rx_position: np.ndarray

        def __init__(self):
            self.theta_min = -np.pi / 2
            self.theta_max = np.pi / 2
            self.theta_count = 180
            self.tau_min = -10
            self.tau_max = 40
            self.tau_count = 100
            self.rx_position = np.array(
                [0, 0, 0, -0.026, 0, -0.052, 0, -0.078]
            ).reshape((-1, 2))
            self.buffer_size = 20
