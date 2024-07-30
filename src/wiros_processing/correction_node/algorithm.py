import numpy as np
from typing_extensions import override

from ..constants import F_DELTA, USABLE_SUBCARRIER_INDICES


class Algorithm:
    """Implementation of a phase offset correction algorithm. See README for more.

    Attributes:
        name: The name of the algorithm as referenced as a ROS parameter.
    """

    name: str

    def correct(self, csi):
        """Corrects the phase offsets of the given CSI frame in-place.

        Params:
            csi: the CSI frame to correct, with shape (n_sub, n_rx, n_tx)
        """
        raise NotImplementedError()

    @classmethod
    def from_name(cls, name):
        """Instantiates an algorithm with the given name.

        Args:
            name: name of the algorithm as mentioned in the README.
        """
        for subclass in cls.__subclasses__():
            if subclass.name == name:
                return subclass()
        raise NameError(f'Could not find an algorithm with name "{name}"')


class SpotfiAlgorithm(Algorithm):
    """SpotFi ToF correction algorithm applied across transmitters."""

    name = "spotfi"

    @override
    def correct(self, csi):
        n_sub, n_rx, n_tx = np.shape(csi)
        for tx in range(n_tx):
            theta = np.unwrap(np.angle(csi[:, :, tx]), axis=0)
            b_hat = np.reshape(theta, (n_sub * n_rx,), order="F")
            phase_constant = -2 * np.pi * np.arange(n_sub) * F_DELTA
            A = np.stack([np.ones(n_sub), phase_constant], axis=1)
            A_hat = np.tile(A, reps=(n_rx, 1))
            _, tau = np.linalg.solve(A_hat.T @ A_hat, A_hat.T @ b_hat)
            # apply per-subcarrier phase adjustment
            csi[..., tx] *= np.expand_dims(np.exp(1j * -phase_constant * tau), axis=1)


class WirosAlgorithm(Algorithm):
    """ToF correction algorithm as originally implemented by WiROS."""

    name = "wiros"

    @override
    def correct(self, csi):
        freqs = (USABLE_SUBCARRIER_INDICES - 128) * F_DELTA
        for tx in range(csi.shape[2]):
            hpk = csi[:, :, tx]
            line = np.polyfit(freqs, np.unwrap(np.angle(hpk), axis=0), 1)
            tch = np.min(line[0, :])
            subc_angle = np.exp(-1j * tch * freqs)
            csi[:, :, tx] = hpk * subc_angle[:, np.newaxis]


class MaAlgorithm(Algorithm):
    """Phase offset correction algorithm from review paper."""

    name = "ma"

    @override
    def correct(self, csi):
        n_sub, n_rx, n_tx = np.shape(csi)
        # make theta have shape (n_tx, n_sub, n_rx) for proper flattening
        theta = np.unwrap(np.angle(csi), axis=0).transpose(2, 0, 1)
        b_hat = np.reshape(theta, (n_sub * n_rx * n_tx,), order="F")
        A = np.zeros((n_sub, n_tx, n_tx + 2))
        for k in range(n_sub):
            phase_constant = -2 * np.pi * k * F_DELTA
            A[k, :, 0] = 1
            A[k, :, 1] = phase_constant
            r, c = np.diag_indices(n_tx)
            A[k, r, 2 + c] = phase_constant
        A_hat = np.tile(np.concatenate(A, axis=0), reps=(n_rx, 1))
        (_, omega, *tau) = np.linalg.solve(A_hat.T @ A_hat, A_hat.T @ b_hat)
        # apply per-subcarrier and per-transmitter phase adjustment
        for tx in range(n_tx):
            for k in range(n_sub):
                phase = tau[tx] + omega
                csi[k, :, tx] *= np.exp(1j * 2 * np.pi * F_DELTA * k * phase)
