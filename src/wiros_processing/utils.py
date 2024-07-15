from __future__ import annotations

import numpy as np
from rf_msgs.msg import Wifi

from .constants import SUBCARRIER_INDICES


def array_from_wifi_message(msg: Wifi):
    """
    Converts the given ROS `Wifi` message `msg` into a numpy array with shape `(n_sub, n_rx, n_tx)`
    """
    bw = msg.bw * 1e6  # msg.bw is in MHz
    n_sub = msg.n_sub
    n_rows = msg.n_rows
    n_cols = msg.n_cols

    # extract csi matrix (copied from WiROS code)
    csi = np.asarray(msg.csi_real) + 1.0j * np.asarray(msg.csi_imag)
    # note: the Wifi message format stores CSI data with a Fortran-style ordering instead of the
    # C-style ordering that numpy uses by default. As documented by np.isfortran, the transpose
    # of an array with Fortran-style ordering is an array with C-style ordering. By reversing
    # the order of the dimensions in the reshape and then taking the transpose, this line of
    # code will correctly read the Fortran-style ordered data in a way that allows for C-style
    # modification prevalent in numpy.
    csi = csi.reshape((n_cols, n_rows, n_sub)).T

    # only apply corrections to raw data
    n_sub_full = 2 ** (bw / 10e6)
    if n_sub == n_sub_full:
        # these subcarriers are rotated. need to find justification for why this is the case
        if bw == 80e6:
            csi[:64] *= -1
        if bw == 40e6:
            csi[:64] *= -1j

        # store only relevant subcarriers
        csi = csi[SUBCARRIER_INDICES[bw]]

        # unexplained correction factor
        if bw == 80e6:
            csi[117] = csi[118]

    return csi
