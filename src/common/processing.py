from __future__ import annotations

import numpy as np
from common.constants import SUBCARRIER_INDICES
from rf_msgs.msg import Wifi


def array_from_wifi_message(msg: Wifi):
    """
    Converts the given ROS `Wifi` message `msg` into a numpy array with shape `(n_sub, n_rows, n_cols)`, where the first dimension corresponds to receivers and the second to transmitters.
    """
    bw = msg.bw * 1e6  # msg.bw is in MHz
    n_sub = msg.n_sub
    n_rows = msg.n_rows
    n_cols = msg.n_cols
    csi_shape = (n_sub, n_rows, n_cols)
    # extract raw csi matrix
    csi_real = np.array(msg.csi_real).reshape(csi_shape)
    csi_imag = np.array(msg.csi_imag).reshape(csi_shape)
    csi = csi_real + 1j * csi_imag.astype(np.complex128)

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
