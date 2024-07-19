from __future__ import annotations

import numpy as np
from rf_msgs.msg import Wifi


def array_from_wifi_message(msg: Wifi):
    """
    Converts the given ROS `Wifi` message `msg` into a numpy array with shape `(n_sub, n_rx, n_tx)`
    """
    n_sub = msg.n_sub
    n_rows = msg.n_rows
    n_cols = msg.n_cols

    # extract csi matrix
    csi_real = np.reshape(msg.csi_real, (n_sub, n_rows, n_cols), order="F")
    csi_imag = np.reshape(msg.csi_imag, (n_sub, n_rows, n_cols), order="F")
    csi = csi_real + 1.0j * csi_imag.astype(np.complex128)

    return csi
