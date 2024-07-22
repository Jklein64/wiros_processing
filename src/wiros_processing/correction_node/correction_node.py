import numpy as np
import rospy
from rf_msgs.msg import Wifi
from rospy import Publisher
from std_msgs.msg import Header

from ..constants import SUBCARRIER_FREQUENCIES, SUBCARRIER_SPACING


class CorrectionNode:
    """ROS node to apply compensation and correction to incoming raw CSI data.

    Attributes:
        params: Paramaters for the node.
        csi_pub: Publisher for corrected CSI data.
    """

    def __init__(self):
        self.params = Params()
        self.csi_pub = Publisher("csi", Wifi, queue_size=1000)

        try:
            self.compensation_array = np.load(self.params.compensation_path)
        except (KeyError, OSError):
            rospy.logwarn(
                "Error loading compensation file. Compensation will not take place."
            )
            self.compensation_array = None

    def csi_callback(self, msg: Wifi):
        """Republishes the given raw Wifi message after applying correction."""

        # apply RSSI threshold
        if self.params.rssi_threshold is not None:
            if msg.rssi < self.params.rssi_threshold:
                return

        # extract raw csi matrix
        n_sub = msg.n_sub
        n_rows = msg.n_rows
        n_cols = msg.n_cols
        csi_real = np.reshape(msg.csi_real, (n_sub, n_rows, n_cols), order="F")
        csi_imag = np.reshape(msg.csi_imag, (n_sub, n_rows, n_cols), order="F")
        csi = csi_real + 1.0j * csi_imag.astype(np.complex128)

        # adjustments
        bandwidth = msg.bw * 1e6
        if bandwidth == 80e6:
            csi[:64] *= -1
        elif bandwidth == 40e6:
            csi[:64] *= -1j

        csi = csi[SUBCARRIER_SPACING[bandwidth] + 128]

        if bandwidth == 80e6:
            csi[117] = csi[118]

        # ToF correction
        freqs = SUBCARRIER_FREQUENCIES[bandwidth]
        for tx in range(csi.shape[2]):
            hpk = csi[:, :, tx]
            line = np.polyfit(freqs, np.unwrap(np.angle(hpk), axis=0), 1)
            tch = np.min(line[0, :])
            subc_angle = np.exp(-1.0j * tch * freqs)
            csi[:, :, tx] = hpk * subc_angle[:, np.newaxis]

        # compensation
        if self.compensation_array is not None:
            csi *= self.compensation_array

        # republish as clean CSI
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.n_sub, msg.n_rows, msg.n_cols = np.shape(csi)
        msg.csi_real = np.reshape(np.real(csi), (-1), order="F")
        msg.csi_imag = np.reshape(np.imag(csi), (-1), order="F")
        self.csi_pub.publish(msg)


class Params:
    """Parameters for the CorrectionNode.

    Attributes:
        compensation_path: File path to a static compensation file.
        rssi_threshold: Messages with RSSI below this will be dropped.
    """

    def __init__(self):
        self.compensation_path = rospy.get_param("~compensation_path")
        self.rssi_threshold = rospy.get_param("~rssi_threshold", None)
