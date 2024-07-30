import numpy as np
import rospy
from rf_msgs.msg import Csi, Rssi, Wifi
from rospy import Publisher
from std_msgs.msg import Header

from ..constants import USABLE_SUBCARRIER_INDICES
from .algorithm import Algorithm


class CorrectionNode:
    """ROS node to apply compensation and correction to incoming raw CSI data.

    Attributes:
        params: Paramaters for the node.
        csi_pub: Publisher for corrected CSI data.
    """

    def __init__(self):
        self.params = Params()
        self.algo = Algorithm.from_name(self.params.algo)
        self.csi_pub = Publisher("csi", Csi, queue_size=1000)
        self.rssi_pub = Publisher("rssi", Rssi, queue_size=1000)

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

        csi = csi[USABLE_SUBCARRIER_INDICES]

        if bandwidth == 80e6:
            csi[117] = csi[118]

        # compensation
        if self.compensation_array is not None:
            csi *= self.compensation_array

        # ToF correction
        self.algo.correct(csi)

        # republish as clean CSI
        csi_msg = Csi(header=Header(stamp=rospy.Time.now()))
        csi_msg.bandwidth = msg.bw
        csi_msg.n_sub, csi_msg.n_rx, csi_msg.n_tx = np.shape(csi)
        csi_msg.csi_real = np.reshape(np.real(csi), (-1), order="F")
        csi_msg.csi_imag = np.reshape(np.imag(csi), (-1), order="F")
        self.csi_pub.publish(csi_msg)

        # also publish RSSI
        rssi_msg = Rssi(header=Header(stamp=rospy.Time.now()))
        rssi_msg.rssi = msg.rssi
        self.rssi_pub.publish(rssi_msg)


class Params:
    """Parameters for the CorrectionNode.

    Attributes:
        compensation_path: File path to a static compensation file.
        rssi_threshold: Messages with RSSI below this will be dropped.
    """

    def __init__(self):
        self.algo = rospy.get_param("~algo", "ma")
        self.compensation_path = rospy.get_param("~compensation_path")
        self.rssi_threshold = rospy.get_param("~rssi_threshold", None)
