import numpy as np
import rospy
from rf_msgs.msg import Wifi
from rospy import Publisher
from std_msgs.msg import Header

from ..utils import array_from_wifi_message
from .correction_params import CorrectionParams


class CorrectionNode:
    def __init__(self):
        self.params = CorrectionParams()
        self.csi_pub = Publisher("csi", Wifi, queue_size=1)

    def csi_callback(self, msg: Wifi):
        # apply RSSI threshold
        if self.params.rssi_threshold is not None:
            if msg.rssi < self.params.rssi_threshold:
                return

        # convert message into a numpy array
        # csi has shape (n_sub, n_rx, n_tx)
        csi = array_from_wifi_message(msg)

        # bw = msg.bw * 1e6  # msg.bw is in MHz
        # these subcarriers are rotated. need to find justification for why this is the case
        # if bw == 80e6:
        csi[:64] *= -1
        # if bw == 40e6:
        #     csi[:64] *= -1j

        # store only relevant subcarriers
        subcarrier_spacing = np.array(
            sorted(
                set(range(-122, 123)) - {-103, -75, -39, -11, -1, 0, 1, 11, 39, 75, 103}
            )
        )
        subcarrier_frequencies = subcarrier_spacing * 312500
        csi = csi[subcarrier_spacing + 128]
        # csi = csi[SUBCARRIER_INDICES[bw]]

        # unexplained correction factor
        # if bw == 80e6:
        csi[117] = csi[118]

        # ToF correction
        if self.params.tof_offset_correction:
            # msg.bw is in MHz; we need it in Hz
            freqs = subcarrier_frequencies  # SUBCARRIER_FREQUENCIES[msg.bw * 1e6]
            for tx in range(csi.shape[2]):
                hpk = csi[:, :, tx]
                line = np.polyfit(freqs, np.unwrap(np.angle(hpk), axis=0), 1)
                tch = np.min(line[0, :])
                subc_angle = np.exp(-1.0j * tch * freqs)
                csi[:, :, tx] = hpk * subc_angle[:, np.newaxis]

        # apply compensation if it exists
        if (c := self.params.compensation_array) is not None:
            csi *= c

        # republish as clean CSI
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.n_sub, msg.n_rows, msg.n_cols = np.shape(csi)
        msg.csi_real = np.ravel(np.real(csi))
        msg.csi_imag = np.ravel(np.imag(csi))
        self.csi_pub.publish(msg)
