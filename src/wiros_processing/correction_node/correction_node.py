#! /usr/bin/env python3

import numpy as np
import rospy
from rf_msgs.msg import Wifi
from rospy import Publisher, ROSInterruptException, Subscriber
from std_msgs.msg import Header

from ..constants import SUBCARRIER_FREQUENCIES
from ..processing import array_from_wifi_message
from .correction_params import CorrectionParams


class CorrectionNode:
    def __init__(self, params: CorrectionParams):
        self.params = params
        self.csi_pub = Publisher("/csi", Wifi, queue_size=3)

    def csi_callback(self, msg: Wifi):
        # apply RSSI threshold
        if self.params.rssi_threshold is not None:
            if msg.rssi < self.params.rssi_threshold:
                return

        # convert message into a numpy array
        # csi has shape (n_sub, n_rx, n_tx)
        csi = array_from_wifi_message(msg)

        # apply compensation if it exists
        if (c := self.params.compensation_array) is not None:
            csi *= c

        # ToF correction
        if self.params.tof_offset_correction:
            # msg.bw is in MHz; we need it in Hz
            freqs = SUBCARRIER_FREQUENCIES[msg.bw * 1e6]
            for tx in range(csi.shape[2]):
                hpk = csi[:, :, tx]
                line = np.polyfit(freqs, np.unwrap(np.angle(hpk), axis=0), 1)
                tch = np.min(line[0, :])
                subc_angle = np.exp(-1.0j * tch * freqs)
                csi[:, :, tx] = hpk * subc_angle[:, np.newaxis]

        # republish as clean CSI
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.n_sub, msg.n_rows, msg.n_cols = np.shape(csi)
        msg.csi_real = np.ravel(np.real(csi))
        msg.csi_imag = np.ravel(np.imag(csi))
        self.csi_pub.publish(msg)


def main():
    try:
        rospy.init_node("csi_clean_node", anonymous=True)

        correction_node = CorrectionNode(CorrectionParams())
        Subscriber("/csi_raw", Wifi, correction_node.csi_callback, queue_size=1)
        # continuously handle new csi data
        rospy.spin()

    except ROSInterruptException:
        pass
