from __future__ import annotations

import numpy as np
import rospy
from rf_msgs.msg import Bearing, Profile1d, Profile2d, Wifi
from rospy import Publisher
from std_msgs.msg import Header

from ..utils import array_from_wifi_message
from .algorithm import Algorithm
from .aoa_params import AoaParams


class AoaNode:
    def __init__(self):
        self.params = AoaParams()
        self.bearing_pub = Publisher("bearing", Bearing, queue_size=3)

        self.publish_profile1d = False
        self.publish_profile2d = False

        if self.params.profile_type in {"1D", "both"}:
            self.profile1d_pub = Publisher("profile_1d", Profile1d, queue_size=3)
            self.publish_profile1d = True
        if self.params.profile_type in {"2D", "both"}:
            self.profile2d_pub = Publisher("profile_2d", Profile2d, queue_size=3)
            self.publish_profile2d = True

        # maps (channel, bandwidth) tuples to algorithm instances
        self.algo_instances: dict[tuple[int, float], Algorithm] = {}

    def csi_callback(self, msg: Wifi):
        # extract csi matrix
        csi = array_from_wifi_message(msg)

        # calculate AoA and profile
        if (ch_bw := (msg.chan, msg.bw)) not in self.algo_instances:
            self.algo_instances[ch_bw] = Algorithm.from_params(self.params, *ch_bw)
        theta, profile = self.algo_instances[ch_bw].evaluate(csi)

        # publish bearing
        bearing_msg = Bearing(
            header=Header(stamp=rospy.Time.now()),
            ap_id=msg.ap_id,
            txmac=msg.txmac,
            n_rx=msg.n_rows,
            n_tx=msg.n_cols,
            seq=msg.seq_num,
            rssi=msg.rssi,
            aoa=theta,
        )
        self.bearing_pub.publish(bearing_msg)

        # publish 2D profile
        if self.publish_profile2d:
            if len(np.squeeze(np.shape(profile))) <= 1:
                rospy.logwarn(
                    "Failed to publish a 2d profile because data did not have 2 meaningful dimensions."
                )
            else:
                profile2d_msg = Profile2d(
                    header=Header(stamp=rospy.Time.now()),
                    theta_count=self.params.theta_count,
                    theta_min=self.params.theta_min,
                    theta_max=self.params.theta_max,
                    tau_count=self.params.tau_count,
                    tau_min=self.params.tau_min,
                    tau_max=self.params.tau_max,
                    intensity=np.ravel(profile),
                )
                self.profile2d_pub.publish(profile2d_msg)

        # publish 1D profile
        # warn if the node is taking too long during processing
        self._time_stop = perf_counter()
        self._elapsed.append(self._time_stop - self._time_start)
        avg_elapsed = np.mean(self._elapsed)
        if avg_elapsed > (1 / 30):
            rospy.logwarn(
                f"CSI data is at ~20-30 Hz, but aoa_node is at ~{1/avg_elapsed:.2f} Hz"
            )
