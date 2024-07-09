from __future__ import annotations

import rospy
from rf_msgs.msg import Bearing, Profile1D, Profile2D, Wifi
from rospy import Publisher, ROSInterruptException, Subscriber

from .params import Params


class AoaNode:
    def __init__(self, params: Params):
        self.params = params
        self.csi_pub = Publisher("/csi", Wifi, queue_size=3)
        self.bearing_pub = Publisher("/bearing", Bearing, queue_size=3)

        if params.profile_type in {"1D", "both"}:
            self.profile1D_pub = Publisher("/profile-1D", Profile1D, queue_size=3)
        if params.profile_type in {"2D", "both"}:
            self.profile2D_pub = Publisher("/profile-2D", Profile2D, queue_size=3)

    def csi_callback(self, msg: Wifi):
        # enforce RSSI threshold
        if self.params.rssi_threshold:
            if msg.rssi < self.params.rssi_threshold:
                return
        # apply compensation
        # publish compensated CSI
        # calculate AoA and profile
        # publish bearing
        # publish 2D profile
        # publish 1D profile


if __name__ == "__main__":
    try:
        rospy.init_node("aoa_node", anonymous=True)

        aoa_node = AoaNode(Params())
        Subscriber("/csi_raw", Wifi, aoa_node.csi_callback, queue_size=1)
        # continuously handle new csi data
        rospy.spin()

    except ROSInterruptException:
        pass
