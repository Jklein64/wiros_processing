from __future__ import annotations

import rospy
from aoa_node.params import Params
from common.processing import array_from_wifi_message
from rf_msgs.msg import Bearing, Profile1D, Profile2D, Wifi
from rospy import Publisher, ROSInterruptException, Subscriber


class AoaNode:
    def __init__(self, params: Params):
        self.params = params
        self.bearing_pub = Publisher("/bearing", Bearing, queue_size=3)

        if params.profile_type in {"1D", "both"}:
            self.profile1D_pub = Publisher("/profile-1D", Profile1D, queue_size=3)
        if params.profile_type in {"2D", "both"}:
            self.profile2D_pub = Publisher("/profile-2D", Profile2D, queue_size=3)

    def csi_callback(self, msg: Wifi):
        # extract csi matrix
        csi = array_from_wifi_message(msg)

        # calculate AoA and profile

        # publish bearing

        # publish 2D profile

        # publish 1D profile


if __name__ == "__main__":
    try:
        rospy.init_node("aoa_node", anonymous=True)

        aoa_node = AoaNode(Params())
        Subscriber("/csi", Wifi, aoa_node.csi_callback, queue_size=1)
        # continuously handle new csi data
        rospy.spin()

    except ROSInterruptException:
        pass
