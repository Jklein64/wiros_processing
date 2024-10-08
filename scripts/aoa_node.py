#! /usr/bin/env python3
import os
import sys

import numpy as np
import rospy
from csi_utils.aoa_node_main import AoaNode

EPS = np.finfo("float").eps


if __name__ == "__main__":
    rospy.init_node("aoa_node", anonymous=True)

    # create node wrapper
    aoa = AoaNode()

    # setup node options
    aoa.algo = rospy.get_param("~algo", "fft")

    COMP_PATH = rospy.get_param("~comp", None)
    do_comp = rospy.get_param("~compensate_channel", True)

    if not do_comp:
        rospy.logwarn("Turning off compensation.")
        COMP_PATH = None
    if COMP_PATH is not None:
        aoa.comp_path = COMP_PATH
        print(COMP_PATH)
        if os.path.isdir(COMP_PATH):
            aoa.comp = {}
            aoa.use_comp_folder = True
        elif os.path.isfile(COMP_PATH):
            aoa.comp = np.load(COMP_PATH)
            aoa.use_comp_folder = False
        else:
            print(
                f"fatal: compensation was turned on, but the provided path {COMP_PATH} does not exist."
            )
            sys.exit(1)

    aoa.rx_position = np.asarray(rospy.get_param("~rx_position", None)).reshape((-1, 2))
    if aoa.rx_position is None:
        rospy.logfatal(
            "The parameter rx_position was not set. Please set it with list of 8 floats as detailed in antennas.md"
        )

    compensate_channel = rospy.get_param("~compensate_channel", False)
    aoa.apply_nts = rospy.get_param("~correct_tof_offset", False)

    aoa.pub_prof = rospy.get_param("~publish_profile", True)
    aoa.pub_channel = rospy.get_param("~publish_channel", False)
    aoa.pub_rel_channel = rospy.get_param("~relative_phase", False)
    aoa.use_color = rospy.get_param("~color_profile", True)
    aoa.prof_tx_id = rospy.get_param("~profile_tx_id", 0)
    aoa.chan_tx_id = rospy.get_param("~channel_tx_id", 0)

    aoa.num_theta_steps = rospy.get_param("~num_theta_steps", 180)
    aoa.num_d_steps = rospy.get_param("~num_d_steps", 100)
    aoa.pkt_smoothing_window = rospy.get_param("~smoothing_window", 40)

    aoa.theta_thresh = rospy.get_param("~theta_thresh", [-180, 180])
    aoa.d_thresh = rospy.get_param("~d_thresh", [-10, 40])

    aoa.rssi_threshold = rospy.get_param("~rssi_thresh", -65)
    # Will use all TX if left as None
    aoa.valid_tx_ant = rospy.get_param("~valid_tx_ant", None)

    # start node
    aoa.start()
