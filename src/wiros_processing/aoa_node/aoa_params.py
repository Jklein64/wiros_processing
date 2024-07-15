from __future__ import annotations

import numpy as np
import rospy


class AoaParams:
    # algorithm parameters
    algo: str  # name of algorithm to use
    theta_min: float  # min value of theta/AoA samples (radians)
    theta_max: float  # max value of theta/AoA samples (radians)
    theta_count: int  # number of theta/AoA samples
    tau_min: float  # min value of tau/ToF samples (seconds)
    tau_max: float  # max value of tau/ToF samples (seconds)
    tau_count: int  # number of tau/ToF samples

    # device parameters
    rx_position: np.ndarray

    # output parameters
    profile_type: "1D" | "2D" | "both" | "none"

    def __init__(self):
        self.algo = rospy.get_param("~algo")
        self.theta_min = rospy.get_param("~theta_min", -np.pi)
        self.theta_max = rospy.get_param("~theta_max", np.pi)
        self.theta_count = rospy.get_param("~theta_count", 180)
        self.tau_min = rospy.get_param("~tau_min", -10)
        self.tau_max = rospy.get_param("~tau_max", 40)
        self.tau_count = rospy.get_param("~tau_count", 100)
        self.rx_position = np.asarray(rospy.get_param("~rx_position")).reshape((-1, 2))
        self.profile_type = rospy.get_param("~profile_type", "both")
