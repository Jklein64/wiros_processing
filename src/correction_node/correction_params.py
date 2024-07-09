from __future__ import annotations

import numpy as np
import rospy


class CorrectionParams:
    compensation_array: np.ndarray | None
    tof_offset_correction: bool
    rssi_threshold: int | None

    def __init__(self):
        try:
            compensation_path = rospy.get_param("~compensation_path")
            self.compensation_array = np.load(compensation_path)
        except (KeyError, OSError):
            rospy.logwarn(
                "Error loading compensation file. Compensation will not take place."
            )
            self.compensation_array = None

        self.tof_offset_correction = rospy.get_param("~tof_offset_correction", True)
        self.rssi_threshold = rospy.get_param("~rssi_threshold", None)
