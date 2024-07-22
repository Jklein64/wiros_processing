from __future__ import annotations

import numpy as np
import rospy
from rf_msgs.msg import Bearing, Profile1d, Profile2d

from .algorithm import Algorithm


class AoaNode:
    """ROS node to extract AoA and ToF information from incoming CSI data.

    Attributes:
        params: Parameters for the node.
        algo_instances: A dictionary mapping (channel, bw) to algo instances
        bearing_pub: ROS publisher for the bearing topic
        profile1d_pub: ROS publisher for the profile1d topic
        profile2d_pub: ROS publisher for the profile2d topic
        last_msg: Reference to the last Wifi message for timer callback
        last_msg: Reference to the last (channel, bw) for timer callback
    """

    def __init__(self):
        self.params = Params()
        self.algo_instances: dict[tuple[int, int], Algorithm] = {}
        self.bearing_pub = rospy.Publisher("bearing", Bearing, queue_size=1000)
        if self.params.profiles in {1, 3}:
            self.profile1d_pub = rospy.Publisher("profile1d", Profile1d, queue_size=10)
        if self.params.profiles in {2, 3}:
            self.profile2d_pub = rospy.Publisher("profile2d", Profile2d, queue_size=10)
        self.last_msg = None
        self.last_ch_bw = None

    def csi_callback(self, msg):
        """Reads incoming Wifi messages and passes arrays to the algorithm.

        Args:
            msg: the incoming Wifi message with CSI data and shape.
        """
        self.last_msg = msg
        n_sub = msg.n_sub
        n_rows = msg.n_rows
        n_cols = msg.n_cols
        # extract csi matrix
        csi_real = np.reshape(msg.csi_real, (n_sub, n_rows, n_cols), order="F")
        csi_imag = np.reshape(msg.csi_imag, (n_sub, n_rows, n_cols), order="F")
        csi = csi_real + 1.0j * csi_imag.astype(np.complex128)
        # instantiate algorithm for channel and bandwidth
        if (ch_bw := (msg.chan, msg.bw * 1e6)) not in self.algo_instances:
            self.algo_instances[ch_bw] = Algorithm.from_params(self.params, *ch_bw)
        # give algorithm instance the csi data
        self.algo_instances[ch_bw].csi_callback(csi)
        self.last_ch_bw = ch_bw

    def timer_callback(self, _: rospy.timer.TimerEvent):
        """Uses the selected algorithm to evaluate the AoA/ToF profile.

        The evaluation and publishing is determined by the rate parameter, and is
        separate from the rate at which the CSI data is received. This is to handle
        cases where the algorithm cannot run faster than 20-30 Hz.
        """
        profile = self.algo_instances[self.last_ch_bw].evaluate()

        if self.params.profiles in {1, 3}:
            # flatten 2d profile by picking most likely tau
            tau_index = np.argmax(np.sum(profile, axis=0))
            profile1d = profile[:, tau_index]

            profile1d_msg = Profile1d(
                header=rospy.Header(stamp=rospy.Time.now()),
                theta_count=self.params.theta_count,
                theta_min=self.params.theta_min,
                theta_max=self.params.theta_max,
                intensity=profile1d,
            )
            self.profile1d_pub.publish(profile1d_msg)

        if self.params.profiles in {2, 3}:
            profile2d_msg = Profile2d(
                header=rospy.Header(stamp=rospy.Time.now()),
                theta_count=self.params.theta_count,
                theta_min=self.params.theta_min,
                theta_max=self.params.theta_max,
                tau_count=self.params.tau_count,
                tau_min=self.params.tau_min,
                tau_max=self.params.tau_max,
                intensity=np.ravel(profile),
            )
            self.profile2d_pub.publish(profile2d_msg)

        # publish bearing
        profile_shape = (self.params.theta_count, self.params.tau_count)
        # pylint: disable-next=unbalanced-tuple-unpacking
        theta_index, _ = np.unravel_index(np.argmax(profile), profile_shape)
        bearing_msg = Bearing(
            header=rospy.Header(stamp=rospy.Time.now()),
            ap_id=self.last_msg.ap_id,
            txmac=self.last_msg.txmac,
            n_rx=self.last_msg.n_rows,
            n_tx=self.last_msg.n_cols,
            seq=self.last_msg.seq_num,
            rssi=self.last_msg.rssi,
            aoa=self.algo_instances[self.last_ch_bw].theta_samples[theta_index],
        )
        self.bearing_pub.publish(bearing_msg)


class Params:
    """Parameters for the AoaNode.

    Attributes:
        theta_min: Min value of theta/AoA samples (radians).
        theta_max: Max value of theta/AoA samples (radians).
        theta_count: Number of theta/AoA samples.
        tau_min: Min value of tau/ToF samples (seconds).
        tau_max: Max value of tau/ToF samples (seconds).
        tau_count: Number of tau/ToF samples.
        buffer_size: Size of circular CSI buffer.
        rate: Target processing/publish rate (Hz).
        algo: Name of the direction-finding algorithm to use.
        profiles: Which, if any, of 1D and 2D profiles to compute.
    """

    def __init__(self):
        # algorithm
        self.theta_min = rospy.get_param("~theta_min", -np.pi / 2)
        self.theta_max = rospy.get_param("~theta_max", np.pi / 2)
        self.theta_count = rospy.get_param("~theta_count", 180)
        self.tau_min = rospy.get_param("~tau_min", -10)
        self.tau_max = rospy.get_param("~tau_max", 40)
        self.tau_count = rospy.get_param("~tau_count", 100)
        self.buffer_size = rospy.get_param("~buffer_size", 20)
        self.rx_position = np.array(rospy.get_param("~rx_position"))

        # ros
        self.rate = rospy.get_param("~rate", 20)
        self.algo = rospy.get_param("~algo", "full_svd")
        self.profiles = rospy.get_param("~profiles", 3)
