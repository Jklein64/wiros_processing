_Part of the [WiROS](https://github.com/Jklein64/wiros) Wifi sensing toolbox_

# wiros_processing

ROS package providing utilities for CSI data correction and calculation of Angle-of-Arrival (AoA) from input CSI messages.

This package contains two nodes:

- `aoa_node.py` takes in input CSI from the `/csi` topic and publishes resulting AoA as a [`Bearing`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Bearing.msg) message to the `/bearing` topic, as well as optionally publishing 1d and 2d AoA/ToF profiles as [`Profile1d`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Profile1d.msg) and [`Profile2d`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Profile2d.msg) messages to `/profile1d` and `/profile2d` respectively

- `correction_node.py` applies compensation and corrects phase shifts in raw CSI data from `/csi_raw`, republishing a [`Wifi`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Wifi.msg) message to `/csi`

These nodes support arbitrary (planar) antenna array configurations allowing for linear, square, etc. array shapes. Several AoA-estimation algorithms including spotfi, 2d-fft, and some averaging-based expansions of 2d-fft are present, and [`algorithm.py`](https://github.com/Jklein64/wiros_processing/blob/main/src/aoa_node/algorithm.py) is written in such a way that it is very easy to add new ones.

## Computing AoA

All of the methods we use to compute AoA have the same fundamental idea, which is to compare the space of steering vectors with the received channel, sort of like beamforming in reverse. We provide several algorithms out-of-the-box for different compute/accuracy/time-averaging considerations.

## Parameters

### AoA Node

- `algo` : The algorithm to use. Provided are:

  - `aoa_only`
  - `fft`
  - `rx_svd`
  - `full_svd`
  - `music`
  - `spotfi`

- `theta_min`, `theta_max`, `theta_count` : the AoA values to search over.
- `tau_min`, `tau_max`, `tau_count` : the ToF values to search over.
- `profile_type` : One of `"1D"`, `"2D"`, `"both"`, or `"none"`. The type of profile or profiles to publish. Defaults to `"both"`.
- `rx_position` : The position of the receiver antennas in the coordinate frame you wish to measure in. AoA is measured going counter-clockwise from the positive x-axis. Typically you would put the first antenna at the origin. More explanations as well as some example antenna array setups can be found in [antennas.md](antennas.md). Also note that SpotFi assumes standard uniform linear array, as the CSI-Smoothing algorithm isn't well-defined for other array shapes.

### Correction Node

- `rssi_threshold` : Only keep CSI for frames where RSSI is above this limit. Filters out poor-quality measurements.

- `compensation_array` : Optional path to a [compensation file](#collecting-compensation-data). These are channel-specific files containing calibration values for the measured phase. Compensation files should be stored in the format `{receiver IP}-{channel number}.npy`, e.g. `192.168.1.1-36.npy`. This repo only supports using one channel's compensation file at a time.

- `tof_offset_correction` : Try to shift TOF to 0 by fitting a complex exponential to subcarrier phase and then dividing it out. Helps remove CFO, NTS offset.

## Collecting Compensation Data

In order to accurately measure AoA, we need the relative phase between different pairs of antennas. However, due to differing wire lengths as well as filter effects from the onboard RF circuitry, each RX chain will have a phase bias which varies per subcarrier and per channel. So in order to accurately measure AoA, these biases need to be measured and removed from incoming data. The necessary phases to remove these biases can then be passed in the `comp` parameter and will be applied to incoming CSI by the ROS node.

The static compensation method provides very clean results but requires a [4-to-1 power splitter](https://www.minicircuits.com/pdfs/ZN4PD-642W+.pdf), 3 SMA 50-ohm terminators, about 55dB of attenuators, and a second AC86u.

The simplest way to measure the phase bias at the receiver is to ensure that the phase at each receiver is identical and measure the resulting phase. The idea is to ensure this is the case by injecting exactly the same signal to each receiver.

1. Connect the 4 receiver ports of the AC86u to the output of the power splitter. Ensure that you have the same length of cable between the board and each splitter port as you normally would between the board and each antenna.

2. Connect the attenuators to the input of the splitter, and one of the outputs of the other AC86u to the attenuators. Ideally you should terminate the other 3 antenna ports to cancel any crosstalk.

3. Start the [csi_node](https://github.com/ucsdwcsng/wiros_csi) at both the transmitter and receiver ends. The transmitter should have packet injection turned on and the receiver should set its MAC filter to the address the transmitter is injecting with.

4. Save the CSI data measured to a bag file, via `rosbag record /csi -O compensation_csi.bag`

5. Use the rosbag processing script to create the compensation file:
   ```
   rosrun wiros_processing generate_static_compensation.py BAG_PATH
   ```

Compensation files follow the naming convention `{IP}-{chanspec}.npy`. NOTE: The above file conjugates the received CSI matrix.

## Algorithms

Several algorithms are provided, more will be added in the future.

### aoa_only

Stacks all transmitters and subcarriers over the last 8 packets and takes SVD. This emulates a channel with 4 receivers and very many transmitters providing a good U matrix at low compute. Provides AoA only and does not publish a profile

### fft

Standard 2D-FFT. Takes median AoA across the transmitters and optionally returns the profile of the last transmitter.

### rx_svd

Takes the first principle component over the last 8 packets of each 4x32 submatrix across subcarriers, giving a 234x4x1 channel and computes fft. This provides a best trade-off between accuracy and compute efficiency. For more details, see [2]

### full_svd

Stacks receivers across subcarriers and takes first principle component over the last 8 packets of the 936x32 matrix, giving a 936x1 = 234x4x1 channel and computes fft.

### music

Stacks receivers across subcarriers and computes the 936 x 935 null space, then computes the reciprocal of the projection of the steering vectors onto the channel null space. Very slow.

### spotfi [1]

Performs super-resolution (increasing the rank of the measurements matrix) across antennas and subcarriers to better resolve multipath components of a signal from the direct path. Additionally, performs MUSIC on the super-resolved measurements. NOTE: This is a compute intensive method unsuitable for real-time operations and does not work for non-linear antenna arrays.

## Citations:

1. Kotaru, Manikanta, et al. "Spotfi: Decimeter level localization using wifi." Proceedings of the 2015 ACM Conference on Special Interest Group on Data Communication. 2015.
2. "WAIS: Leveraging WiFi for Online and Resource-Efficient SLAM"
3. Blanco, Alejandro, et al. "Accurate ubiquitous localization with off-the-shelf ieee 802.11 ac devices." The 19th Annual International Conference on Mobile Systems, Applications, and Services (MobiSys 2021). 2021.
