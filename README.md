_Part of the [WiROS](https://github.com/Jklein64/wiros) Wifi sensing toolbox_

# WiROS Processing

ROS package providing utilities for CSI data correction and calculation of Angle-of-Arrival (AoA) from input CSI messages.

This package contains two nodes:

- **aoa_node** takes in input CSI from the `/csi` topic and publishes 1d and 2d AoA/ToF profiles as [`Profile1d`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Profile1d.msg) and [`Profile2d`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Profile2d.msg) messages to `/profile1d` and `/profile2d` respectively

- **correction_node** applies compensation and corrects phase shifts in raw CSI data from `/csi_raw`, republishing the CSI and RSSI from incoming [`Wifi`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Wifi.msg) messages as [`Csi`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Csi.msg) and [`Rssi`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Rssi.msg) messages to `/csi` and `/rssi`

These nodes support arbitrary (planar) antenna array configurations allowing for linear, square, etc. array shapes, though some algorithms assume a linear arrangement. [algorithm.py](https://github.com/Jklein64/wiros_processing/blob/main/src/aoa_node/algorithm.py) is written in such a way that it is very easy to add new ones.

## Parameters

### AoA Node

- `algo` : The algorithm to use. See the [algorithms section](#algorithms).

- `theta_min`, `theta_max`, `theta_count` : the AoA values to search over.
- `tau_min`, `tau_max`, `tau_count` : the ToF values to search over.
- `profiles` : One of `0`, `1`, `2`, or `3`. The type of profile or profiles to publish. `1` and `2` are for 1d-only and 2d-only; `3` is both. Defaults to `3`.
- `rx_position` : The position of the receiver antennas in the coordinate frame you wish to measure in. AoA is measured going counter-clockwise from the positive x-axis. Typically you would put the first antenna at the origin. More explanations as well as some example antenna array setups can be found in [antennas.md](antennas.md).
- `buffer_size` : The size of the circular buffer for CSI data. Not used by all algorithms.
- `rate` : Target publishing rate. The processing is controlled by a timer running at this rate, and is separate from the rate at which data is published to `/csi`.

### Correction Node

- `rssi_threshold` : Only keep CSI for frames where RSSI is above this limit. Filters out poor-quality measurements.

- `compensation_array` : Optional path to a [compensation file](#collecting-compensation-data). These are channel-specific files containing calibration values for the measured phase. Compensation files should be stored in the format `{receiver IP}-{channel number}.npy`, e.g. `192.168.1.1-36.npy`. This repo only supports using one channel's compensation file at a time.

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

## Correction Algorithms

CSI data has a lot of artifacts from processes and errors that might throw off later algorithms. Ma et. al. (2019) model these as

$$H_{i, j, k} =
\underbrace{\left(\sum_n^N a_ne^{-j2\pi \frac{d_{i,j,n}}{c} f_k }\right)}_\text{Multi-Path Channel}\
\underbrace{e^{-j2\pi\tau_i f_k}}_\text{CSD}\
\underbrace{e^{-j2\pi \rho f_k}}_\text{STO}\
\underbrace{e^{-j2\pi \eta(f_k'/f_k-1)f_k}}_\text{SFO}\
\underbrace{q_{i,j}e^{-j2\pi \zeta_{i,j}}}_\text{Beamforming}$$

Where $H_{i,j,k}$ is the CSI data for transmitter $i$, receiver $j$, and subcarrier $k$, $a_n$ is the attenuation along the $n$'th path, $d_{i, j, n}$ is the distance of the $n$'th path from transmitter $i$ to receiver $j$, $f_k$ is the carrier frequency, $\tau_i$ is a time offset due to Cyclic Shift Diversity (CSD), $\rho$ is the Sampling Time Offset (STO), $\eta$ is the sampling frequency offset, and $q_{i,j}$ and $\zeta_{i, j}$ are attenuation and phase shift due to beamforming.

Note that most of these processing artifacts and errors cause phase shifts that vary linearly in frequency. The core idea behind the correction algorithms is to do linear regression on the phase of CSI data to extract the parameters.

TODO write up correction algorithms.

## Direction-Finding Algorithms

Here we will describe the functioning concepts of the direction-finding algorithms. For simplicity, the derivations here assume a linear antenna array and an OFDM signal on channel 155 with 80 MHz bandwidth, though the reasoning extends to arbitrary planar arrays and other OFDM channels and bandwidths.

We know the speed of light $c \approx 3\cdot 10^8$ m/s, the carrier frquency $f_c = 5.775$ GHz, and the spacing between adjacent subcarriers $f_\delta = 312.5$ KHz. Suppose that we have a planar array of $M$ receiving antennas $1, \ldots, M$ for an OFDM signal with $N$ subcarriers. Let $r_m \in \mathbb{R}^2$ be the displacement of antenna $m$ from antenna $1$, and let $\theta \in [\theta_\text{min}, \theta_\text{max}]$ be the angle of attack (AoA) of an incoming wavefront, where $\theta=0$ corresponds to straight ahead.

Because each antenna is in a different location and the speed of light is finite, there is a small, direction-dependent phase delay $a_m(\theta)$ for each antenna $m$ relative to the first antenna:

$$a_m(\theta) = \exp\left(-j2\pi \cdot (v^\top r_m) \cdot f_c/c\right)$$

where $v =\begin{bmatrix}\cos\theta & \sin\theta\end{bmatrix}^\top$ is the unit vector in the direction of $\theta$ pointing away from the receiver. We can create a "steering vector" $a(\theta)\in\mathbb{C}^M$ using all $M$ of these phase delays, where $[a(\theta)]_m = a_m(\theta)$. If we uniformly sample $\theta_\text{count}$ values of $\theta$, then we can collect all of the steering vectors into an $M\times \theta_\text{count}$ matrix $A$.

In addition to AoA, some methods leverage per-subcarrier phase delays due to differences in time of flight (ToF). Let $\tau \in [\tau_\text{min}, \tau_\text{max}]$ be the ToF of an incoming wavefront. Differences in subcarrier frequencies cause a noticeable phase delay $b_n(\tau)$ for the $n$'th subcarrier:

$$
b_n(\tau) = \exp\left(
   -j2\pi \cdot (n-1)f_\delta \cdot \tau
\right)
$$

We can create another steering vector $b(\tau) \in \mathbb{C}^N$ using all $N$ of these phase delays, where $[b(\tau)]_n=b_n(\tau)$. If we uniformly sample $\tau_\text{count}$ values of $\tau$, then we can collect all of the steering vectors into a $N \times \tau_\text{count}$ matrix $B$.

Each method uses the $A$ matrix (and maybe also the $B$ matrix) to compute a matrix we call a "profile" $p_{i,j} \in \mathbb{R}^{\theta_\text{count} \times \tau_\text{count}}$ that describes the intensity given $\theta=\theta_i$ and $\tau=\tau_j$, calculated using csi data $H \in \mathbb{C}^{N \times M}$. Higher intensities correspond to increased confidence that those are the true values of $\theta$ and $\tau$, but the values themselves are (1) arbitrary and (2) not necessarily comparable across algorithms.

This 2D profile is published as a [`Profile2d`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Profile2d.msg) message to `/profile2d`. To create the 1D profile $p_i$ that is published as a [`Profile1d`](https://github.com/Jklein64/rf_msgs/blob/main/msg/Profile1d.msg) to `/profile1d`, we compute the single most likely ToF sample index

$$j^* = \arg\max_j \sum_{i=1}^{\theta_\text{count}} p_{i,j}$$

And then define $p_i = p_{i, j^*}$.

The specifics of each method are documented below. Note that the `new_csi` given to the `csi_callback` has shape `(n_sub, n_rx, n_tx)`, so `n_sub` is $N$ and `n_rx` is $M$. The descriptions use $X^*$ to denote the conjugate transpose of $X$, and $`\Re\{z\}`$ to denote the real component of a complex variable $z$.

### conventional

Conventional beamforming but in reverse. Often referred to as the FFT method.

1. Fill a length $k$ circular buffer $C \in \mathbb{C}^{N \times M \times k}$ with collected CSI data, treating data corresponding to each transmitter as if it was a completely new measurement.
2. For each subcarrier $n$, let $C_n \in \mathbb{C}^{M\times k}$ be a matrix whose columns are CSI data for each receiver, and let $\mu_n \in \mathbb{C}^M$ be the average of each row of $C_n$. Compute the covariance matrix $S_n = \frac{1}{k-1}(C_n - \mu_n)(C_n - \mu_n)^*$ (broadcasting the subtraction).
3. Compute the per-subcarrier profile $`p_{i,n} = \Re \{a(\theta_i)^*S_n a(\theta_i)\}`$.
4. Average across subcarriers to compute $p_i = \frac{1}{n}\sum_n p_{i,n}$.

This method only requires using one subcarrier, not necessarily all of them.

### svd

Uses the first principal component to merge CSI data from multiple frames. The SVD alleviates the noise present in methods that only use the current frame.

1. Fill a length $k$ circular buffer $C \in \mathbb{C}^{NM \times k}$ with collected CSI matrices that have been turned into vectors by stacking their columns, treating data corresponding to each transmitter as if it was a completely new measurement.
1. Compute the row-wise mean $\mu$ and the leading left singular vector $u \in \mathbb{C}^{NM}$ of $C$.
1. Compute $c = \mu^*u/u^*u$ so $u$, a normalization factor rescaling $u$ to its original range.
1. Unvectorize $cu$ into a CSI matrix $X \in \mathbb{C}^{N \times M}$.
1. Compute the conventional beamforming profile $p = |A^*X^*B|$.

The normalization is so that $X$ is in a similar range as the original CSI data, which would not be the case normally due to the effects of truncating the SVD.

### wiros

A method developed as part of the [original WiROS source code](https://github.com/ucsdwcsng/wiros_processing_node/blob/af16caa8fb049d899288e459d727b13c143a391b/src/csi_utils/transform_utils.py#L175), originally named `rx_svd`. This method provides a significant speed boost over the **full_svd** method while providing results of comparable quality.

1. Fill a length $k$ circular buffer $C \in \mathbb{C}^{N \times M \times k}$ with collected CSI data, treating data corresponding to each transmitter as if it was a completely new measurement.
1. For each subcarrier $n$, compute the row-wise mean $\mu_n \in \mathbb{C}^M$ and the leading left singular vector $u_n \in \mathbb{C}^{M}$ of the $M \times k$ matrix corresponding to the data in $C$ for the $n$'th subcarrier.
1. Compute $`c_n = \mu_n^*u_n/u_n^* u_n`$, a normalization factor rescaling $u_n$ to its original range.
1. Collect $c_n u_n$ into a CSI matrix $X \in \mathbb{C}^{N \times M}$, where $[X]_{n,\cdot} = c_n u_n$.
1. Compute the conventional beamforming profile $p = |A^*X^*B|$.

The normalization is so that $X$ is in a similar range as the original CSI data, which would not be the case normally due to the effects of truncating the SVD.

### music

The MUSIC algorithm is a subspace method. It assumes that the eigenspace of the covariance matrix can be partitioned into a subspace corresponding to the signal of interest and a subspace corresponding to noise. MUSIC identifies a basis for the noise subspace and rejects steering vectors that are close to it.

This version of the algorithm estimates the number of incoming wavefronts.

1. Fill a length $k$ circular buffer $C \in \mathbb{C}^{N \times M \times k}$ with collected CSI data, treating data corresponding to each transmitter as if it was a completely new measurement.
1. For each subcarrier $n$, let $C_n \in \mathbb{C}^{M\times k}$ be a matrix whose columns are CSI data for each receiver, and let $\mu_n \in \mathbb{C}^M$ be the average of each row of $C_n$. Compute the covariance matrix $R_n = \frac{1}{k-1}(C_n - \mu_n)(C_n - \mu_n)^*$ (broadcasting the subtraction).
1. Let $e_n^i$ and $v_n^i$ be the eigenvalues and eigenvectors of $R_n$ respectively. Create a basis for the noise subspace $\mathcal E_n = \{e_n^i \mid e_n^i < \alpha \cdot \max_i e_n^i\}$, where $\alpha$ is a threshold parameter (normally $0.1$).
1. Let $E_n \in \mathbb{C}^{M \times L}$ be a matrix whose columns are elements of $\mathcal E_n$, so that the column space of $E_n$ is the noise subspace.
1. Compute the per-subcarrier profile $`p_{i,n} = \frac{1}{\mathbb{R}e \left\{a(\theta_i)^* E_nE_n^* a(\theta_i)\right\}}`$.
1. Average across subcarriers to compute $p_i = \frac{1}{n}\sum_n p_{i,n}$.

This method only requires using one subcarrier, not necessarily all of them.

### capon

Capon beamforming, also known as the Minimum Variance Distortionless Response (MVDR). This algorithm is an adaptive beamformer, and has better performance than conventional beamforming without the need to select or estimate the number of incoming wavefronts like in MUSIC.

1. Fill a length $k$ circular buffer $C \in \mathbb{C}^{N \times M \times k}$ with collected CSI data, treating data corresponding to each transmitter as if it was a completely new measurement.
1. For each subcarrier $n$, let $C_n \in \mathbb{C}^{M\times k}$ be a matrix whose columns are CSI data for each receiver, and let $\mu_n \in \mathbb{C}^M$ be the average of each row of $C_n$. Compute the covariance matrix $R_n = \frac{1}{k-1}(C_n - \mu_n)(C_n - \mu_n)^*$ (broadcasting the subtraction).
1. Compute the per-subcarrier profile $`p_{i,n} = \frac{1}{\mathbb{R}e \left\{a(\theta_i)^* R^{-1} a(\theta_i)\right\}}`$.
1. Average across subcarriers to compute $p_i = \frac{1}{n}\sum_n p_{i,n}$.

This method only requires using one subcarrier, not necessarily all of them.

## Resources

- [Beamforming & DoA, PySDR Docs](https://pysdr.org/content/doa.html)
