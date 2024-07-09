import numpy as np

C = 2.99792458e8

# Refer to Gast, Matthew S. 802.11 ac: a survival guide: Wi-Fi at gigabit and beyond.
# " O'Reilly Media, Inc.", 2013 Chapter 2. Note: For 20 MHz channel, we ignore the
# last subcarrier index, i.e. consider -27 to 27 and ignore the -28th and 28th SC;
# We observe these subcarriers estimated by ASUS has lower power than the rest
SUBCARRIER_SPACING = {
    80e6: np.array(
        sorted(set(range(-122, 123)) - {-103, -75, -39, -11, -1, 0, 1, 11, 39, 75, 103})
    ),
    40e6: np.array(sorted(set(range(-58, 59)) - {-53, -25, -11, -1, 0, 1, 11, 25, 53})),
    20e6: np.array(sorted(set(range(-27, 28)) - {-21, -7, 0, 7, 21})),
}

# `SUBCARRIER_INDICES[bw]` is an array of indices into CSI data `csi` that extracts only the
# subcarriers that are actually used for the given bandwidth
SUBCARRIER_INDICES = {
    # convert from number that describes offset from the center frequency index to an
    # absolute index into the subcarriers array by adding `2**(bw / 10e6) / 2`
    80e6: SUBCARRIER_SPACING[80e6] + 128,
    40e6: SUBCARRIER_SPACING[40e6] + 64,
    20e6: SUBCARRIER_SPACING[20e6] + 32,
}

# `SUBCARRIER_FREQUENCIES[bw]` is an array of subcarrier frequencies, where the
# `i`'th element is the `i`'th subcarrier frequency
SUBCARRIER_FREQUENCIES = {
    bw: offsets * bw / (2 ** (bw / 10e6))
    for (bw, offsets) in SUBCARRIER_SPACING.items()
}
