import numpy as np

C = 3e8
# SUBCARRIER_WIDTH = 80e6 / 256

# Refer to Gast, Matthew S. 802.11 ac: a survival guide: Wi-Fi at gigabit and beyond. " O'Reilly Media, Inc.", 2013;
# Chapter 2. The Phy
# Note: For 20 MHz channel, we ignore the last subcarrier index, i.e. consider -27 to 27 and ignore the -28th and 28th SC;
# We observe these subcarriers estimated by ASUS has lower power than the rest.

SUBCARRIER_SPACING = {
    80e6: np.arange(-122, 123, 1)[
        np.logical_not(
            np.in1d(
                np.arange(-122, 123),
                np.array([-103, -75, -39, -11, -1, 0, 1, 11, 39, 75, 103]),
            )
        )
    ],
    40e6: np.arange(-58, 59, 1)[
        np.logical_not(
            np.in1d(np.arange(-58, 59), np.array([-53, -25, -11, -1, 0, 1, 11, 25, 53]))
        )
    ],
    20e6: np.arange(-27, 28, 1)[
        np.logical_not(np.in1d(np.arange(-27, 28), np.array([-21, -7, 0, 7, 21])))
    ],
}

SUBCARRIER_INDICES = {
    # convert from number that describes offset from the center frequency index to an
    # absolute index into the subcarriers array by adding `2**(bw / 10e6) / 2`
    80e6: SUBCARRIER_SPACING[80e6] + 128,
    40e6: SUBCARRIER_SPACING[40e6] + 64,
    20e6: SUBCARRIER_SPACING[20e6] + 32,
}

n_sub = {bw: np.shape(offsets)[0] for (bw, offsets) in SUBCARRIER_SPACING.items()}

subcarrier_frequencies = {
    bw: offsets * bw / (2 ** (bw / 10e6))
    for (bw, offsets) in SUBCARRIER_SPACING.items()
}


def get_channel_frequencies(ch, bw):
    if bw not in subcarrier_frequencies:
        print("Invalid bandwidth for OFDM")
        return None
    return 5e9 + ch * 5e6 + subcarrier_frequencies[bw]


no_prof_algos = ["aoa_only"]
