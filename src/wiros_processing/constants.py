import numpy as np

C = 2.99792458e8

# everything here is hardcoded for channel 155, 5 GHz with 80 MHz bandwidth

NUM_RAW_SUBCARRIERS = 256
NUM_USABLE_SUBCARRIERS = 234

# center frequency of the channel
F0 = 5_775_000_000
# frequency spacing between subcarriers
F_DELTA = 312_500

# index offsets from the channel center frequency
nulls = {-128, -127, -126, -125, -124, -123, -1, 0, 1, 123, 124, 125, 126, 127}
pilots = {11, -11, 39, -39, 75, -75, 103, -103}

RAW_SUBCARRIER_INDICES = np.arange(0, 256)
RAW_SUBCARRIER_FREQUENCIES = F0 + F_DELTA * (RAW_SUBCARRIER_INDICES - 128)
# pilots and nulls have information that might confuse algorithms, so shouldn't be used
USABLE_SUBCARRIER_INDICES = np.sort(list(set(range(-128, 128)) - pilots - nulls)) + 128
USABLE_SUBCARRIER_FREQUENCIES = F0 + F_DELTA * (USABLE_SUBCARRIER_INDICES - 128)
