#! /usr/bin/env python3
import argparse
import os
from os.path import join

import csi_utils.pipeline_utils as pipeline_utils
import numpy as np
import rosbag
import tqdm

parse = argparse.ArgumentParser()
parse.add_argument("bag", help="Input CSI bag file")
args, _ = parse.parse_known_args()
bag = rosbag.Bag(args.bag)


channels = {}
times = {}
aoas = {}
rssis = {}
profs = {}
aoa_sensors = {}
rx = None
out = os.getcwd()

init_skip = 50  # packets
ii = 0

for topic, msg, t in tqdm.tqdm(bag.read_messages("/csi")):

    if ii < init_skip:
        ii += 1
        continue
    # print(msg)
    csi = pipeline_utils.extract_csi(msg)
    # assuming this does not change
    rx = msg.rx_id
    mac = pipeline_utils.mac_to_str(tuple(msg.txmac))
    break

Hcomp = csi
np.save(join(out, f"comp-{rx}.npy"), Hcomp)
print(f"Created {join(out,f'{rx}-{ksplit[1]}.npy')}")
