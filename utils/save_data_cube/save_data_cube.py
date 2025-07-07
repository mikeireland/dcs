#!/usr/bin/env python3

import numpy as np
import astropy.io.fits as pf
from xaosim.shmlib import shm
import argparse
import sys
import datetime
import os

def main():
    parser = argparse.ArgumentParser(description="Saving a datacube")
    parser.add_argument("--N", dest="nbframes", action="store", type=int,
                        default=100, help="Number of images to save in cube")
    parser.add_argument("--shm", dest="target", action="store", type=str,
                        default="/dev/shm/cred1.im.shm",
                        help="The shared memory to track")
    args = parser.parse_args()
    now = datetime.datetime.utcnow()

    # --------
    stream = shm(args.target, nosem=False)
    label = args.target.split("/dev/shm/")[1].split(".")[0]
    ys, xs = stream.mtdata['y'], stream.mtdata['x']
    dcube = np.zeros((args.nbframes, ys, xs))

    semid = 0
    stream.catch_up_with_sem(semid)

    if (stream.mtdata['naxis'] == 3):
        for ii in range(args.nbframes):
            dcube[ii] = stream.get_latest_data_slice(semid)
        pass
    else:
        for ii in range(args.nbframes):
            dcube[ii] = stream.get_latest_data(semid)

    # --------
    sdir = f"/home/asg/Data/{now.year}{now.month:02d}{now.day:02d}/custom/"
    if not os.path.exists(sdir):
        os.makedirs(sdir)
        
    fname = sdir+f"cube_{now.hour:02d}:{now.minute:02d}:{now.second:02d}"
    fname += f"_{label}.fits"
    pf.writeto(fname, dcube, overwrite=True)
    print("wrote ", fname)
    sys.exit()

# -----------------------
if __name__ == "__main__":
    main()
