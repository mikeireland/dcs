#!/usr/bin/env python3

import numpy as np
from xaosim.shmlib import shm
from xara.core import _dist as dist

shift = np.fft.fftshift
fft = np.fft.fft2
ifft = np.fft.ifft2

xsz = 32

empty = np.zeros((xsz, xsz))
empty2 = np.zeros((xsz, 2*xsz))

dd = dist(xsz, xsz, between_pix=False)
mask = np.exp(-(dd / 10)**4)

def centered_ft(arr):
    return shift(fft(shift(arr * mask)))

hei_ks = shm("/dev/shm/hei_k1.im.shm", nosem=False)
hei_kl = shm("/dev/shm/hei_k2.im.shm", nosem=False)

hei_ps = shm("/dev/shm/hei_ps.im.shm", data=empty2, nosem=False)
hei_ph = shm("/dev/shm/hei_ph.im.shm", data=empty2, nosem=False)

# hei_ph1 = shm("/dev/shm/hei_ph1.im.shm", data=empty)
# hei_ph2 = shm("/dev/shm/hei_ph2.im.shm", data=empty)

while True:
    im1 = hei_ks.get_latest_data(semid=1).astype(float)
    im2 = hei_kl.get_latest_data(semid=1).astype(float)

    # im1[9, 29] = 0.0
    # im2[1, 18] = 0.0

    im1 -= np.median(im1)
    im2 -= np.median(im2)

    norm1 = im1.sum()
    norm2 = im2.sum()
    if norm1 != 0 and norm2 != 0:
        im1 /= im1.sum() / 10000
        im2 /= im2.sum() / 10000

        ft1 = centered_ft(im1)
        ft2 = centered_ft(im2)
        ps1 = np.abs(ft1)
        ps2 = np.abs(ft2)

        ps1 /= ps1.max() / 25000
        ps2 /= ps2.max() / 25000

        tmp = np.zeros_like(empty2)
        tmp[:,:xsz] = ps1
        tmp[:, xsz:] = ps2

        hei_ps.set_data(tmp)
    
        ph1 = np.angle(ft1)
        ph2 = np.angle(ft2)

        ph1[ps1 < 200] = 0.0
        ph2[ps2 < 200] = 0.0

        tmp[:,:xsz] = ph1
        tmp[:, xsz:] = ph2
    
        hei_ph.set_data(tmp)
