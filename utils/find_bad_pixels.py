# For a given camera readout mode, we will have dark pixels that can be sent directly
# to the Heimdallr server or added to the SHM header. Here we print them out.
# We also average the 
import sys
from astropy.io import fits
import numpy as np

fn = sys.argv[0]
dloc = fn.rfind('.')
out_fn = fn[:dloc] + '_orig.fits'
#fn = '20250916/dark_cred1_CROP_GCDS_200_fps_1000_gain_020.fits'

hei_boxes = {"hei_k1": {
        "x0": 12,
        "y0": 19,
        "xsz": 32,
        "ysz": 32,
        "nrs": 3,
        "tsig": [
            2,
            -1,
            -1
        ]
    },
    "hei_k2": {
        "x0": 274,
        "y0": 20,
        "xsz": 32,
        "ysz": 32,
        "nrs": 3,
        "tsig": [
            2,
            -1,
            -1
        ]}}
        
k1x0 = hei_boxes['hei_k1']['x0']
k1y0 = hei_boxes['hei_k1']['y0']
k2x0 = hei_boxes['hei_k2']['x0']
k2y0 = hei_boxes['hei_k2']['y0']
sz = hei_boxes['hei_k1']['xsz']
if ((hei_boxes['hei_k1']['ysz'] != sz) | (hei_boxes['hei_k2']['xsz'] != sz) | (hei_boxes['hei_k2']['ysz'] != sz)):
    raise UserWarning

dd = fits.getdata(fn)
ddmed = np.median(dd, axis=0)
#dd abslute deviation 99th percentile
ddad99 = np.percentile(np.abs(dd - ddmed), 99, axis=0)

badpix = np.where(ddad99 > 3*np.median(ddad99)) #y,x order
k1xpx = []
k1ypx = []
k2xpx = []
k2ypx = []
for y,x in zip(*badpix):
    if ((x >= k1x0) & (x< k1x0+sz) & (y>=k1y0) & (y < k1y0+sz)):
        k1xpx += [x-k1x0]
        k1ypx += [y-k1y0]
    if ((x >= k2x0) & (x< k2x0+sz) & (y>=k2y0) & (y < k2y0+sz)):
        k2xpx += [x-k2x0]
        k2ypx += [y-k2y0]
print(k1xpx)
print(k1ypx)
print(k2xpx)
print(k2ypx)
    
os.rename(fn, out_fn)
dd[:]=ddad99
fits.writeto(fn, dd)


