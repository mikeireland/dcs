#!/usr/bin/env python3

import numpy as np
import sys
from xaosim.shmlib import shm
from xara.iwfs import IWFS
from xara.core import recenter
from xaosim.pupil import _dist as dist
import astropy.io.fits as pf
import time
import zmq
import datetime
import os

# from scipy.interpolate import griddata

# ----------------------------------------
# pupil geometry design
hcoords = np.array([[ 0.  ,  0.  ],  # Beam 3
                    [-1.04,  2.46],  # Beam 2
                    [ 2.77,  0.01],  # Beam 4
                    [-1.20, -1.94]]) # Beam 1

hcoords = np.array([[ 0.  ,  0.  ],    # Beam 3
                    [-1.025,  2.460],  # Beam 2
                    [ 2.785,  0.035],  # Beam 4
                    [-1.145, -1.965]]) # Beam 1

# reordered so that bench & xara order are same?

hcoords = np.array([[-1.025,  2.460],  # Bench Beam 1
                    [-1.145, -1.965],  # Bench Beam 2
                    [ 0.000  ,0.000],  # Bench Beam 3
                    [ 2.785,  0.035]]) # Bench Beam 4

# ----------------------------------------
# piston mode design
dms = 12
dd = dist(dms, dms, between_pix=True)  # auxilliary array
tprad = 5.5  # the taper function radius
taper = np.exp(-(dd/tprad)**20)  # power to be adjusted ?
amask = taper > 0.4  # seems to work well

pst = np.zeros((dms, dms))
pst[amask] = 0.02  # 0.2 DM piston on GUI -> 10 ADU units # optical gain ~ 3µm / ADU

im_offset = 1000.0

class Heimdallr():
    # =========================================================================
    def __init__(self):
        self.ndm = 4  # number of DMs
        self.chn = 4  # channel number dedicated to Heimdallr
        self.xsz = 32
        self.dd = dist(self.xsz, self.xsz, between_pix=True)
        self.apod = np.exp(-(self.dd/8)**4)

        self.gd_offset = np.zeros(6)
        # pf.writeto("apodizer.fits", self.apod, overwrite=True)
        
        self.pscale = 35
        self.Ks_wl = 2.05e-6  # True Heimdallr Ks wavelength (in meters)
        self.Kl_wl = 2.25e-6  # True Heimdallr Kl wavelength (in meters)

        self.dl_factor = self.Kl_wl / (self.Kl_wl - self.Ks_wl) / 2*np.pi

        self.hdlr1 = IWFS(array=hcoords)
        self.hdlr2 = IWFS(array=hcoords)

        # phase and group delay to be measured relative to beam 3
        # so a custom PINV is requested here
        self.PINV = np.round(np.linalg.pinv(
            np.delete(self.hdlr1.kpi.BLM, 2, axis=1)), 2)
        
        # self.PINV = np.round(np.linalg.pinv(self.hdlr1.kpi.BLM), 2)

        self.hdlr1.update_img_properties(
            isz=self.xsz, wl=self.Ks_wl, pscale=self.pscale)
        self.hdlr2.update_img_properties(
            isz=self.xsz, wl=self.Kl_wl, pscale=self.pscale)

        self.Ks = shm("/dev/shm/hei_k1.im.shm", nosem=False)
        self.Kl = shm("/dev/shm/hei_k2.im.shm", nosem=False)
        self.semid = 7

        self.dms = []
        self.sems = []

        for ii in range(self.ndm):
            self.dms.append(shm(f"/dev/shm/dm{ii+1}disp{self.chn:02d}.im.shm"))
            self.sems.append(shm(f"/dev/shm/dm{ii+1}.im.shm", nosem=False))

        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, 10000)
        self.socket.connect("tcp://192.168.100.2:5555")

        self.hfo_pos = np.zeros(4)
        print("---")
        for ii in range(self.ndm):
            self.hfo_pos[ii] = self.get_dl_pos(ii+1)
            print(f"HFO{ii+1} = {self.hfo_pos[ii]:.2f} um")
        print("---")

        self.keepgoing = True
        self.log_len = 2000  # length of the sensor log
        self.opds = [[], [], []]  # the log of the measured OPDs
        self.gdlays = [[], [], [], [], [], []]  # the log of group delays
        self.vis_k1 = [[], [], [], [], [], []]  # log of K1 visibility
        self.vis_k2 = [[], [], [], [], [], []]  # log of K2 visibility

        self.calibrated = False
        self.calibrating = False
        self.cloop_on = False
        self.disps = np.zeros(self.ndm)

        self.gain = 0.01

    # =========================================================================
    def calc_wfs_data(self):
        k1d = self.Ks.get_latest_data(self.semid).astype(float) - im_offset
        k2d = self.Kl.get_latest_data(self.semid).astype(float) - im_offset
        k1d *= self.apod
        k2d *= self.apod
        # img = recenter(data, verbose=False)
        norm1 = k1d.sum()
        norm2 = k2d.sum()
        if norm1 != 0:
            k1d /= norm1
        if norm2 != 0:
            k2d /= norm2

        if norm1 != 0:
            self.hdlr1.extract_data(k1d)
        if norm2 != 0:
            self.hdlr2.extract_data(k2d)

        self.opd_now_k1 = self.PINV.dot(np.angle(self.hdlr1.cvis[0]))
        self.opd_now_k2 = self.PINV.dot(np.angle(self.hdlr2.cvis[0]))
        tmp = np.angle(self.hdlr1.cvis[0] * self.hdlr2.cvis[0].conj())
        self.gdlay = tmp * self.dl_factor - self.gd_offset

        self.dms_cmds = self.PINV.dot(self.gdlay)
               
        # self.dms_cmds = self.opd_now_k1  # (or k2) - as a test?
        # self.dms_cmds = np.insert(self.dms_cmds, 0, 0)
        # self.dms_cmds -= self.dms_cmds[2] # everything relative to Beam 3
        # print(f"\r{self.dms_cmds}", end="")

    # =========================================================================
    def log_opds(self):
        for ii in range(6):
            self.gdlays[ii].append(self.gdlay[ii])

        for ii in range(self.ndm-1):
            # self.opds[ii].append(self.opd_now_k1[ii])
            self.opds[ii].append(self.dms_cmds[ii])

        if len(self.opds[0]) > self.log_len:
            for ii in range(self.ndm-1):
                self.opds[ii].pop(0)

            for ii in range(6):
                self.gdlays[ii].pop(0)

    # =========================================================================
    def log_vis(self):
        for ii in range(6):
            self.vis_k1[ii].append(np.abs(self.hdlr1.cvis[0][ii]))
            self.vis_k2[ii].append(np.abs(self.hdlr2.cvis[0][ii]))

        if len(self.vis_k1[0]) > self.log_len:
            for ii in range(6):
                self.vis_k1[ii].pop(0)
                self.vis_k2[ii].pop(0)

    # =========================================================================
    def dispatch_opds(self):
        ref_beam = 0.25 * np.sum(self.dms_cmds)


        self.disps[0] = self.dms_cmds[0]
        self.disps[1] = self.dms_cmds[1] #0.0 # ref_beam
        self.disps[2] = 0.0
        self.disps[3] = self.dms_cmds[2]

        if self.cloop_on:
            for ii in range(self.ndm):
                p0 = self.dms[ii].get_data()
                dm = 0.999 * (p0 + self.gain * self.disps[ii] * pst)
                self.dms[ii].set_data(dm)
                self.sems[ii].post_sems(1)

    # =========================================================================
    def get_dl_pos(self, beamid=1):
        self.socket.send_string(f"read HFO{beamid}")
        return float(self.socket.recv_string().strip()) * 1e3

    # =========================================================================
    def move_dl(self, pos, beamid=1):
        # print(f"moveabs HFO{beamid} {pos:.5f}")
        self.socket.send_string(f"moveabs HFO{beamid} {1e-3 * pos:.5f}")
        self.socket.recv_string()  # acknowledgement
        # cur_pos = self.get_dl_pos(beamid)
        # while not np.isclose(pos, cur_pos, atol=1.0):
        #     time.sleep(0.5)
        #     cur_pos = self.get_dl_pos(beamid)

    # =========================================================================
    def fringe_search(self, beamid=1, srange=100.0, step=5.0, band="K1"):
        ''' -------------------------------------------------- 
        Fringe search!

        Parameters:
        ----------

        - beamid : 1, 2, 3 or 4 (BEAM ID #) (int)
        - srange : the +/- search range in microns (float)
        - step   : the scan step in microns (float)
        -------------------------------------------------- '''
        nav = 5  # number of measurements to average

        for ii in range(self.ndm):
            self.hfo_pos[ii] = self.get_dl_pos(ii+1)
            
        x0 = self.hfo_pos[beamid-1] # startup position
        steps = np.arange(x0 - srange, x0 + srange, step)
        if band == "K1":
            sensor = self.hdlr1
        else:
            sensor = self.hdlr2

        BLM = sensor.kpi.BLM.copy()
        bl_ii = np.argwhere(BLM[:, beamid-1] != 0)[:, 0]  # concerned BLines
        # the starting point
        best_pos = x0
        vis = []
        for jj in range(nav):
            vis.append(np.abs(sensor.cvis[0]))
        vis = np.mean(np.array(vis), axis = 0)
        uvis = np.round(np.abs(vis)[bl_ii], 2)  # "useful" visibilities
        best_vis = np.round(np.sqrt(np.mean(uvis**2)), 2)
        print(f"HFO{beamid} x0  = {x0:8.2f}", end="")
        print(uvis, best_vis)
        found_one = 0

        if self.cloop_on:
            self.cloop_on = False  # interrupting the loop
            print("Opening the loop")

        for ii, pos in enumerate(steps):
            self.move_dl(pos, beamid)
            print(f"HFO{beamid} pos = {pos:8.2f} ", end="")
            time.sleep(0.5)
            vis = []
            for jj in range(nav):
                vis.append(np.abs(sensor.cvis[0]))
            vis = np.mean(np.array(vis), axis = 0)

            uvis = np.round(vis[bl_ii], 2)  # the useful visibilities here
            global_vis = np.round(np.sqrt(np.mean(uvis**2)), 2)
            print(uvis, global_vis, end="")
            
            if (global_vis >= 1.01 * best_vis) and (global_vis > 0.15) :
                best_vis = global_vis
                best_pos = pos
                found_one += 1 # (found_one == 1) and
                print(f"    - Current best pos: {pos:.2f}")
            else:
                print()

            if (global_vis < 0.8 * best_vis) and \
               (pos > x0) and (best_vis > 0.15):
                time.sleep(0.5)
                break
            # time.sleep(0.5)  --> wait after moving the HFO instead?
        print(f"Done! Best pos is {best_pos:.2f} um for v = {best_vis:.2f}\n")
        self.move_dl(best_pos, beamid)

    # =========================================================================
    def dm_modulation_response(self):
        ''' -------------------------------------------------- 
        Sinusoidal DM modulation
        -------------------------------------------------- '''
        nmod = 100
        nc = [1, 2, 3, 4]
        a0 = 10.0 # 0.5
        nav = 10  # number of measures per modulation
        
        self.reset_dms()
        now = datetime.datetime.utcnow()

        # prepare sinusoidal modulation commands
        cmds = np.zeros((self.ndm, nmod))
        for jj in range(self.ndm):
            cmds[jj] = a0 * np.sin(np.linspace(0, nc[jj] * 2*np.pi, nmod))

        # cmds[0] *= 0.0
        # cmds[1] *= 0.0
        # cmds[3] *= 0.0
        
        # to store the data
        cvis1 = np.zeros((self.hdlr1.kpi.nbuv, nmod * nav), dtype=complex)
        cvis2 = np.zeros((self.hdlr2.kpi.nbuv, nmod * nav), dtype=complex)

        cube1 = np.zeros((nmod * nav, 32, 32))
        cube2 = np.zeros((nmod * nav, 32, 32))

        for ii in range(nmod):
            # modulate the DMs
            for jj in range(self.ndm):
                self.dms[jj].set_data(cmds[jj, ii] * pst)
                self.sems[jj].post_sems(1)
                time.sleep(0.05)

            # record the data
            for jj in range(nav):
                imK1 = self.Ks.get_latest_data() - im_offset
                imK2 = self.Kl.get_latest_data() - im_offset
                cube1[ii * nav + jj] = imK1
                cube2[ii * nav + jj] = imK2

                cvis1[:, ii * nav + jj] = self.hdlr1.extract_cvis_from_img(imK1)
                cvis2[:, ii * nav + jj] = self.hdlr2.extract_cvis_from_img(imK2)

        self.reset_dms()

        sdir = f"/home/asg/Data/{now.year}{now.month:02d}{now.day:02d}/custom/"
        if not os.path.exists(sdir):
            os.makedirs(sdir)
        
        fname_root = sdir+f"data_{now.hour:02d}:{now.minute:02d}:{now.second:02d}_"

        np.savetxt(fname_root + f"modulation_cvis1_a0={a0:.2f}.txt", cvis1)
        np.savetxt(fname_root + f"modulation_cvis2_a0={a0:.2f}.txt", cvis2)

        pf.writeto(fname_root + f"modulation_cube_k1_a0={a0:.2f}.fits", cube1,
                   overwrite=True)
        pf.writeto(fname_root + f"modulation_cube_k2_a0={a0:.2f}.fits", cube2,
                   overwrite=True)
        print("modulation test done")

    # =========================================================================
    def stop(self):
        self.keepgoing = False
        self.cloop_on = False
        time.sleep(0.1)
        print("\n")

    # =========================================================================
    def loop(self):
        self.keepgoing = True
        # catch-up with the semaphore
        self.Ks.catch_up_with_sem(self.semid)

        # start the loop
        while self.keepgoing:
            self.calc_wfs_data()
            self.dispatch_opds()
            self.log_opds()
            self.log_vis()

    # =========================================================================
    def reset_dms(self):
        for ii in range(self.ndm):
            self.dms[ii].set_data(0.0 * pst)
            self.sems[ii].post_sems(1)

    # =========================================================================
    def close(self):
        self.Ks.close(erase_file=False)
        self.Kl.close(erase_file=False)
        self.socket.disconnect()
        # for ii in range(self.ndm):
        #     self.dms[ii].close(erase_file=False)
        #     self.sems[ii].close(erase_file=False)
