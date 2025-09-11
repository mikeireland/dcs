""" ====================================================================
Live display of the location of the Fourier power wrt the assumed hole
coordinates. Should be used for pupil alignment wih MDS
==================================================================== """
import numpy as np
import pyqtgraph as pg
import matplotlib.cm as cm

from xaosim.QtMain import QtMain
from xaosim.shmlib import shm
from xaosim.pupil import hex_grid_coords as hexcoords
from xara.iwfs import IWFS

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QRect

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

from wfs import Heimdallr

import threading

import time
import sys

# =====================================================================
#                   global variables and tools
# =====================================================================

myqt = 0  # myqt is a global variable
nbuv = 6  # the number of baselines to track for

# electric rainbow burst (ndg, blu, grn, ylw,rng, red)
palette6 = [(100, 52, 233), (44,124,229), (73,204,92),
            (248,196,33), (251,102,64), (248,37,83)]

# tropical summer vibes (dgrn, trk, grn, ylw, orng, dorng)
palette6 = [(38, 70, 83), (42,157,143), (138,177,125),
            (233,196,106), (244,162,97), (231,111,81)]

cmap = cm.rainbow

semid = 8
dstream = shm("/dev/shm/hei_k2.im.shm", nosem=False)  # data stream
im_offset = 1000

# ------------ hole center coordinates --------------
hc = np.loadtxt("N1_hole_coordinates.txt")
K2_wl = 2.25e-6  # pupil tracking uses K2
pscale = 64.75

# ----------- holes with finer sampling -------------
hc_full = []
for jj in range(hc.shape[0]):
    tmp = hexcoords(2, radius=0.1) # 1
    for ii in range(tmp.shape[1]):
        tmp[:, ii] += hc[jj, :]
    hc_full.append(tmp.T)
hc_full = np.reshape(hc_full, (4*19, 2)) # 4 * 7

hdlr0 = IWFS(array=hc)
hdlr0.update_img_properties(isz=32, wl=K2_wl, pscale=pscale)
uu0, vv0 = hdlr0.kpi.UVC.T
uu0, vv0 = np.append(uu0, -uu0), np.append(vv0, -vv0)

hdlr = IWFS(array=hc_full)
hdlr.update_img_properties(isz=32, wl=K2_wl, pscale=pscale)
uu, vv = hdlr.kpi.UVC.T
uu, vv = np.append(uu, -uu), np.append(vv, -vv)

rad0 = 0.5
sp_indices = []
for ii in range(2 * nbuv):
    sp_indices.append(
        np.argwhere(
            (np.abs(uu-uu0[ii]) < rad0) * (np.abs(vv-vv0[ii]) < rad0))[:,0])

# Fourier u,v slope
uusl0 = uu[sp_indices[0]]
uusl0 -= uusl0.mean()
uunorm = uusl0.dot(uusl0)

vvsl0 = vv[sp_indices[0]]
vvsl0 -= vvsl0.mean()
vvnorm = vvsl0.dot(vvsl0)


# breakpoint()

# ============================================================
#                   Thread specifics
# ============================================================
class GenericThread(QtCore.QThread):
    ''' ---------------------------------------------------
    generic thread class used to externalize the execution
    of a function (calibration, closed-loop) to a separate
    thread.
    --------------------------------------------------- '''
    def __init__(self, function, *args, **kwargs):
        QtCore.QThread.__init__(self)
        self.function = function
        self.args = args
        self.kwargs = kwargs
 
    def __del__(self):
        self.wait()
 
    def run(self):
        self.function(*self.args,**self.kwargs)
        return

# ==========================================================
#              Creating the Application
# ==========================================================
class App(QtWidgets.QMainWindow): 
    # ------------------------------------------------------
    def __init__(self):
        super().__init__()
        self.title = 'Heimdallr K2 Power Spectrum display'
        self.left, self.top = 0, 0
        self.width, self.height = 520, 520

        self.setWindowTitle(self.title) 
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setMinimumSize(QtCore.QSize(self.width, self.height))
        self.setMaximumSize(QtCore.QSize(self.width, self.height))
        self.main_widget = MyMainWidget(self) 
        self.setCentralWidget(self.main_widget) 

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(500)

    # ------------------------------------------------------
    def refresh(self):
        self.main_widget.refresh_plot()

    # ------------------------------------------------------
    def closeEvent(self, event):
        self.main_widget.close_program()


# =============================================================================
# =============================================================================
class MyMainWidget(QWidget):
    def __init__(self, parent): 
        super(QWidget, self).__init__(parent)

        # ---------------------------------------------------------------------
        #                              top menu
        # ---------------------------------------------------------------------
        self.actionOpen = QtWidgets.QAction(
            QtGui.QIcon(":/images/open.png"), "&Open...", self)

        self.actionQuit = QtWidgets.QAction(
            QtGui.QIcon(":/images/open.png"), "&Quit", self)

        self.actionQuit.triggered.connect(self.close_program)
        self.actionQuit.setShortcut('Ctrl+Q')

        self.menu = parent.menuBar()
        file_menu = self.menu.addMenu("&File")
        file_menu.addAction(self.actionQuit)

        self.gView_psp = pg.PlotWidget(self)
        self.apply_layout()


    # =========================================================================
    def close_program(self):
        # called when using menu or ctrl-Q
        dstream.close(erase_file=False)
        sys.exit()

    # =========================================================================
    def apply_layout(self):
        clh = 28   # control line height
        plw = 500  # plot width
        plh = 500  # plot height

        # -------------------
        #  the live displays
        # -------------------
        # px = 10 + np.array([0, 0, 0, plw, plw, plw])
        # py = 10 + np.tile(np.arange(nbuv/2) * (10 + plh), 2).astype(int)

        self.gView_psp.setGeometry(QRect(10, 10, plw, plh))
        self.gView_psp.showGrid(x=True, y=True, alpha=0.3)
        self.gView_psp.setBackground('w')

        for ii in range(nbuv):
            circ = pg.CircleROI([uu0[ii]-rad0, vv0[ii]-rad0], radius=rad0,
                                pen=pg.mkPen(palette6[ii], width=5))
            self.gView_psp.addItem(circ)
            lbl = pg.TextItem(f"Baseline #{ii+1}", color='k', fill='w',
                              anchor=(0.5, 0.5))
            self.gView_psp.addItem(lbl)
            lbl.setPos(uu0[ii], vv0[ii] + 1.2*rad0)

        self.scatter_plot = pg.ScatterPlotItem(x=uu, y=vv, size=8)
        self.gView_psp.addItem(self.scatter_plot)

    # =========================================================================
    def refresh_plot(self):
        frame = dstream.get_data().astype(float) - im_offset
        cvis = hdlr.extract_cvis_from_img(frame) / 19
        cvis0 = hdlr0.extract_cvis_from_img(frame)

        # ---------------------------------------
        #    power processing for pupil-drift
        # ---------------------------------------
        # data = np.append(np.abs(cvis), np.abs(cvis))
        # values = np.clip(data, 0, 1)

        # ---------------------------------------
        #    phase processing for tip-tilt
        # ---------------------------------------
        btx, bty = np.array([]), np.array([])
        phi = np.angle(cvis)
        # print(f"\rphi min = {phi.min()}, max = {phi.max()}", end="", flush=True)

        phi = np.append(phi, -phi)
        phi0 = np.angle(cvis0) # piston term
        phi0 = np.append(phi0, -phi0)

        for ii in range(2 * nbuv):
            tmp = phi[sp_indices[ii]]
            tmp -= phi0[ii]
            tmp = np.mod(tmp + np.pi, 2*np.pi) - np.pi
            # phi[sp_indices[ii]] -= phi0[ii]
            phi[sp_indices[ii]] = tmp

        for ii in range(nbuv):
            btx = np.append(btx, phi[sp_indices[ii]].dot(uusl0) / uunorm)
            bty = np.append(bty, phi[sp_indices[ii]].dot(vvsl0) / vvnorm)

        tx = hdlr0.PINV.dot(btx)
        ty = hdlr0.PINV.dot(bty)

        # ---------------------------------------
        print("\rtx = " + np.array2string(
            tx, formatter={'float_kind':lambda x: "%+5.2f" % x}) + " ty = " +
              np.array2string(ty, formatter={'float_kind':lambda x: "%+5.2f" % x}),
              end='', flush=True)
        offset = np.pi / 2
        colors = 255 * cmap((phi + offset) / (2*offset))
        self.scatter_plot.setData(x=uu, y=vv, pen=colors, brush=colors, size=8)
        # pass

# ==========================================================
# ==========================================================
def main():
    global myqt
    myqt = QtMain()

    gui = App()
    gui.show()
    myqt.mainloop()
    myqt.gui_quit()


# ==========================================================
# ==========================================================
if __name__ == '__main__': 
    main()
