import numpy as np
import pyqtgraph as pg

from xaosim.QtMain import QtMain
from xaosim.shmlib import shm

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

myqt = 0   # myqt is a global variable


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
        self.title = 'Asgard Lab Fringe Monitor'
        self.left, self.top = 0, 0
        self.width, self.height = 650, 810
        self.band = "K1"  # default band for fringe search

        self.setWindowTitle(self.title) 
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.setMinimumSize(QtCore.QSize(self.width, self.height))
        self.setMaximumSize(QtCore.QSize(self.width, self.height))
        self.main_widget = MyMainWidget(self) 
        self.setCentralWidget(self.main_widget) 

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)

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

        self.gView_plot_vis_k1 = pg.PlotWidget(self)
        self.gView_plot_vis_k2 = pg.PlotWidget(self)
        self.gView_plot_gdlay = pg.PlotWidget(self)

        self.pB_start = QtWidgets.QPushButton(self)
        self.pB_start.setText("START")

        self.pB_stop = QtWidgets.QPushButton(self)
        self.pB_stop.setText("STOP")

        self.pB_cloop = QtWidgets.QPushButton(self)
        self.pB_cloop.setText("C-LOOP")

        self.pB_oloop = QtWidgets.QPushButton(self)
        self.pB_oloop.setText("O-LOOP")
        
        self.pB_reset_dms = QtWidgets.QPushButton(self)
        self.pB_reset_dms.setText("RESET DMs")

        self.pB_gd_set_offset = QtWidgets.QPushButton(self)
        self.pB_gd_set_offset.setText("GD offset")

        self.pB_gd_fgt_offset = QtWidgets.QPushButton(self)
        self.pB_gd_fgt_offset.setText("Reset GD")
        
        self.pB_test = QtWidgets.QPushButton(self)
        self.pB_test.setText("TEST")

        # ----- scanning for fringes ----
        self.cmB_select_filter = QtWidgets.QComboBox(self)
        self.cmB_select_filter.addItem("K1")
        self.cmB_select_filter.addItem("K2")        
        
        self.dspB_scan_range = QtWidgets.QDoubleSpinBox(self)
        self.dspB_scan_range.setDecimals(0)
        self.srange_val = 100.0  # default scan range in microns
        self.scan_step = 5.0     # default scan step size in microns
        self.dspB_scan_range.setValue(self.srange_val)
        self.dspB_scan_range.setMinimum(5.0)
        self.dspB_scan_range.setMaximum(2000.0)
        
        self.chB_fine_scan = QtWidgets.QCheckBox(self)
        self.chB_fine_scan.setText("fine")
        
        self.pB_scan_beam1 = QtWidgets.QPushButton(self)
        self.pB_scan_beam1.setText("SCAN HFO1")
        self.pB_scan_beam2 = QtWidgets.QPushButton(self)
        self.pB_scan_beam2.setText("SCAN HFO2")
        self.pB_scan_beam3 = QtWidgets.QPushButton(self)
        self.pB_scan_beam3.setText("SCAN HFO3")
        self.pB_scan_beam4 = QtWidgets.QPushButton(self)
        self.pB_scan_beam4.setText("SCAN HFO4")


        self.data_setup()
        self.apply_layout()

        self.tracking = False

    # =========================================================================
    def wfs_start(self):
        if self.tracking:
            print("Already tracking")
        else:
            self.tracking = True
            self.wfsThread = GenericThread(self.wfs.loop)
            self.wfsThread.start()

    # =========================================================================
    def wfs_stop(self):
        self.tracking = False
        self.wfs.stop()

    # =========================================================================
    def wfc_start(self):
        if self.wfs.cloop_on is True:
            return
        self.wfs.cloop_on = True
        print("Closing the loop!")

    # =========================================================================
    def wfc_stop(self):
        self.wfs.cloop_on = False
        print("Breaking the loop!")
        
    # =========================================================================
    def close_program(self):
        # called when using menu or ctrl-Q
        self.wfs_stop()
        try:
            self.wfs.close()
        except:
            pass
        sys.exit()

    # =========================================================================
    def data_setup(self):
        self.wfs = Heimdallr()

    # =========================================================================
    def apply_layout(self):
        clh = 28   # control line height
        plw = 500  # plot width
        plh = 250  # plot height
        btx = plw + 20  # buttons x position
        
        self.pB_start.setGeometry(QRect(btx, 30, 100, clh))
        self.pB_cloop.setGeometry(QRect(btx, 60, 100, clh))
        self.pB_oloop.setGeometry(QRect(btx, 90, 100, clh))
        self.pB_stop.setGeometry(QRect(btx, 120, 100, clh))

        self.pB_reset_dms.setGeometry(QRect(btx, 180, 100, clh))

        self.cmB_select_filter.setGeometry(QRect(btx, 230, 50, clh))
        self.dspB_scan_range.setGeometry(QRect(btx, 260, 60, clh))
        self.chB_fine_scan.setGeometry(QRect(btx + 70, 260, 50, clh))

        self.pB_scan_beam1.setGeometry(QRect(btx, 290, 100, clh))
        self.pB_scan_beam2.setGeometry(QRect(btx, 320, 100, clh))
        self.pB_scan_beam3.setGeometry(QRect(btx, 350, 100, clh))
        self.pB_scan_beam4.setGeometry(QRect(btx, 380, 100, clh))

        self.pB_gd_set_offset.setGeometry(QRect(btx, 440, 100, clh))
        self.pB_gd_fgt_offset.setGeometry(QRect(btx, 470, 100, clh))

        # the TEST button
        self.pB_test.setGeometry(QRect(btx, 600, 100, clh))

        # -------------------
        #  the live displays
        # -------------------
        py = 10 + np.arange(3) * (10 + plh)
        self.gView_plot_vis_k1.setGeometry(QRect(10, py[0], plw, plh))
        self.gView_plot_vis_k1.setYRange(0, 0.8)
        self.gView_plot_vis_k2.setGeometry(QRect(10, py[1], plw, plh))
        self.gView_plot_vis_k2.setYRange(0, 0.8)
        self.gView_plot_gdlay.setGeometry(QRect(10, py[2], plw, plh))
        self.logplot_vis_k1 = []
        self.logplot_vis_k2 = []
        self.logplot_gdlay = []

        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
                  (255, 255, 0), (255, 0, 255), (0, 255, 255)]

        for ii in range(6):
            self.logplot_vis_k1.append(self.gView_plot_vis_k1.plot(
                [0, self.wfs.log_len], [ii, ii],
                pen=colors[ii], name=f"v2 #{ii+1}"))

        for ii in range(6):
            self.logplot_vis_k2.append(self.gView_plot_vis_k2.plot(
                [0, self.wfs.log_len], [ii, ii],
                pen=colors[ii], name=f"v2 #{ii+1}"))

        for ii in range(3):
            self.logplot_gdlay.append(self.gView_plot_gdlay.plot(
                [0, self.wfs.log_len], [ii, ii],
                pen=colors[ii], name=f"OPD #{ii+1}"))


        self.pB_start.clicked.connect(self.wfs_start)
        self.pB_stop.clicked.connect(self.wfs_stop)
        self.pB_cloop.clicked.connect(self.wfc_start)
        self.pB_oloop.clicked.connect(self.wfc_stop)

        self.pB_reset_dms.clicked.connect(self.reset_dms)

        self.cmB_select_filter.activated[str].connect(self.select_filter)
        self.select_filter()

        self.dspB_scan_range.valueChanged.connect(self.update_scan_range)
        self.chB_fine_scan.stateChanged[int].connect(self.update_scan_step)
        self.pB_scan_beam1.clicked.connect(self.scan_beam1)
        self.pB_scan_beam2.clicked.connect(self.scan_beam2)
        self.pB_scan_beam3.clicked.connect(self.scan_beam3)
        self.pB_scan_beam4.clicked.connect(self.scan_beam4)

        self.pB_gd_set_offset.clicked.connect(self.set_gd_offset)
        self.pB_gd_fgt_offset.clicked.connect(self.fgt_gd_offset)

        self.pB_test.clicked.connect(self.trigger_test)
        # self.pB_dec_pscale.clicked.connect(self.dec_pscale)

    # =========================================================================
    def set_gd_offset(self):
        self.wfs.gd_offset = self.wfs.gdlay
        print("New set point!")

    # =========================================================================
    def fgt_gd_offset(self):
        self.wfs.gd_offset = np.zeros(6)
        print("Forgot set point!")

    # =========================================================================
    def select_filter(self):
        self.band = str(self.cmB_select_filter.currentText())
        # print(self.band)
        pass

    # =========================================================================
    def update_scan_step(self):
        "Control of the scan step in microns"
        if self.chB_fine_scan.isChecked():
            self.scan_step = 0.5
        else:
            self.scan_step = 5.0
        # print(f"step size updated to {self.scan_step} um")

    # =========================================================================
    def trigger_test(self):
        print("start")
        self.modulation_thread = GenericThread(
            self.wfs.dm_modulation_response)
        self.modulation_thread.start()

    # =========================================================================
    def update_scan_range(self):
        self.srange_val = self.dspB_scan_range.value()

    # =========================================================================
    def scan_beam(self, beamid=1):
        self.vscan_thread = GenericThread(
            self.wfs.fringe_search,
            beamid=beamid, srange=self.srange_val,
            step=self.scan_step, band=self.band)
        self.vscan_thread.start()

    # =========================================================================
    def scan_beam1(self):
        self.scan_beam(1)
        
    # =========================================================================
    def scan_beam2(self):
        self.scan_beam(2)

    # =========================================================================
    def scan_beam3(self):
        self.scan_beam(3)

    # =========================================================================
    def scan_beam4(self):
        self.scan_beam(4)
    
    # =========================================================================
    def reset_dms(self):
        self.wfs.reset_dms()

    # =========================================================================
    def refresh_plot(self):
        for ii in range(3):
            self.logplot_gdlay[ii].setData(self.wfs.opds[ii])
        for ii in range(6):
            self.logplot_vis_k1[ii].setData(self.wfs.vis_k1[ii])
            self.logplot_vis_k2[ii].setData(self.wfs.vis_k2[ii])

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
