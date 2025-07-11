"""
Various methods of drawing scrolling plots using pyqtgraph for speed and simplicity.
"""

import ZMQ_control_client as Z
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

status = Z.send("status")
v2_K1 = np.zeros((100, 6))
v2_K2 = np.zeros((100, 6))
pd_tel = np.zeros((100, 4))
gd_tel = np.zeros((100, 4))
dm = np.zeros((100, 4))
offload = np.zeros((100, 4))

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Scrolling Plots")
win.resize(1000, 800)
win.setWindowTitle("Heimdallr Real-Time Plots")

plots = []
curves = []

# V^2 plots
p1 = win.addPlot(title="V^2 K1")
p1.setLabel("left", "V^2")
p1.setLabel("bottom", "samples")
c1 = [p1.plot(pen=pg.intColor(i, 6)) for i in range(6)]
plots.append(p1)
curves.append(c1)

p6 = win.addPlot(title="V^2 K2")
p6.setLabel("left", "V^2")
p6.setLabel("bottom", "samples")
c6 = [p6.plot(pen=pg.intColor(i, 6)) for i in range(6)]
plots.append(p6)
curves.append(c6)

win.nextRow()
# Phase Delay plot
p2 = win.addPlot(title="Phase Delay (wavelengths)")
p2.setLabel("left", "Phase Delay (wavelengths)")
p2.setLabel("bottom", "samples")
c2 = [p2.plot(pen=pg.intColor(i, 4)) for i in range(4)]
plots.append(p2)
curves.append(c2)

# Group Delay plot
p3 = win.addPlot(title="Group Delay (wavelengths)")
p3.setLabel("left", "Group Delay (wavelengths)")
p3.setLabel("bottom", "samples")
c3 = [p3.plot(pen=pg.intColor(i, 4)) for i in range(4)]
plots.append(p3)
curves.append(c3)

win.nextRow()
# Mirror Piston plot
p4 = win.addPlot(title="Mirror Piston (fractional stroke)")
p4.setLabel("left", "Mirror Piston (fractional stroke)")
p4.setLabel("bottom", "samples")
c4 = [p4.plot(pen=pg.intColor(i, 4)) for i in range(4)]
plots.append(p4)
curves.append(c4)

#'closure_phase_K1', 'closure_phase_K2', 'dl_offload', 'dm_piston', 'gd_bl', 'gd_snr', 'gd_tel', 'pd_av', 'pd_av_filtered', 'pd_bl', 'pd_offset', 'pd_snr', 'pd_tel', 'test_ix', 'test_n', 'v2_K1', 'v2_K2'
# Offloaded Piston plot
p5 = win.addPlot(title="Offloaded Piston (microns)")
p5.setLabel("left", "Offloaded Piston (microns)")
p5.setLabel("bottom", "samples")
c5 = [p5.plot(pen=pg.intColor(i, 4)) for i in range(4)]
plots.append(p5)
curves.append(c5)


def update():
    global status, v2_K1, v2_K2, pd_tel, gd_tel, dm, offload
    status = Z.send("status")
    v2_K1[:-1] = v2_K1[1:]
    v2_K1[-1] = status["v2_K1"]
    v2_K2[:-1] = v2_K2[1:]
    v2_K2[-1] = status["v2_K2"]
    pd_tel[:-1] = pd_tel[1:]
    pd_tel[-1] = status["pd_tel"]
    gd_tel[:-1] = gd_tel[1:]
    gd_tel[-1] = status["gd_tel"]
    dm[:-1] = dm[1:]
    dm[-1] = status["dm_piston"]
    offload[:-1] = offload[1:]
    offload[-1] = status["dl_offload"]

    # Update plots
    for i in range(6):
        curves[0][i].setData(v2_K1[:, i])
        curves[1][i].setData(v2_K2[:, i])
    for i in range(4):
        curves[2][i].setData(pd_tel[:, i])
        curves[3][i].setData(gd_tel[:, i])
        curves[4][i].setData(dm[:, i])
        curves[5][i].setData(offload[:, i])


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # update every 100 ms

if __name__ == "__main__":
    QtWidgets.QApplication.instance().exec_()
