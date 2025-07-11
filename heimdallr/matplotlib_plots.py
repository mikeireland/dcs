"""
Various methods of drawing scrolling plots using pyqtgraph for speed and simplicity.
"""

import ZMQ_control_client as Z
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

N_TSCOPES = 4
N_BASELINES = 6

status = Z.send("status")
v2_K1 = np.zeros((100, N_BASELINES))
v2_K2 = np.zeros((100, N_BASELINES))
pd_tel = np.zeros((100, N_TSCOPES))
gd_tel = np.zeros((100, N_TSCOPES))
dm = np.zeros((100, N_TSCOPES))
offload = np.zeros((100, N_TSCOPES))
gd_snr = np.zeros((100, N_BASELINES))  # new: group delay SNR
pd_snr = np.zeros((100, N_BASELINES))  # new: phase delay SNR

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Scrolling Plots")
win.resize(1200, 900)
win.setWindowTitle("Heimdallr Real-Time Plots")

plots = []
curves = []

# --- Left Column: Telescopes ---
# Subheader
telescopes_label = pg.LabelItem(justify="center", color="w")
telescopes_label.setText("<span style='font-size:16pt'><b>Telescopes</b></span>")
win.addItem(telescopes_label, row=0, col=0)

# gd_tel
p_gd_tel = win.addPlot(row=1, col=0, title="Group Delay (wavelengths)")
p_gd_tel.setLabel("left", "GD (wavelengths)")
p_gd_tel.setLabel("bottom", "samples")
c_gd_tel = [p_gd_tel.plot(pen=pg.intColor(i, N_TSCOPES)) for i in range(N_TSCOPES)]

# pd_tel
p_pd_tel = win.addPlot(row=2, col=0, title="Phase Delay (wavelengths)")
p_pd_tel.setLabel("left", "PD (wavelengths)")
p_pd_tel.setLabel("bottom", "samples")
c_pd_tel = [p_pd_tel.plot(pen=pg.intColor(i, N_TSCOPES)) for i in range(N_TSCOPES)]

# offload
p_offload = win.addPlot(row=3, col=0, title="Offloaded Piston (microns)")
p_offload.setLabel("left", "Offloaded Piston (μm)")
p_offload.setLabel("bottom", "samples")
c_offload = [p_offload.plot(pen=pg.intColor(i, N_TSCOPES)) for i in range(N_TSCOPES)]

# dm
p_dm = win.addPlot(row=4, col=0, title="Mirror Piston (fractional stroke)")
p_dm.setLabel("left", "Mirror Piston")
p_dm.setLabel("bottom", "samples")
c_dm = [p_dm.plot(pen=pg.intColor(i, N_TSCOPES)) for i in range(N_TSCOPES)]

# --- Right Column: Baselines ---
# Subheader
baselines_label = pg.LabelItem(justify="center", color="w")
baselines_label.setText("<span style='font-size:16pt'><b>Baselines</b></span>")
win.addItem(baselines_label, row=0, col=1)

# v2_K1
p_v2_K1 = win.addPlot(row=1, col=1, title="V² K1")
p_v2_K1.setLabel("left", "V² K1")
p_v2_K1.setLabel("bottom", "samples")
c_v2_K1 = [
    p_v2_K1.plot(
        pen=pg.intColor(i, N_BASELINES)
    )
    for i in range(N_BASELINES)
]

# v2_K2
p_v2_K2 = win.addPlot(row=2, col=1, title="V² K2")
p_v2_K2.setLabel("left", "V² K2")
p_v2_K2.setLabel("bottom", "samples")
c_v2_K2 = [
    p_v2_K2.plot(
        pen=pg.intColor(i, N_BASELINES)
    )
    for i in range(N_BASELINES)
]

# gd_snr
p_gd_snr = win.addPlot(row=3, col=1, title="Group Delay SNR")
p_gd_snr.setLabel("left", "GD SNR")
p_gd_snr.setLabel("bottom", "samples")
c_gd_snr = [
    p_gd_snr.plot(
        pen=pg.intColor(i, N_BASELINES)
    )
    for i in range(N_BASELINES)
]

# pd_snr
p_pd_snr = win.addPlot(row=4, col=1, title="Phase Delay SNR")
p_pd_snr.setLabel("left", "PD SNR")
p_pd_snr.setLabel("bottom", "samples")
c_pd_snr = [
    p_pd_snr.plot(
        pen=pg.intColor(i, N_BASELINES)
    )
    for i in range(N_BASELINES)
]

# --- Store curves for update ---
curves = [
    c_gd_tel,  # 0
    c_pd_tel,  # 1
    c_offload,  # 2
    c_dm,  # 3
    c_v2_K1,  # 4
    c_v2_K2,  # 5
    c_gd_snr,  # 6
    c_pd_snr,  # 7
]


def update():
    global status, v2_K1, v2_K2, pd_tel, gd_tel, dm, offload, gd_snr, pd_snr
    status = Z.send("status")
    arrays = [
        (gd_tel, "gd_tel"),
        (pd_tel, "pd_tel"),
        (offload, "dl_offload"),
        (dm, "dm_piston"),
        (v2_K1, "v2_K1"),
        (v2_K2, "v2_K2"),
        (gd_snr, "gd_snr"),
        (pd_snr, "pd_snr"),
    ]
    for arr, key in arrays:
        arr[:] = np.roll(arr, -1, axis=0)
        arr[-1] = status[key]

    # Update plots
    for i in range(N_TSCOPES):
        curves[0][i].setData(gd_tel[:, i])
        curves[1][i].setData(pd_tel[:, i])
        curves[2][i].setData(offload[:, i])
        curves[3][i].setData(dm[:, i])
    for i in range(N_BASELINES):
        curves[4][i].setData(v2_K1[:, i])
        curves[5][i].setData(v2_K2[:, i])
        curves[6][i].setData(gd_snr[:, i])
        curves[7][i].setData(pd_snr[:, i])


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # update every 100 ms

if __name__ == "__main__":
    QtWidgets.QApplication.instance().exec_()
