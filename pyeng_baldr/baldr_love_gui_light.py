#!/usr/bin/env python3
"""
Light‑weight Baldr live GUI
---------------------------
Goals vs original:
- Keep the ZMQ command‑line interface intact.
- Default update rate ≈ 2 Hz (editable).
- 1D plots: only "strehl proxy" (rmse_est) and e_LO (RMS). Keep 10 s rolling history (time‑based).
- 2D views: only latest frame for "signal", "e_HO", "img_dm", "c_LO", "c_HO" (no history).
- Use pyqtgraph PlotWidget + ImageItem (faster than ImageView), avoid autoLevels per frame.
- Avoid unnecessary copies; cast arrays to float32; prune histories aggressively.

Run:
  python baldr_live_gui_light.py --beams 1,2

"""

import sys, time, json, zmq
from collections import deque
import numpy as np
import argparse

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QComboBox, QTextEdit, QLineEdit, QLabel, QDoubleSpinBox,
)
from PyQt5.QtCore import QTimer
import pyqtgraph as pg

# ---------- CLI args ----------
parser = argparse.ArgumentParser(description="Baldr RTC light GUI")
parser.add_argument("--beams", type=str, default="1",
                    help="comma-separated beams to open (e.g. 1 or 1,2,3,4)")
args = parser.parse_args()
BEAMS = [int(b.strip()) for b in args.beams.split(',') if b.strip()]

SERVER_ADDR_DICT = {
    1: "tcp://127.0.0.1:6662",
    2: "tcp://127.0.0.1:6663",
    3: "tcp://127.0.0.1:6664",
    4: "tcp://127.0.0.1:6665",
}

# Global pyqtgraph perf knobs
pg.setConfigOptions(useOpenGL=False, antialias=False, background='k', foreground='w')

# ---------- ZMQ Commander helper ----------
class CommanderClient:
    def __init__(self, addr, rcv_timeout_ms=1000, snd_timeout_ms=1000):
        self.ctx = zmq.Context.instance()
        self.addr = addr
        self.rcv_to = rcv_timeout_ms
        self.snd_to = snd_timeout_ms
        self._make_sock()

    def _make_sock(self):
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.setsockopt(zmq.RCVTIMEO, self.rcv_to)
        self.sock.setsockopt(zmq.SNDTIMEO, self.snd_to)
        self.sock.connect(self.addr)

    def _reset(self):
        try:
            self.sock.close(0)
        except Exception:
            pass
        self._make_sock()

    def _send_recv(self, msg: str):
        try:
            self.sock.send_string(msg)
            return self.sock.recv_string()
        except zmq.error.Again:
            self._reset(); return None
        except zmq.ZMQError as e:
            if e.errno == zmq.EFSM:
                self._reset(); return None
            raise

    def call(self, cmd, *args):
        if len(args) == 0:
            msg = cmd
        elif len(args) == 1:
            msg = f"{cmd} {json.dumps(args[0])}"
        else:
            msg = f"{cmd} {json.dumps(list(args))}"
        reply = self._send_recv(msg)
        if reply is None:
            return {"ok": False, "error": "timeout/efsm", "raw": None}
        try:
            return json.loads(reply)
        except json.JSONDecodeError:
            return {"ok": False, "error": "non-JSON reply", "raw": reply}

    def poll_scalar(self, name: str):
        return self.call("poll_telem_scalar", name)

    def poll_vector(self, name: str):
        return self.call("poll_telem_vector", name)

    def send_raw(self, text: str) -> str:
        reply = self._send_recv(text)
        return reply if reply is not None else "[timeout/efsm]"

# ---------- Utilities ----------
DM_N = 12
DM_MISSING_CORNERS = (0, DM_N-1, DM_N*(DM_N-1), DM_N*DM_N-1)

def dm140_to_12x12(arr):
    arr = np.asarray(arr, dtype=np.float32).ravel()
    if arr.size != 140:
        raise ValueError("DM vector must be length 140")
    out = []
    j = 0
    for k in range(DM_N*DM_N):
        if k in DM_MISSING_CORNERS:
            out.append(np.nan)
        else:
            out.append(arr[j]); j += 1
    return np.asarray(out, dtype=np.float32).reshape(DM_N, DM_N)

# ---------- 2D Panel ----------
class Panel2D(QWidget):
    """Show ONLY the latest frame for one of: signal, e_HO, img_dm, c_LO, c_HO."""
    CHOICES = ["signal", "e_HO", "img_dm", "c_LO", "c_HO"]

    def __init__(self, client: CommanderClient):
        super().__init__()
        self.client = client

        top = QHBoxLayout()
        self.field = QComboBox(); self.field.addItems(self.CHOICES)
        top.addWidget(QLabel("2D field:")); top.addWidget(self.field); top.addStretch(1)

        self.view = pg.PlotWidget()
        self.view.setAspectLocked(True)
        self.imgitem = pg.ImageItem()
        self.view.addItem(self.imgitem)

        lay = QVBoxLayout(); lay.addLayout(top); lay.addWidget(self.view)
        self.setLayout(lay)

        self._levels = None

    def tick(self):
        name = self.field.currentText()
        res = self.client.poll_vector(name)
        if not res.get("ok"):
            return
        vec = np.asarray(res.get("data", []), dtype=np.float32).ravel()
        if name == "img_dm":
            img = dm140_to_12x12(vec)
        else:
            n = int(np.sqrt(vec.size))
            if n*n == vec.size and n >= 2:
                img = vec.reshape(n, n)
            elif vec.size == 140:
                img = dm140_to_12x12(vec)
            elif vec.size == 144:
                img = vec.reshape(12, 12)
            else:
                return
        if self._levels is None:
            finite = np.isfinite(img)
            if not np.any(finite):
                return
            vmin, vmax = float(np.percentile(img[finite], 1)), float(np.percentile(img[finite], 99))
            if vmax <= vmin:
                vmax = vmin + 1.0
            self._levels = (vmin, vmax)
        self.imgitem.setImage(img.T, autoLevels=False, levels=self._levels)

# ---------- 1D Panel ----------
class Panel1D(QWidget):
    """Show a time series for either strehl proxy (rmse_est scalar) or e_LO RMS."""
    CHOICES = ["strehl proxy", "e_LO (RMS)"]

    def __init__(self, client: CommanderClient, default_hz: float = 2.0):
        super().__init__()
        self.client = client
        self.update_hz = float(default_hz)
        self.history_sec = 10.0

        top = QHBoxLayout()
        self.field = QComboBox(); self.field.addItems(self.CHOICES)
        self.rate = QDoubleSpinBox(); self.rate.setDecimals(2); self.rate.setRange(0.2, 20.0); self.rate.setSingleStep(0.2)
        self.rate.setValue(self.update_hz)
        self.rate.valueChanged.connect(self._rate_changed)
        top.addWidget(QLabel("1D:")); top.addWidget(self.field)
        top.addStretch(1)
        top.addWidget(QLabel("Hz:")); top.addWidget(self.rate)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot()

        lay = QVBoxLayout(); lay.addLayout(top); lay.addWidget(self.plot)
        self.setLayout(lay)

        self.t_hist = deque()
        self.v_hist = deque()

        self.timer = QTimer(self); self.timer.timeout.connect(self.tick)
        self._apply_interval()
        self.plot.setLabel('bottom', 'Time', units='s')

    def _rate_changed(self, hz):
        try:
            self.update_hz = float(hz)
            self._apply_interval()
        except Exception:
            pass

    def _apply_interval(self):
        interval_ms = int(max(50, round(1000.0 / max(0.1, self.update_hz))))
        self.timer.start(interval_ms)

    def _prune(self, now):
        cutoff = now - self.history_sec
        while self.t_hist and self.t_hist[0] < cutoff:
            self.t_hist.popleft(); self.v_hist.popleft()

    def _get_strehl_proxy(self):
        res = self.client.poll_scalar("rmse_est")
        if res.get("ok"):
            val = float(res.get("data", np.nan))
            return val
        return np.nan

    def _get_e_lo_rms(self):
        res = self.client.poll_vector("e_LO")
        if res.get("ok"):
            arr = np.asarray(res.get("data", []), dtype=np.float32).ravel()
            if arr.size == 0:
                return np.nan
            return float(np.sqrt(np.nanmean(arr * arr)))
        return np.nan

    def tick(self):
        now = time.time()
        if self.field.currentText() == "strehl proxy":
            val = self._get_strehl_proxy()
        else:
            val = self._get_e_lo_rms()

        self.t_hist.append(now); self.v_hist.append(val)
        self._prune(now)

        if self.t_hist:
            t0 = self.t_hist[0]
            xs = [t - t0 for t in self.t_hist]
            ys = list(self.v_hist)
            self.curve.setData(xs, ys)

# ---------- CLI widget ----------
class CommandLine(QWidget):
    def __init__(self, client: CommanderClient):
        super().__init__()
        self.client = client
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout()
        self.history = QTextEdit(); self.history.setReadOnly(True)
        self.prompt = QLineEdit();  self.prompt.returnPressed.connect(self.handle_command)
        layout.addWidget(self.history); layout.addWidget(self.prompt)
        self.setLayout(layout)

    def handle_command(self):
        text = self.prompt.text().strip()
        if not text:
            return
        self.history.append(f"> {text}")
        self.prompt.clear()
        try:
            parts = text.split(maxsplit=1)
            msg = parts[0] if len(parts) == 1 else f"{parts[0]} {parts[1]}"
            reply = self.client.send_raw(msg)
        except Exception as e:
            reply = f"[ZMQ error] {e}"
        self.history.append(reply)

# ---------- Main window per beam ----------
class MainWindow(QWidget):
    def __init__(self, beam: int):
        super().__init__()
        addr = SERVER_ADDR_DICT.get(beam)
        if addr is None:
            raise ValueError(f"Invalid beam {beam} (valid: 1..4)")
        self.setWindowTitle(f"Baldr Light GUI — Beam {beam}  ({addr})")
        self.client = CommanderClient(addr)

        left = QVBoxLayout()
        header = QLabel(f"Beam {beam} — {addr}")
        header.setStyleSheet("font-weight:600; padding:2px 4px;")
        left.addWidget(header)
        self.cli = CommandLine(self.client)
        left.addWidget(self.cli)

        right = QVBoxLayout()
        self.panel1d = Panel1D(self.client, default_hz=2.0)
        self.panel2d = Panel2D(self.client)
        right.addWidget(self.panel1d, 1)
        right.addWidget(self.panel2d, 2)

        root = QHBoxLayout(); root.addLayout(left, 1); root.addLayout(right, 2)
        self.setLayout(root)

        self.timer2d = QTimer(self)
        self.timer2d.timeout.connect(self.panel2d.tick)
        self.timer2d.start(500)

# ---------- Run ----------

def run_app():
    app = QApplication(sys.argv)
    windows = []
    x0, y0 = 60, 60
    dx, dy = 30, 30
    for k, beam in enumerate(BEAMS):
        w = MainWindow(beam)
        w.resize(1000, 620)
        w.move(x0 + k * dx, y0 + k * dy)
        w.show()
        windows.append(w)

    sys.exit(app.exec_())

if __name__ == "__main__":
    run_app()

# #!/usr/bin/env python3
# """
# Light‑weight Baldr live GUI
# ---------------------------
# Goals vs original:
# - Keep the ZMQ command‑line interface intact.
# - Default update rate ≈ 2 Hz (editable).
# - 1D plots: only "strehl proxy" (rmse_est) and e_LO (RMS). Keep 10 s rolling history (time‑based).
# - 2D views: only latest frame for "signal", "e_HO", "img_dm", "c_LO", "c_HO" (no history).
# - Use pyqtgraph PlotWidget + ImageItem (faster than ImageView), avoid autoLevels per frame.
# - Avoid unnecessary copies; cast arrays to float32; prune histories aggressively.

# Run:
#   python baldr_live_gui_light.py --beams 1,2

# """

# import sys, time, json, zmq
# from collections import deque
# import numpy as np
# import argparse

# from PyQt5.QtWidgets import (
#     QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
#     QComboBox, QTextEdit, QLineEdit, QLabel, QDoubleSpinBox,
# )
# from PyQt5.QtCore import QTimer
# import pyqtgraph as pg

# # ---------- CLI args ----------
# parser = argparse.ArgumentParser(description="Baldr RTC light GUI")
# parser.add_argument("--beams", type=str, default="1",
#                     help="comma-separated beams to open (e.g. 1 or 1,2,3,4)")
# args = parser.parse_args()
# BEAMS = [int(b.strip()) for b in args.beams.split(',') if b.strip()]

# SERVER_ADDR_DICT = {
#     1: "tcp://127.0.0.1:6662",
#     2: "tcp://127.0.0.1:6663",
#     3: "tcp://127.0.0.1:6664",
#     4: "tcp://127.0.0.1:6665",
# }

# # Global pyqtgraph perf knobs
# pg.setConfigOptions(useOpenGL=False, antialias=False, background='k', foreground='w')

# # ---------- ZMQ Commander helper ----------
# class CommanderClient:
#     def __init__(self, addr, rcv_timeout_ms=1000, snd_timeout_ms=1000):
#         self.ctx = zmq.Context.instance()
#         self.addr = addr
#         self.rcv_to = rcv_timeout_ms
#         self.snd_to = snd_timeout_ms
#         self._make_sock()

#     def _make_sock(self):
#         self.sock = self.ctx.socket(zmq.REQ)
#         self.sock.setsockopt(zmq.RCVTIMEO, self.rcv_to)
#         self.sock.setsockopt(zmq.SNDTIMEO, self.snd_to)
#         self.sock.connect(self.addr)

#     def _reset(self):
#         try:
#             self.sock.close(0)
#         except Exception:
#             pass
#         self._make_sock()

#     def _send_recv(self, msg: str):
#         try:
#             self.sock.send_string(msg)
#             return self.sock.recv_string()
#         except zmq.error.Again:
#             self._reset(); return None
#         except zmq.ZMQError as e:
#             if e.errno == zmq.EFSM:
#                 self._reset(); return None
#             raise

#     def call(self, cmd, *args):
#         if len(args) == 0:
#             msg = cmd
#         elif len(args) == 1:
#             msg = f"{cmd} {json.dumps(args[0])}"
#         else:
#             msg = f"{cmd} {json.dumps(list(args))}"
#         reply = self._send_recv(msg)
#         if reply is None:
#             return {"ok": False, "error": "timeout/efsm", "raw": None}
#         try:
#             return json.loads(reply)
#         except json.JSONDecodeError:
#             return {"ok": False, "error": "non-JSON reply", "raw": reply}

#     def poll_scalar(self, name: str):
#         return self.call("poll_telem_scalar", name)

#     def poll_vector(self, name: str):
#         return self.call("poll_telem_vector", name)

#     def send_raw(self, text: str) -> str:
#         reply = self._send_recv(text)
#         return reply if reply is not None else "[timeout/efsm]"

# # ---------- Utilities ----------
# DM_N = 12
# DM_MISSING_CORNERS = (0, DM_N-1, DM_N*(DM_N-1), DM_N*DM_N-1)

# def dm140_to_12x12(arr):
#     arr = np.asarray(arr, dtype=np.float32).ravel()
#     if arr.size != 140:
#         raise ValueError("DM vector must be length 140")
#     out = []
#     j = 0
#     for k in range(DM_N*DM_N):
#         if k in DM_MISSING_CORNERS:
#             out.append(np.nan)
#         else:
#             out.append(arr[j]); j += 1
#     return np.asarray(out, dtype=np.float32).reshape(DM_N, DM_N)

# # ---------- 2D Panel ----------
# class Panel2D(QWidget):
#     """Show ONLY the latest frame for one of: signal, e_HO, img_dm, c_LO, c_HO."""
#     CHOICES = ["signal", "e_HO", "img_dm", "c_LO", "c_HO"]

#     def __init__(self, client: CommanderClient):
#         super().__init__()
#         self.client = client

#         top = QHBoxLayout()
#         self.field = QComboBox(); self.field.addItems(self.CHOICES)
#         top.addWidget(QLabel("2D field:")); top.addWidget(self.field); top.addStretch(1)

#         self.view = pg.PlotWidget()
#         self.view.setAspectLocked(True)
#         self.imgitem = pg.ImageItem()
#         self.view.addItem(self.imgitem)

#         lay = QVBoxLayout(); lay.addLayout(top); lay.addWidget(self.view)
#         self.setLayout(lay)

#         self._levels = None

#     def tick(self):
#         name = self.field.currentText()
#         res = self.client.poll_vector(name)
#         if not res.get("ok"):
#             return
#         vec = np.asarray(res.get("data", []), dtype=np.float32).ravel()
#         if name == "img_dm":
#             img = dm140_to_12x12(vec)
#         else:
#             n = int(np.sqrt(vec.size))
#             if n*n == vec.size and n >= 2:
#                 img = vec.reshape(n, n)
#             elif vec.size == 140:
#                 img = dm140_to_12x12(vec)
#             elif vec.size == 144:
#                 img = vec.reshape(12, 12)
#             else:
#                 return
#         if self._levels is None:
#             finite = np.isfinite(img)
#             if not np.any(finite):
#                 return
#             vmin, vmax = float(np.percentile(img[finite], 1)), float(np.percentile(img[finite], 99))
#             if vmax <= vmin:
#                 vmax = vmin + 1.0
#             self._levels = (vmin, vmax)
#         self.imgitem.setImage(img.T, autoLevels=False, levels=self._levels)

# # ---------- 1D Panel ----------
# class Panel1D(QWidget):
#     """Show a time series for either strehl proxy (rmse_est scalar) or e_LO RMS."""
#     CHOICES = ["strehl proxy", "e_LO (RMS)"]

#     def __init__(self, client: CommanderClient, default_hz: float = 2.0):
#         super().__init__()
#         self.client = client
#         self.update_hz = float(default_hz)
#         self.history_sec = 10.0

#         top = QHBoxLayout()
#         self.field = QComboBox(); self.field.addItems(self.CHOICES)
#         self.rate = QDoubleSpinBox(); self.rate.setDecimals(2); self.rate.setRange(0.2, 20.0); self.rate.setSingleStep(0.2)
#         self.rate.setValue(self.update_hz)
#         self.rate.valueChanged.connect(self._rate_changed)
#         top.addWidget(QLabel("1D:")); top.addWidget(self.field)
#         top.addStretch(1)
#         top.addWidget(QLabel("Hz:")); top.addWidget(self.rate)

#         self.plot = pg.PlotWidget()
#         self.plot.showGrid(x=True, y=True, alpha=0.3)
#         self.curve = self.plot.plot()

#         lay = QVBoxLayout(); lay.addLayout(top); lay.addWidget(self.plot)
#         self.setLayout(lay)

#         self.t_hist = deque()
#         self.v_hist = deque()

#         self.timer = QTimer(self); self.timer.timeout.connect(self.tick)
#         self._apply_interval()
#         self.plot.setLabel('bottom', 'Time', units='s')

#     def _rate_changed(self, hz):
#         try:
#             self.update_hz = float(hz)
#             self._apply_interval()
#         except Exception:
#             pass

#     def _apply_interval(self):
#         interval_ms = int(max(50, round(1000.0 / max(0.1, self.update_hz))))
#         self.timer.start(interval_ms)

#     def _prune(self, now):
#         cutoff = now - self.history_sec
#         while self.t_hist and self.t_hist[0] < cutoff:
#             self.t_hist.popleft(); self.v_hist.popleft()

#     def _get_strehl_proxy(self):
#         res = self.client.poll_scalar("rmse_est")
#         if res.get("ok"):
#             val = float(res.get("data", np.nan))
#             return val
#         return np.nan

#     def _get_e_lo_rms(self):
#         res = self.client.poll_vector("e_LO")
#         if res.get("ok"):
#             arr = np.asarray(res.get("data", []), dtype=np.float32).ravel()
#             if arr.size == 0:
#                 return np.nan
#             return float(np.sqrt(np.nanmean(arr * arr)))
#         return np.nan

#     def tick(self):
#         now = time.time()
#         if self.field.currentText() == "strehl proxy":
#             val = self._get_strehl_proxy()
#         else:
#             val = self._get_e_lo_rms()

#         self.t_hist.append(now); self.v_hist.append(val)
#         self._prune(now)

#         if self.t_hist:
#             t0 = self.t_hist[0]
#             xs = [t - t0 for t in self.t_hist]
#             ys = list(self.v_hist)
#             self.curve.setData(xs, ys)

# # ---------- CLI widget ----------
# class CommandLine(QWidget):
#     def __init__(self, client: CommanderClient):
#         super().__init__()
#         self.client = client
#         self._init_ui()

#     def _init_ui(self):
#         layout = QVBoxLayout()
#         self.history = QTextEdit(); self.history.setReadOnly(True)
#         self.prompt = QLineEdit();  self.prompt.returnPressed.connect(self.handle_command)
#         layout.addWidget(self.history); layout.addWidget(self.prompt)
#         self.setLayout(layout)

#     def handle_command(self):
#         text = self.prompt.text().strip()
#         if not text:
#             return
#         self.history.append(f"> {text}")
#         self.prompt.clear()
#         try:
#             parts = text.split(maxsplit=1)
#             msg = parts[0] if len(parts) == 1 else f"{parts[0]} {parts[1]}"
#             reply = self.client.send_raw(msg)
#         except Exception as e:
#             reply = f"[ZMQ error] {e}"
#         self.history.append(reply)

# # ---------- Main window per beam ----------
# class MainWindow(QWidget):
#     def __init__(self, beam: int):
#         super().__init__()
#         addr = SERVER_ADDR_DICT.get(beam)
#         if addr is None:
#             raise ValueError(f"Invalid beam {beam} (valid: 1..4)")
#         self.setWindowTitle(f"Baldr Light GUI — Beam {beam}  ({addr})")
#         self.client = CommanderClient(addr)

#         left = QVBoxLayout()
#         header = QLabel(f"Beam {beam} — {addr}")
#         header.setStyleSheet("font-weight:600; padding:2px 4px;")
#         left.addWidget(header)
#         self.cli = CommandLine(self.client)
#         left.addWidget(self.cli)

#         right = QVBoxLayout()
#         self.panel1d = Panel1D(self.client, default_hz=2.0)
#         self.panel2d = Panel2D(self.client)
#         right.addWidget(self.panel1d, 1)
#         right.addWidget(self.panel2d, 2)

#         root = QHBoxLayout(); root.addLayout(left, 1); root.addLayout(right, 2)
#         self.setLayout(root)

#         self.timer2d = QTimer(self)
#         self.timer2d.timeout.connect(self.panel2d.tick)
#         self.timer2d.start(500)

# # ---------- Run ----------

# def run_app():
#     app = QApplication(sys.argv)
#     windows = []
#     x0, y0 = 60, 60
#     dx, dy = 30, 30
#     for k, beam in enumerate(BEAMS):
#         w = MainWindow(beam)
#         w.resize(1000, 620)
#         w.move(x0 + k * dx, y0 + k * dy)
#         w.show()
#         windows.append(w)

#     sys.exit(app.exec_())

# if __name__ == "__main__":
#     run_app()
