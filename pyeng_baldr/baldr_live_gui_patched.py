import sys, time, json, zmq
from collections import deque
import numpy as np
import argparse
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QComboBox, QTextEdit, QLineEdit, QGridLayout, QLabel, QSpinBox,
    QCheckBox, QDoubleSpinBox
)
from PyQt5.QtCore import QTimer, pyqtSignal
import pyqtgraph as pg
from functools import partial

# -------------------- Args --------------------
parser = argparse.ArgumentParser(description="Baldr RTC GUI.")
parser.add_argument("--beams", type=str, default="1",
                    help="comma-separated beams to open (e.g. 1 or 1,2,3,4)")
args = parser.parse_args()
BEAMS = [int(b.strip()) for b in args.beams.split(",") if b.strip()]

SERVER_ADDR_DICT = {
    1: "tcp://127.0.0.1:6662",
    2: "tcp://127.0.0.1:6663",
    3: "tcp://127.0.0.1:6664",
    4: "tcp://127.0.0.1:6665",
} #192.168.100.2

# -------------------- Commander ZMQ helper --------------------b792fd616d7913915fcde412207a5e85c2b72963
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
            self._reset()
            return None
        except zmq.ZMQError as e:
            if e.errno == zmq.EFSM:
                self._reset()
                return None
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


# --- helper for 12x12 DM grids from 140-length vectors ---
def get_DM_command_in_2D(cmd, Nx_act=12):
    """Insert NaNs at the four missing corner positions to turn a 140-length
    vector into a 12x12 array suitable for visualization."""
    cmd = np.asarray(cmd, dtype=float).ravel().tolist()
    corner_indices = [0, Nx_act-1, Nx_act * (Nx_act-1), Nx_act*Nx_act]
    cmd_in_2D = list(cmd)
    for i in corner_indices:
        cmd_in_2D.insert(i, np.nan)
    return np.array(cmd_in_2D).reshape(Nx_act, Nx_act)
# -------------------- Plot widget --------------------
class PlotWidget(QWidget):
    new_plot_requested = pyqtSignal()

    SCALAR_FIELDS = ["snr", "rmse_est"]  # extend if you add more
    VECTOR_FIELDS = ["img", "img_dm", "signal", "e_LO", "u_LO", "e_HO", "u_HO", "c_LO", "c_HO", "c_inj"]
    REDUCERS = ["mean", "rms", "index"]

    def __init__(self, server_addr, history=600, beam=None, default_hz=10.0):
        super().__init__()
        self.client = CommanderClient(server_addr)
        self.server_addr = server_addr
        self.history = history
        self.beam = beam
        self.update_hz = float(default_hz)
        self.x = deque(maxlen=self.history)
        self.y = deque(maxlen=self.history)
        self._init_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)  # 10 Hz

    def _init_ui(self):
        layout = QVBoxLayout()

        # Top controls row
        top = QHBoxLayout()
        self.field = QComboBox(); self.field.addItems(self.SCALAR_FIELDS + self.VECTOR_FIELDS)
        self.reducer = QComboBox(); self.reducer.addItems(self.REDUCERS); self.reducer.setCurrentText("mean")
        self.idx_label = QLabel("idx:"); self.idx_spin = QSpinBox(); self.idx_spin.setRange(0, 100000); self.idx_spin.setValue(0)
        self.new_plot_button = QPushButton("+"); self.new_plot_button.setFixedSize(25, 25); self.new_plot_button.clicked.connect(self.new_plot_requested.emit)

        
        # Beam tag (visible inside each plot window)
        if self.beam is not None:
            self.beam_tag = QLabel(f"Beam {self.beam}")
            self.beam_tag.setStyleSheet("padding:2px 6px; border-radius:6px; background:#333; color:#fff;")
            top.addWidget(self.beam_tag)

        top.addWidget(QLabel("Field:")); top.addWidget(self.field); top.addSpacing(10)
        top.addWidget(QLabel("Reducer:")); top.addWidget(self.reducer)
        top.addWidget(self.idx_label); top.addWidget(self.idx_spin)
        top.addStretch(); top.addWidget(self.new_plot_button)

        # Render mode toggles + Update rate
        self.mode_2d = QCheckBox("2D"); self.mode_2d.setToolTip("Render latest frame as 2D image")
        self.mode_psd = QCheckBox("PSD"); self.mode_psd.setToolTip("Plot power spectral density")
        self.hz_spin = QDoubleSpinBox(); self.hz_spin.setDecimals(2); self.hz_spin.setRange(0.1, 200.0); self.hz_spin.setValue(self.update_hz)
        top.addSpacing(10); top.addWidget(self.mode_2d); top.addWidget(self.mode_psd); top.addSpacing(6); top.addWidget(QLabel("Hz:")); top.addWidget(self.hz_spin)

        # Second controls row: Reset + Y autoscale/limits
        controls = QHBoxLayout()
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.reset_plot)

        self.auto_y = QCheckBox("Auto Y"); self.auto_y.setChecked(True)
        self.auto_y.stateChanged.connect(self._apply_y_mode)

        self.ymin = QDoubleSpinBox(); self.ymin.setDecimals(6); self.ymin.setRange(-1e12, 1e12); self.ymin.setValue(-1.0)
        self.ymax = QDoubleSpinBox(); self.ymax.setDecimals(6); self.ymax.setRange(-1e12, 1e12); self.ymax.setValue(1.0)
        self.apply_y = QPushButton("Apply Y")
        self.apply_y.clicked.connect(self._apply_y_limits)

        controls.addWidget(self.reset_btn)
        controls.addSpacing(12)
        controls.addWidget(self.auto_y)
        controls.addSpacing(6)
        controls.addWidget(QLabel("Ymin:")); controls.addWidget(self.ymin)
        controls.addWidget(QLabel("Ymax:")); controls.addWidget(self.ymax)
        controls.addWidget(self.apply_y)
        controls.addStretch()

        # Plot
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot()

        layout.addLayout(top)
        layout.addLayout(controls)
        layout.addWidget(self.plot)
        self.setLayout(layout)

        self.reducer.currentTextChanged.connect(self._toggle_idx_visibility)
        self.image = pg.ImageView()
        self.image.setVisible(False)

        layout.addLayout(top)
        layout.addLayout(controls)
        layout.addWidget(self.plot)
        layout.addWidget(self.image)
        self.setLayout(layout)

        self.reducer.currentTextChanged.connect(self._toggle_idx_visibility)
        self.mode_2d.stateChanged.connect(self._switch_plot_mode)
        self.mode_psd.stateChanged.connect(self._switch_psd_mode)
        self.hz_spin.valueChanged.connect(self._update_rate_changed)
        self._switch_plot_mode()
        self._toggle_idx_visibility(self.reducer.currentText())
        self._apply_y_mode()  # initialize enable/disable of spinners

    def _toggle_idx_visibility(self, txt):
        is_idx = (txt == "index")
        self.idx_label.setVisible(is_idx)
        self.idx_spin.setVisible(is_idx)

    def reset_plot(self):
        """Clear history and refresh."""
        self.x.clear()
        self.y.clear()
        self.curve.setData([], [])

    def _apply_y_mode(self):
        auto = self.auto_y.isChecked()
        # Enable/disable manual inputs
        self.ymin.setEnabled(not auto)
        self.ymax.setEnabled(not auto)
        self.apply_y.setEnabled(not auto)
        # Apply to plot
        self.plot.enableAutoRange('y', enable=auto)
        if not auto:
            self._apply_y_limits()

    def _apply_y_limits(self):
        y0 = float(self.ymin.value())
        y1 = float(self.ymax.value())
        if y1 <= y0:
            # simple guard
            y1 = y0 + 1.0
            self.ymax.setValue(y1)
        self.plot.setYRange(y0, y1, padding=0.0)

    def _reduce_vector(self, vec, how: str):
        if vec is None:
            return float("nan")
        arr = np.asarray(vec, dtype=float).ravel()
        if arr.size == 0:
            return float("nan")
        if how == "mean":
            return float(np.nanmean(arr))
        if how == "rms":
            return float(np.sqrt(np.nanmean(arr * arr)))
        if how == "index":
            i = self.idx_spin.value()
            return float(arr[i]) if 0 <= i < arr.size else float("nan")
        return float("nan")

    
    def _switch_plot_mode(self):
        is2d = self.mode_2d.isChecked()
        if hasattr(self, "image"):
            self.image.setVisible(is2d)
        if hasattr(self, "plot"):
            self.plot.setVisible(not is2d)
        # Disable reducer/index controls in 2D; they're 1D-only
        self.reducer.setEnabled(not is2d)
        self.idx_spin.setEnabled(not is2d and self.reducer.currentText() == "index")
        self.idx_label.setEnabled(not is2d)
        # Y-axis controls apply only to 1D
        self.auto_y.setEnabled(not is2d)
        self.ymin.setEnabled(not is2d and not self.auto_y.isChecked())
        self.ymax.setEnabled(not is2d and not self.auto_y.isChecked())
        self.apply_y.setEnabled(not is2d and not self.auto_y.isChecked())

    def _switch_psd_mode(self):
        # Update axis label when toggling PSD in 1D mode
        if self.mode_psd.isChecked() and not self.mode_2d.isChecked():
            self.plot.setLabel('bottom', 'Frequency', units='Hz')
        else:
            self.plot.setLabel('bottom', 'Time', units='s')

    def _vector_to_2d(self, name: str, vec):
        """Safely turn a telemetry vector into a 2D array for display."""
        arr = np.asarray(vec, dtype=float).ravel()
        if name == "img" and arr.size == 32*32:
            return arr.reshape(32, 32)
        if arr.size == 12*12:
            return arr.reshape(12, 12)
        if arr.size == 140:
            return get_DM_command_in_2D(arr, Nx_act=12)
        n = int(np.sqrt(arr.size))
        if n*n == arr.size and n >= 2:
            return arr.reshape(n, n)
        raise ValueError(f"Cannot reshape '{name}' of length {arr.size} into 2D")

    def _psd_1d(self):
        """Compute and draw 1D PSD of the current y-history (ignore NaNs)."""
        y = np.array([v for v in self.y if np.isfinite(v)], dtype=float)
        n = y.size
        if n < 8:
            self.curve.setData([], [])
            return
        # Estimate sampling interval from timestamps; fallback to 10 Hz
        if len(self.x) >= 2:
            dts = np.diff(list(self.x))
            dT = float(np.median(dts))
            fs = 1.0 / dT if dT > 0 else 10.0
        else:
            fs = 10.0
        y = y - np.nanmean(y)
        Y = np.fft.rfft(y)
        P = (np.abs(Y) ** 2) / n
        f = np.fft.rfftfreq(n, d=1.0 / fs)
        self.curve.setData(f, P)

    def _psd_2d(self, img2d):
        """Return 2D power spectrum (fftshifted) of image."""
        a = np.nan_to_num(img2d, nan=0.0, posinf=0.0, neginf=0.0)
        # Hann window to reduce edge effects
        win_x = np.hanning(a.shape[1])
        win_y = np.hanning(a.shape[0])
        w = np.outer(win_y, win_x)
        A = np.fft.fftshift(np.fft.fft2(a * w))
        return np.log10(np.abs(A) ** 2 + 1e-12)

    def _update_rate_changed(self, hz):
        try:
            hz = float(hz)
            if hz <= 0:
                return
            self.update_hz = hz
            interval = int(max(1, round(1000.0 / hz)))
            self.timer.setInterval(interval)
        except Exception:
            pass

    def update_plot(self):

        t = time.time()
        name = self.field.currentText()
        # 2D mode: render latest frame as image (or its 2D PSD)
        if self.mode_2d.isChecked():
            try:
                if name not in self.VECTOR_FIELDS:
                    return  # 2D only for vectors
                res = self.client.poll_vector(name)
                if not res.get("ok"):
                    return
                img2d = self._vector_to_2d(name, res.get("data", []))
                if self.mode_psd.isChecked():
                    img2d = self._psd_2d(img2d)
                # transpose for conventional orientation
                self.image.setImage(img2d.T, autoLevels=True)
            except Exception:
                return
            return

        # 1D mode: time series or PSD of reduced values
        val = float("nan")
        try:
            if name in self.SCALAR_FIELDS:
                res = self.client.poll_scalar(name)
                if res.get("ok"):
                    val = float(res.get("data", float("nan")))
            else:
                res = self.client.poll_vector(name)
                if res.get("ok"):
                    val = self._reduce_vector(res.get("data", []), self.reducer.currentText())
        except Exception:
            pass  # keep plotting NaNs on transient errors

        self.x.append(t); self.y.append(val)
        if self.mode_psd.isChecked():
            self._psd_1d()
        else:
            self.curve.setData(list(self.x), list(self.y))

# -------------------- CLI --------------------
class CommandLine(QWidget):
    def __init__(self, server_addr):
        super().__init__()
        self.client = CommanderClient(server_addr)
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

# -------------------- Control panel --------------------
class ControlPanel(QWidget):
    def __init__(self, cli_widget):
        super().__init__()
        self.cli = cli_widget
        layout = QGridLayout()

        def send_cmd(text: str):
            self.cli.prompt.setText(text)
            self.cli.handle_command()

        buttons = [
            ("Close LO", partial(send_cmd, 'close_baldr_LO ""')),
            ("Open LO",  partial(send_cmd, 'open_baldr_LO ""')),
            ("Close HO", partial(send_cmd, 'close_baldr_HO ""')),
            ("Open HO",  partial(send_cmd, 'open_baldr_HO ""')),
        ]
        for i, (label, fn) in enumerate(buttons):
            btn = QPushButton(label); btn.clicked.connect(fn)
            layout.addWidget(btn, i // 2, i % 2)
        self.setLayout(layout)

# -------------------- Main window --------------------
class MainWindow(QWidget):
    def __init__(self, beam: int):
        super().__init__()
        addr = SERVER_ADDR_DICT.get(beam)
        if addr is None:
            raise ValueError(f"Invalid beam {beam} (valid: 1..4)")
        self.beam = beam
        self.server_addr = addr
        self.setWindowTitle(f"Baldr Telemetry GUI — Beam {beam}  ({addr})")
        self.plot_windows = []
        self._init_ui()

    def _init_ui(self):
        main = QHBoxLayout()
        left = QVBoxLayout()
        # Beam header visible in-window
        header = QLabel(f"Beam {self.beam} — {self.server_addr}")
        header.setStyleSheet("font-weight:600;")
        left.addWidget(header)
        self.cli = CommandLine(self.server_addr)
        self.ctrl = ControlPanel(self.cli)
        left.addWidget(self.ctrl)
        left.addWidget(self.cli)

        self.plot = PlotWidget(self.server_addr, beam=self.beam)
        self.plot.new_plot_requested.connect(self.spawn_new_plot)

        main.addLayout(left, 1)
        main.addWidget(self.plot, 2)
        self.setLayout(main)

    def spawn_new_plot(self):
        w = PlotWidget(self.server_addr, beam=self.beam)
        w.setWindowTitle(f"Telemetry Plot — Beam {self.beam}")
        w.resize(500, 300)
        w.new_plot_requested.connect(self.spawn_new_plot)
        w.show()
        self.plot_windows.append(w)

# -------------------- Run --------------------
def run_app():
    app = QApplication(sys.argv)
    windows = []
    # Stagger placement a bit so multiple beams don't overlap perfectly
    x0, y0 = 80, 80
    dx, dy = 40, 40
    for k, beam in enumerate(BEAMS):
        w = MainWindow(beam)
        w.resize(1100, 650)
        w.move(x0 + k * dx, y0 + k * dy)
        w.show()
        windows.append(w)
    sys.exit(app.exec_())

if __name__ == "__main__":
    run_app()
    
# import sys, time, json, zmq
# from collections import deque
# import numpy as np
# import argparse

# from PyQt5.QtWidgets import (
#     QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
#     QComboBox, QTextEdit, QLineEdit, QGridLayout, QLabel, QSpinBox
# )
# from PyQt5.QtCore import QTimer, pyqtSignal
# import pyqtgraph as pg
# from functools import partial

# parser = argparse.ArgumentParser(description="Baldr RTC GUI.")
# parser.add_argument("--beam", type=int, default=1, help="which beam")
# args = parser.parse_args()

# # --- FIXED: proper dict (key: value), and centralize build via function
# SERVER_ADDR_DICT = {
#     1: "tcp://127.0.0.1:6662",
#     2: "tcp://127.0.0.1:6663",
#     3: "tcp://127.0.0.1:6664",
#     4: "tcp://127.0.0.1:6665",
# }

# # -------------------- Commander ZMQ helper --------------------
# class CommanderClient:
#     def __init__(self, addr=None, rcv_timeout_ms=1000, snd_timeout_ms=1000):
#         self.ctx = zmq.Context.instance()
#         self.addr = addr or SERVER_ADDR_DICT[args.beam]
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
#             # send/recv timeout -> reset and bubble a marker
#             self._reset()
#             return None
#         except zmq.ZMQError as e:
#             # EFSM happens if REQ order violated; reset and surface
#             if e.errno == zmq.EFSM:
#                 self._reset()
#                 return None
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

#     # Convenience wrappers
#     def poll_scalar(self, name: str):
#         return self.call("poll_telem_scalar", name)

#     def poll_vector(self, name: str):
#         return self.call("poll_telem_vector", name)

#     def send_raw(self, text: str) -> str:
#         reply = self._send_recv(text)
#         return reply if reply is not None else "[timeout/efsm]"

# # -------------------- Plot widget --------------------
# class PlotWidget(QWidget):
#     new_plot_requested = pyqtSignal()

#     SCALAR_FIELDS = ["snr", "rmse_est"]  # extend if you add more
#     VECTOR_FIELDS = ["img", "img_dm", "signal", "e_LO", "u_LO", "e_HO", "u_HO", "c_LO", "c_HO", "c_inj"]
#     REDUCERS = ["mean", "rms", "index"]

#     def __init__(self, server_addr=None, history=600):  # keep ~1 min at 10 Hz
#         super().__init__()
#         self.client = CommanderClient(server_addr or SERVER_ADDR_DICT[args.beam])
#         self.history = history
#         self.x = deque(maxlen=self.history)
#         self.y = deque(maxlen=self.history)
#         self._init_ui()

#         self.timer = QTimer(self)
#         self.timer.timeout.connect(self.update_plot)
#         self.timer.start(100)  # 10 Hz

#     def _init_ui(self):
#         layout = QVBoxLayout()

#         top = QHBoxLayout()
#         self.field = QComboBox(); self.field.addItems(self.SCALAR_FIELDS + self.VECTOR_FIELDS)
#         self.reducer = QComboBox(); self.reducer.addItems(self.REDUCERS); self.reducer.setCurrentText("mean")
#         self.idx_label = QLabel("idx:"); self.idx_spin = QSpinBox(); self.idx_spin.setRange(0, 100000); self.idx_spin.setValue(0)
#         self.new_plot_button = QPushButton("+"); self.new_plot_button.setFixedSize(25, 25)
#         self.new_plot_button.clicked.connect(self.new_plot_requested.emit)

#         top.addWidget(QLabel("Field:")); top.addWidget(self.field); top.addSpacing(10)
#         top.addWidget(QLabel("Reducer:")); top.addWidget(self.reducer)
#         top.addWidget(self.idx_label); top.addWidget(self.idx_spin)
#         top.addStretch(); top.addWidget(self.new_plot_button)

#         self.plot = pg.PlotWidget(); self.plot.showGrid(x=True, y=True, alpha=0.3)
#         self.curve = self.plot.plot()

#         layout.addLayout(top); layout.addWidget(self.plot)
#         self.setLayout(layout)
#         self.reducer.currentTextChanged.connect(self._toggle_idx_visibility)
#         self._toggle_idx_visibility(self.reducer.currentText())

#     def _toggle_idx_visibility(self, txt):
#         is_idx = (txt == "index")
#         self.idx_label.setVisible(is_idx)
#         self.idx_spin.setVisible(is_idx)

#     def _reduce_vector(self, vec, how: str):
#         if vec is None:
#             return float("nan")
#         # ensure 1D float array
#         arr = np.asarray(vec, dtype=float).ravel()
#         if arr.size == 0:
#             return float("nan")
#         if how == "mean":
#             return float(np.nanmean(arr))
#         if how == "rms":
#             return float(np.sqrt(np.nanmean(arr * arr)))
#         if how == "index":
#             i = self.idx_spin.value()
#             return float(arr[i]) if 0 <= i < arr.size else float("nan")
#         return float("nan")

#     
    def _switch_plot_mode(self):
        is2d = self.mode_2d.isChecked()
        if hasattr(self, "image"):
            self.image.setVisible(is2d)
        if hasattr(self, "plot"):
            self.plot.setVisible(not is2d)
        # Disable reducer/index controls in 2D; they're 1D-only
        self.reducer.setEnabled(not is2d)
        self.idx_spin.setEnabled(not is2d and self.reducer.currentText() == "index")
        self.idx_label.setEnabled(not is2d)
        # Y-axis controls apply only to 1D
        self.auto_y.setEnabled(not is2d)
        self.ymin.setEnabled(not is2d and not self.auto_y.isChecked())
        self.ymax.setEnabled(not is2d and not self.auto_y.isChecked())
        self.apply_y.setEnabled(not is2d and not self.auto_y.isChecked())

    def _switch_psd_mode(self):
        # Update axis label when toggling PSD in 1D mode
        if self.mode_psd.isChecked() and not self.mode_2d.isChecked():
            self.plot.setLabel('bottom', 'Frequency', units='Hz')
        else:
            self.plot.setLabel('bottom', 'Time', units='s')

    def _vector_to_2d(self, name: str, vec):
        """Safely turn a telemetry vector into a 2D array for display."""
        arr = np.asarray(vec, dtype=float).ravel()
        if name == "img" and arr.size == 32*32:
            return arr.reshape(32, 32)
        if arr.size == 12*12:
            return arr.reshape(12, 12)
        if arr.size == 140:
            return get_DM_command_in_2D(arr, Nx_act=12)
        n = int(np.sqrt(arr.size))
        if n*n == arr.size and n >= 2:
            return arr.reshape(n, n)
        raise ValueError(f"Cannot reshape '{name}' of length {arr.size} into 2D")

    def _psd_1d(self):
        """Compute and draw 1D PSD of the current y-history (ignore NaNs)."""
        y = np.array([v for v in self.y if np.isfinite(v)], dtype=float)
        n = y.size
        if n < 8:
            self.curve.setData([], [])
            return
        # Estimate sampling interval from timestamps; fallback to 10 Hz
        if len(self.x) >= 2:
            dts = np.diff(list(self.x))
            dT = float(np.median(dts))
            fs = 1.0 / dT if dT > 0 else 10.0
        else:
            fs = 10.0
        y = y - np.nanmean(y)
        Y = np.fft.rfft(y)
        P = (np.abs(Y) ** 2) / n
        f = np.fft.rfftfreq(n, d=1.0 / fs)
        self.curve.setData(f, P)

    def _psd_2d(self, img2d):
        """Return 2D power spectrum (fftshifted) of image."""
        a = np.nan_to_num(img2d, nan=0.0, posinf=0.0, neginf=0.0)
        # Hann window to reduce edge effects
        win_x = np.hanning(a.shape[1])
        win_y = np.hanning(a.shape[0])
        w = np.outer(win_y, win_x)
        A = np.fft.fftshift(np.fft.fft2(a * w))
        return np.log10(np.abs(A) ** 2 + 1e-12)

    def _update_rate_changed(self, hz):
        try:
            hz = float(hz)
            if hz <= 0:
                return
            self.update_hz = hz
            interval = int(max(1, round(1000.0 / hz)))
            self.timer.setInterval(interval)
        except Exception:
            pass

    def update_plot(self):

        t = time.time()
        name = self.field.currentText()
        # 2D mode: render latest frame as image (or its 2D PSD)
        if self.mode_2d.isChecked():
            try:
                if name not in self.VECTOR_FIELDS:
                    return  # 2D only for vectors
                res = self.client.poll_vector(name)
                if not res.get("ok"):
                    return
                img2d = self._vector_to_2d(name, res.get("data", []))
                if self.mode_psd.isChecked():
                    img2d = self._psd_2d(img2d)
                # transpose for conventional orientation
                self.image.setImage(img2d.T, autoLevels=True)
            except Exception:
                return
            return

        # 1D mode: time series or PSD of reduced values
        val = float("nan")
        try:
            if name in self.SCALAR_FIELDS:
                res = self.client.poll_scalar(name)
                if res.get("ok"):
                    val = float(res.get("data", float("nan")))
            else:
                res = self.client.poll_vector(name)
                if res.get("ok"):
                    val = self._reduce_vector(res.get("data", []), self.reducer.currentText())
        except Exception:
            pass  # keep plotting NaNs on transient errors

        self.x.append(t); self.y.append(val)
        if self.mode_psd.isChecked():
            self._psd_1d()
        else:
            self.curve.setData(list(self.x), list(self.y))

    def _switch_plot_mode(self):
        is2d = self.mode_2d.isChecked()
        if hasattr(self, "image"):
            self.image.setVisible(is2d)
        if hasattr(self, "plot"):
            self.plot.setVisible(not is2d)
        # Disable reducer/index controls in 2D; they're 1D-only
        self.reducer.setEnabled(not is2d)
        self.idx_spin.setEnabled(not is2d and self.reducer.currentText() == "index")
        self.idx_label.setEnabled(not is2d)
        # Y-axis controls apply only to 1D
        self.auto_y.setEnabled(not is2d)
        self.ymin.setEnabled(not is2d and not self.auto_y.isChecked())
        self.ymax.setEnabled(not is2d and not self.auto_y.isChecked())
        self.apply_y.setEnabled(not is2d and not self.auto_y.isChecked())

    def _switch_psd_mode(self):
        # Update axis label when toggling PSD in 1D mode
        if self.mode_psd.isChecked() and not self.mode_2d.isChecked():
            self.plot.setLabel('bottom', 'Frequency', units='Hz')
        else:
            self.plot.setLabel('bottom', 'Time', units='s')

    def _vector_to_2d(self, name: str, vec):
        """Safely turn a telemetry vector into a 2D array for display."""
        arr = np.asarray(vec, dtype=float).ravel()
        if name == "img" and arr.size == 32*32:
            return arr.reshape(32, 32)
        if arr.size == 12*12:
            return arr.reshape(12, 12)
        if arr.size == 140:
            return get_DM_command_in_2D(arr, Nx_act=12)
        n = int(np.sqrt(arr.size))
        if n*n == arr.size and n >= 2:
            return arr.reshape(n, n)
        raise ValueError(f"Cannot reshape '{name}' of length {arr.size} into 2D")

    def _psd_1d(self):
        """Compute and draw 1D PSD of the current y-history (ignore NaNs)."""
        y = np.array([v for v in self.y if np.isfinite(v)], dtype=float)
        n = y.size
        if n < 8:
            self.curve.setData([], [])
            return
        # Estimate sampling interval from timestamps; fallback to 10 Hz
        if len(self.x) >= 2:
            dts = np.diff(list(self.x))
            dT = float(np.median(dts))
            fs = 1.0 / dT if dT > 0 else 10.0
        else:
            fs = 10.0
        y = y - np.nanmean(y)
        Y = np.fft.rfft(y)
        P = (np.abs(Y) ** 2) / n
        f = np.fft.rfftfreq(n, d=1.0 / fs)
        self.curve.setData(f, P)

    def _psd_2d(self, img2d):
        """Return 2D power spectrum (fftshifted) of image."""
        a = np.nan_to_num(img2d, nan=0.0, posinf=0.0, neginf=0.0)
        # Hann window to reduce edge effects
        win_x = np.hanning(a.shape[1])
        win_y = np.hanning(a.shape[0])
        w = np.outer(win_y, win_x)
        A = np.fft.fftshift(np.fft.fft2(a * w))
        return np.log10(np.abs(A) ** 2 + 1e-12)

    def _update_rate_changed(self, hz):
        try:
            hz = float(hz)
            if hz <= 0:
                return
            self.update_hz = hz
            interval = int(max(1, round(1000.0 / hz)))
            self.timer.setInterval(interval)
        except Exception:
            pass

    def update_plot(self):

        t = time.time()
        name = self.field.currentText()
        # 2D mode: render latest frame as image (or its 2D PSD)
        if self.mode_2d.isChecked():
            try:
                if name not in self.VECTOR_FIELDS:
                    return  # 2D only for vectors
                res = self.client.poll_vector(name)
                if not res.get("ok"):
                    return
                img2d = self._vector_to_2d(name, res.get("data", []))
                if self.mode_psd.isChecked():
                    img2d = self._psd_2d(img2d)
                # transpose for conventional orientation
                self.image.setImage(img2d.T, autoLevels=True)
            except Exception:
                return
            return

        # 1D mode: time series or PSD of reduced values
        val = float("nan")
        try:
            if name in self.SCALAR_FIELDS:
                res = self.client.poll_scalar(name)
                if res.get("ok"):
                    val = float(res.get("data", float("nan")))
            else:
                res = self.client.poll_vector(name)
                if res.get("ok"):
                    val = self._reduce_vector(res.get("data", []), self.reducer.currentText())
        except Exception:
            pass  # keep plotting NaNs on transient errors

        self.x.append(t); self.y.append(val)
        if self.mode_psd.isChecked():
            self._psd_1d()
        else:
            self.curve.setData(list(self.x), list(self.y))

    def _switch_plot_mode(self):
        is2d = self.mode_2d.isChecked()
        if hasattr(self, "image"):
            self.image.setVisible(is2d)
        if hasattr(self, "plot"):
            self.plot.setVisible(not is2d)
        # Disable reducer/index controls in 2D; they're 1D-only
        self.reducer.setEnabled(not is2d)
        self.idx_spin.setEnabled(not is2d and self.reducer.currentText() == "index")
        self.idx_label.setEnabled(not is2d)
        # Y-axis controls apply only to 1D
        self.auto_y.setEnabled(not is2d)
        self.ymin.setEnabled(not is2d and not self.auto_y.isChecked())
        self.ymax.setEnabled(not is2d and not self.auto_y.isChecked())
        self.apply_y.setEnabled(not is2d and not self.auto_y.isChecked())

    def _switch_psd_mode(self):
        # Update axis label when toggling PSD in 1D mode
        if self.mode_psd.isChecked() and not self.mode_2d.isChecked():
            self.plot.setLabel('bottom', 'Frequency', units='Hz')
        else:
            self.plot.setLabel('bottom', 'Time', units='s')

    def _vector_to_2d(self, name: str, vec):
        """Safely turn a telemetry vector into a 2D array for display."""
        arr = np.asarray(vec, dtype=float).ravel()
        if name == "img" and arr.size == 32*32:
            return arr.reshape(32, 32)
        if arr.size == 12*12:
            return arr.reshape(12, 12)
        if arr.size == 140:
            return get_DM_command_in_2D(arr, Nx_act=12)
        n = int(np.sqrt(arr.size))
        if n*n == arr.size and n >= 2:
            return arr.reshape(n, n)
        raise ValueError(f"Cannot reshape '{name}' of length {arr.size} into 2D")

    def _psd_1d(self):
        """Compute and draw 1D PSD of the current y-history (ignore NaNs)."""
        y = np.array([v for v in self.y if np.isfinite(v)], dtype=float)
        n = y.size
        if n < 8:
            self.curve.setData([], [])
            return
        # Estimate sampling interval from timestamps; fallback to 10 Hz
        if len(self.x) >= 2:
            dts = np.diff(list(self.x))
            dT = float(np.median(dts))
            fs = 1.0 / dT if dT > 0 else 10.0
        else:
            fs = 10.0
        y = y - np.nanmean(y)
        Y = np.fft.rfft(y)
        P = (np.abs(Y) ** 2) / n
        f = np.fft.rfftfreq(n, d=1.0 / fs)
        self.curve.setData(f, P)

    def _psd_2d(self, img2d):
        """Return 2D power spectrum (fftshifted) of image."""
        a = np.nan_to_num(img2d, nan=0.0, posinf=0.0, neginf=0.0)
        # Hann window to reduce edge effects
        win_x = np.hanning(a.shape[1])
        win_y = np.hanning(a.shape[0])
        w = np.outer(win_y, win_x)
        A = np.fft.fftshift(np.fft.fft2(a * w))
        return np.log10(np.abs(A) ** 2 + 1e-12)

    def _update_rate_changed(self, hz):
        try:
            hz = float(hz)
            if hz <= 0:
                return
            self.update_hz = hz
            interval = int(max(1, round(1000.0 / hz)))
            self.timer.setInterval(interval)
        except Exception:
            pass

    def update_plot(self):

        t = time.time()
        name = self.field.currentText()
        # 2D mode: render latest frame as image (or its 2D PSD)
        if self.mode_2d.isChecked():
            try:
                if name not in self.VECTOR_FIELDS:
                    return  # 2D only for vectors
                res = self.client.poll_vector(name)
                if not res.get("ok"):
                    return
                img2d = self._vector_to_2d(name, res.get("data", []))
                if self.mode_psd.isChecked():
                    img2d = self._psd_2d(img2d)
                # transpose for conventional orientation
                self.image.setImage(img2d.T, autoLevels=True)
            except Exception:
                return
            return

        # 1D mode: time series or PSD of reduced values
        val = float("nan")
        try:
            if name in self.SCALAR_FIELDS:
                res = self.client.poll_scalar(name)
                if res.get("ok"):
                    val = float(res.get("data", float("nan")))
            else:
                res = self.client.poll_vector(name)
                if res.get("ok"):
                    val = self._reduce_vector(res.get("data", []), self.reducer.currentText())
        except Exception:
            pass  # keep plotting NaNs on transient errors

        self.x.append(t); self.y.append(val)
        if self.mode_psd.isChecked():
            self._psd_1d()
        else:
            self.curve.setData(list(self.x), list(self.y))

    def _switch_plot_mode(self):
        is2d = self.mode_2d.isChecked()
        if hasattr(self, "image"):
            self.image.setVisible(is2d)
        if hasattr(self, "plot"):
            self.plot.setVisible(not is2d)
        # Disable reducer/index controls in 2D; they're 1D-only
        self.reducer.setEnabled(not is2d)
        self.idx_spin.setEnabled(not is2d and self.reducer.currentText() == "index")
        self.idx_label.setEnabled(not is2d)
        # Y-axis controls apply only to 1D
        self.auto_y.setEnabled(not is2d)
        self.ymin.setEnabled(not is2d and not self.auto_y.isChecked())
        self.ymax.setEnabled(not is2d and not self.auto_y.isChecked())
        self.apply_y.setEnabled(not is2d and not self.auto_y.isChecked())

    def _switch_psd_mode(self):
        # Update axis label when toggling PSD in 1D mode
        if self.mode_psd.isChecked() and not self.mode_2d.isChecked():
            self.plot.setLabel('bottom', 'Frequency', units='Hz')
        else:
            self.plot.setLabel('bottom', 'Time', units='s')

    def _vector_to_2d(self, name: str, vec):
        """Safely turn a telemetry vector into a 2D array for display."""
        arr = np.asarray(vec, dtype=float).ravel()
        if name == "img" and arr.size == 32*32:
            return arr.reshape(32, 32)
        if arr.size == 12*12:
            return arr.reshape(12, 12)
        if arr.size == 140:
            return get_DM_command_in_2D(arr, Nx_act=12)
        n = int(np.sqrt(arr.size))
        if n*n == arr.size and n >= 2:
            return arr.reshape(n, n)
        raise ValueError(f"Cannot reshape '{name}' of length {arr.size} into 2D")

    def _psd_1d(self):
        """Compute and draw 1D PSD of the current y-history (ignore NaNs)."""
        y = np.array([v for v in self.y if np.isfinite(v)], dtype=float)
        n = y.size
        if n < 8:
            self.curve.setData([], [])
            return
        # Estimate sampling interval from timestamps; fallback to 10 Hz
        if len(self.x) >= 2:
            dts = np.diff(list(self.x))
            dT = float(np.median(dts))
            fs = 1.0 / dT if dT > 0 else 10.0
        else:
            fs = 10.0
        y = y - np.nanmean(y)
        Y = np.fft.rfft(y)
        P = (np.abs(Y) ** 2) / n
        f = np.fft.rfftfreq(n, d=1.0 / fs)
        self.curve.setData(f, P)

    def _psd_2d(self, img2d):
        """Return 2D power spectrum (fftshifted) of image."""
        a = np.nan_to_num(img2d, nan=0.0, posinf=0.0, neginf=0.0)
        # Hann window to reduce edge effects
        win_x = np.hanning(a.shape[1])
        win_y = np.hanning(a.shape[0])
        w = np.outer(win_y, win_x)
        A = np.fft.fftshift(np.fft.fft2(a * w))
        return np.log10(np.abs(A) ** 2 + 1e-12)

    def _update_rate_changed(self, hz):
        try:
            hz = float(hz)
            if hz <= 0:
                return
            self.update_hz = hz
            interval = int(max(1, round(1000.0 / hz)))
            self.timer.setInterval(interval)
        except Exception:
            pass

    def update_plot(self):

        t = time.time()
        name = self.field.currentText()
        # 2D mode: render latest frame as image (or its 2D PSD)
        if self.mode_2d.isChecked():
            try:
                if name not in self.VECTOR_FIELDS:
                    return  # 2D only for vectors
                res = self.client.poll_vector(name)
                if not res.get("ok"):
                    return
                img2d = self._vector_to_2d(name, res.get("data", []))
                if self.mode_psd.isChecked():
                    img2d = self._psd_2d(img2d)
                # transpose for conventional orientation
                self.image.setImage(img2d.T, autoLevels=True)
            except Exception:
                return
            return

        # 1D mode: time series or PSD of reduced values
        val = float("nan")
        try:
            if name in self.SCALAR_FIELDS:
                res = self.client.poll_scalar(name)
                if res.get("ok"):
                    val = float(res.get("data", float("nan")))
            else:
                res = self.client.poll_vector(name)
                if res.get("ok"):
                    val = self._reduce_vector(res.get("data", []), self.reducer.currentText())
        except Exception:
            pass  # keep plotting NaNs on transient errors

        self.x.append(t); self.y.append(val)
        if self.mode_psd.isChecked():
            self._psd_1d()
        else:
            self.curve.setData(list(self.x), list(self.y))

    def _switch_plot_mode(self):
        is2d = self.mode_2d.isChecked()
        if hasattr(self, "image"):
            self.image.setVisible(is2d)
        if hasattr(self, "plot"):
            self.plot.setVisible(not is2d)
        # Disable reducer/index controls in 2D; they're 1D-only
        self.reducer.setEnabled(not is2d)
        self.idx_spin.setEnabled(not is2d and self.reducer.currentText() == "index")
        self.idx_label.setEnabled(not is2d)
        # Y-axis controls apply only to 1D
        self.auto_y.setEnabled(not is2d)
        self.ymin.setEnabled(not is2d and not self.auto_y.isChecked())
        self.ymax.setEnabled(not is2d and not self.auto_y.isChecked())
        self.apply_y.setEnabled(not is2d and not self.auto_y.isChecked())

    def _switch_psd_mode(self):
        # Update axis label when toggling PSD in 1D mode
        if self.mode_psd.isChecked() and not self.mode_2d.isChecked():
            self.plot.setLabel('bottom', 'Frequency', units='Hz')
        else:
            self.plot.setLabel('bottom', 'Time', units='s')

    def _vector_to_2d(self, name: str, vec):
        """Safely turn a telemetry vector into a 2D array for display."""
        arr = np.asarray(vec, dtype=float).ravel()
        if name == "img" and arr.size == 32*32:
            return arr.reshape(32, 32)
        if arr.size == 12*12:
            return arr.reshape(12, 12)
        if arr.size == 140:
            return get_DM_command_in_2D(arr, Nx_act=12)
        n = int(np.sqrt(arr.size))
        if n*n == arr.size and n >= 2:
            return arr.reshape(n, n)
        raise ValueError(f"Cannot reshape '{name}' of length {arr.size} into 2D")

    def _psd_1d(self):
        """Compute and draw 1D PSD of the current y-history (ignore NaNs)."""
        y = np.array([v for v in self.y if np.isfinite(v)], dtype=float)
        n = y.size
        if n < 8:
            self.curve.setData([], [])
            return
        # Estimate sampling interval from timestamps; fallback to 10 Hz
        if len(self.x) >= 2:
            dts = np.diff(list(self.x))
            dT = float(np.median(dts))
            fs = 1.0 / dT if dT > 0 else 10.0
        else:
            fs = 10.0
        y = y - np.nanmean(y)
        Y = np.fft.rfft(y)
        P = (np.abs(Y) ** 2) / n
        f = np.fft.rfftfreq(n, d=1.0 / fs)
        self.curve.setData(f, P)

    def _psd_2d(self, img2d):
        """Return 2D power spectrum (fftshifted) of image."""
        a = np.nan_to_num(img2d, nan=0.0, posinf=0.0, neginf=0.0)
        # Hann window to reduce edge effects
        win_x = np.hanning(a.shape[1])
        win_y = np.hanning(a.shape[0])
        w = np.outer(win_y, win_x)
        A = np.fft.fftshift(np.fft.fft2(a * w))
        return np.log10(np.abs(A) ** 2 + 1e-12)

    def _update_rate_changed(self, hz):
        try:
            hz = float(hz)
            if hz <= 0:
                return
            self.update_hz = hz
            interval = int(max(1, round(1000.0 / hz)))
            self.timer.setInterval(interval)
        except Exception:
            pass

    def update_plot(self):

        t = time.time()
        name = self.field.currentText()
        # 2D mode: render latest frame as image (or its 2D PSD)
        if self.mode_2d.isChecked():
            try:
                if name not in self.VECTOR_FIELDS:
                    return  # 2D only for vectors
                res = self.client.poll_vector(name)
                if not res.get("ok"):
                    return
                img2d = self._vector_to_2d(name, res.get("data", []))
                if self.mode_psd.isChecked():
                    img2d = self._psd_2d(img2d)
                # transpose for conventional orientation
                self.image.setImage(img2d.T, autoLevels=True)
            except Exception:
                return
            return

        # 1D mode: time series or PSD of reduced values
        val = float("nan")
        try:
            if name in self.SCALAR_FIELDS:
                res = self.client.poll_scalar(name)
                if res.get("ok"):
                    val = float(res.get("data", float("nan")))
            else:
                res = self.client.poll_vector(name)
                if res.get("ok"):
                    val = self._reduce_vector(res.get("data", []), self.reducer.currentText())
        except Exception:
            pass  # keep plotting NaNs on transient errors

        self.x.append(t); self.y.append(val)
        if self.mode_psd.isChecked():
            self._psd_1d()
        else:
            self.curve.setData(list(self.x), list(self.y))