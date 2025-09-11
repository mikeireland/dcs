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

# -------------------- Plot widget --------------------
class PlotWidget(QWidget):
    new_plot_requested = pyqtSignal()

    SCALAR_FIELDS = ["snr", "rmse_est"]  # extend if you add more
    VECTOR_FIELDS = ["img", "img_dm", "signal", "e_LO", "u_LO", "e_HO", "u_HO", "c_LO", "c_HO", "c_inj"]
    REDUCERS = ["mean", "rms", "index"]

    def __init__(self, server_addr, history=600):
        super().__init__()
        self.client = CommanderClient(server_addr)
        self.server_addr = server_addr
        self.history = history
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

        top.addWidget(QLabel("Field:")); top.addWidget(self.field); top.addSpacing(10)
        top.addWidget(QLabel("Reducer:")); top.addWidget(self.reducer)
        top.addWidget(self.idx_label); top.addWidget(self.idx_spin)
        top.addStretch(); top.addWidget(self.new_plot_button)

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

    def update_plot(self):
        t = time.time()
        name = self.field.currentText()
        val = float("nan")
        try:
            if name in self.SCALAR_FIELDS:
                res = self.client.poll_scalar(name)
                if res.get("ok"):
                    val = float(res["data"])
            else:
                res = self.client.poll_vector(name)
                if res.get("ok"):
                    val = self._reduce_vector(res.get("data", []), self.reducer.currentText())
        except Exception:
            pass

        self.x.append(t); self.y.append(val)
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
        self.cli = CommandLine(self.server_addr)
        self.ctrl = ControlPanel(self.cli)
        left.addWidget(self.ctrl)
        left.addWidget(self.cli)

        self.plot = PlotWidget(self.server_addr)
        self.plot.new_plot_requested.connect(self.spawn_new_plot)

        main.addLayout(left, 1)
        main.addWidget(self.plot, 2)
        self.setLayout(main)

    def spawn_new_plot(self):
        w = PlotWidget(self.server_addr)
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

#     def update_plot(self):
#         t = time.time()
#         name = self.field.currentText()
#         val = float("nan")

#         try:
#             if name in self.SCALAR_FIELDS:
#                 res = self.client.poll_scalar(name)
#                 if res.get("ok"):
#                     val = float(res["data"])
#             else:
#                 res = self.client.poll_vector(name)
#                 if res.get("ok"):
#                     val = self._reduce_vector(res.get("data", []), self.reducer.currentText())
#         except Exception:
#             # keep plotting NANs on transient errors
#             pass

#         self.x.append(t); self.y.append(val)
#         self.curve.setData(list(self.x), list(self.y))

# # -------------------- CLI --------------------
# class CommandLine(QWidget):
#     def __init__(self, server_addr=None):
#         super().__init__()
#         self.client = CommanderClient(server_addr or SERVER_ADDR_DICT[args.beam])
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
#             # Keep user JSON verbatim after the first token
#             parts = text.split(maxsplit=1)
#             msg = parts[0] if len(parts) == 1 else f"{parts[0]} {parts[1]}"
#             reply = self.client.send_raw(msg)
#         except Exception as e:
#             reply = f"[ZMQ error] {e}"
#         self.history.append(reply)

# # -------------------- Control panel --------------------
# class ControlPanel(QWidget):
#     def __init__(self, cli_widget):
#         super().__init__()
#         self.cli = cli_widget
#         layout = QGridLayout()

#         def send_cmd(text: str):
#             self.cli.prompt.setText(text)
#             self.cli.handle_command()

#         buttons = [
#             ("Close LO", partial(send_cmd, 'close_baldr_LO ""')),
#             ("Open LO",  partial(send_cmd, 'open_baldr_LO ""')),
#             ("Close HO", partial(send_cmd, 'close_baldr_HO ""')),
#             ("Open HO",  partial(send_cmd, 'open_baldr_HO ""')),
#         ]
#         for i, (label, fn) in enumerate(buttons):
#             btn = QPushButton(label); btn.clicked.connect(fn)
#             layout.addWidget(btn, i // 2, i % 2)

#         self.setLayout(layout)

# # -------------------- Main window --------------------
# class MainWindow(QWidget):
#     def __init__(self, server_addr=None):
#         super().__init__()
#         self.setWindowTitle("Baldr Telemetry GUI")
#         self.server_addr = server_addr or SERVER_ADDR_DICT[args.beam]
#         self.plot_windows = []
#         self._init_ui()

#     def _init_ui(self):
#         main = QHBoxLayout()
#         left = QVBoxLayout()
#         self.cli = CommandLine(self.server_addr)
#         self.ctrl = ControlPanel(self.cli)
#         left.addWidget(self.ctrl)
#         left.addWidget(self.cli)

#         self.plot = PlotWidget(self.server_addr)
#         self.plot.new_plot_requested.connect(self.spawn_new_plot)

#         main.addLayout(left, 1)
#         main.addWidget(self.plot, 2)
#         self.setLayout(main)

#     def spawn_new_plot(self):
#         w = PlotWidget(self.server_addr)
#         w.setWindowTitle("Telemetry Plot")
#         w.resize(500, 300)
#         w.new_plot_requested.connect(self.spawn_new_plot)
#         w.show()
#         self.plot_windows.append(w)

# # -------------------- Run --------------------
# def run_app():
#     app = QApplication(sys.argv)
#     win = MainWindow()
#     win.resize(1100, 650)
#     win.show()
#     sys.exit(app.exec_())

# if __name__ == "__main__":
#     run_app()


# # import sys, time, json, math, zmq
# # from collections import deque
# # import numpy as np
# # import argparse
# # from PyQt5.QtWidgets import (
# #     QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
# #     QComboBox, QTextEdit, QLineEdit, QGridLayout, QLabel, QSpinBox
# # )
# # from PyQt5.QtCore import QTimer, pyqtSignal
# # import pyqtgraph as pg

# # from functools import partial




# # parser = argparse.ArgumentParser(description="Baldr RTC GUI.")

# # parser.add_argument("--beam", type=int, default=1, help="which beam")

# # args=parser.parse_args()

# # SERVER_ADDR_DICT = {1,"tcp://127.0.0.1:6662",
# #                     2,"tcp://127.0.0.1:6663",
# #                     3,"tcp://127.0.0.1:6664",
# #                     4,"tcp://127.0.0.1:6665"}

# # # -------------------- Commander ZMQ helper --------------------
# # class CommanderClient:
# #     def __init__(self, addr=SERVER_ADDR_DICT[args.beam], rcv_timeout_ms=1000, snd_timeout_ms=1000):
# #         self.ctx = zmq.Context.instance()
# #         self.sock = self.ctx.socket(zmq.REQ)
# #         self.sock.setsockopt(zmq.RCVTIMEO, rcv_timeout_ms)
# #         self.sock.setsockopt(zmq.SNDTIMEO, snd_timeout_ms)
# #         self.sock.connect(addr)

# #     def call(self, cmd, *args):
# #         if len(args) == 0:
# #             msg = cmd
# #         elif len(args) == 1:
# #             msg = f"{cmd} {json.dumps(args[0])}"     # single param -> JSON string
# #         else:
# #             msg = f"{cmd} {json.dumps(list(args))}"  # multi -> JSON array
# #         self.sock.send_string(msg)
# #         reply = self.sock.recv_string()
# #         try:
# #             return json.loads(reply)
# #         except json.JSONDecodeError:
# #             return {"ok": False, "error": "non-JSON reply", "raw": reply}

# #     # Convenience wrappers
# #     def poll_scalar(self, name: str):
# #         return self.call("poll_telem_scalar", name)

# #     def poll_vector(self, name: str):
# #         return self.call("poll_telem_vector", name)

# #     def send_raw(self, text: str) -> str:
# #         try:
# #             self.sock.send_string(text)
# #             return self.sock.recv_string()
# #         except zmq.error.Again:
# #             # timeout on send or recv
# #             self._reset()
# #             return '[timeout]'
# #         except zmq.ZMQError as e:
# #             # EFSM = wrong state (e.g., double-send without recv)
# #             if e.errno == zmq.EFSM:
# #                 self._reset()
# #                 return '[efsm: socket resynced]'
# #             raise
# # # -------------------- Plot widget --------------------
# # class PlotWidget(QWidget):
# #     new_plot_requested = pyqtSignal()

# #     SCALAR_FIELDS = ["snr", "rmse_est"]  # extend if you add more
# #     VECTOR_FIELDS = ["img", "img_dm", "signal", "e_LO", "u_LO", "e_HO", "u_HO", "c_LO", "c_HO", "c_inj"]
# #     REDUCERS = ["mean", "rms", "index"]

# #     def __init__(self, server_addr=SERVER_ADDR_DICT[args.beam], history=600):  # keep ~1 min at 10 Hz
# #         super().__init__()
# #         self.client = CommanderClient(server_addr)
# #         self.history = history
# #         self.x = deque(maxlen=self.history)
# #         self.y = deque(maxlen=self.history)
# #         self._init_ui()

# #         self.timer = QTimer(self)
# #         self.timer.timeout.connect(self.update_plot)
# #         self.timer.start(100)  # 10 Hz

# #     def _init_ui(self):
# #         layout = QVBoxLayout()

# #         # Controls row
# #         top = QHBoxLayout()
# #         self.field = QComboBox()
# #         self.field.addItems(self.SCALAR_FIELDS + self.VECTOR_FIELDS)

# #         self.reducer = QComboBox()
# #         self.reducer.addItems(self.REDUCERS)
# #         self.reducer.setCurrentText("mean")

# #         self.idx_label = QLabel("idx:")
# #         self.idx_spin = QSpinBox()
# #         self.idx_spin.setRange(0, 100000)
# #         self.idx_spin.setValue(0)

# #         self.new_plot_button = QPushButton("+")
# #         self.new_plot_button.setFixedSize(25, 25)
# #         self.new_plot_button.clicked.connect(self.new_plot_requested.emit)

# #         top.addWidget(QLabel("Field:"))
# #         top.addWidget(self.field)
# #         top.addSpacing(10)
# #         top.addWidget(QLabel("Reducer:"))
# #         top.addWidget(self.reducer)
# #         top.addWidget(self.idx_label)
# #         top.addWidget(self.idx_spin)
# #         top.addStretch()
# #         top.addWidget(self.new_plot_button)

# #         # Plot
# #         self.plot = pg.PlotWidget()
# #         self.plot.showGrid(x=True, y=True, alpha=0.3)
# #         self.curve = self.plot.plot()

# #         layout.addLayout(top)
# #         layout.addWidget(self.plot)
# #         self.setLayout(layout)

# #         # Toggle index input visibility based on reducer
# #         self.reducer.currentTextChanged.connect(self._toggle_idx_visibility)
# #         self._toggle_idx_visibility(self.reducer.currentText())

# #     def _toggle_idx_visibility(self, txt):
# #         is_idx = (txt == "index")
# #         self.idx_label.setVisible(is_idx)
# #         self.idx_spin.setVisible(is_idx)

# #     def _reduce_vector(self, vec, how: str):
# #         if not vec:
# #             return float("nan")
# #         if how == "mean":
# #             return float(np.mean(vec))
# #         if how == "rms":
# #             arr = np.asarray(vec, dtype=float)
# #             return float(np.sqrt(np.mean(arr * arr)))
# #         if how == "index":
# #             i = self.idx_spin.value()
# #             if 0 <= i < len(vec):
# #                 return float(vec[i])
# #             return float("nan")
# #         return float("nan")

# #     def update_plot(self):
# #         t = time.time()
# #         name = self.field.currentText()
# #         if name in self.SCALAR_FIELDS:
# #             res = self.client.poll_scalar(name)
# #             val = float(res["data"]) if res.get("ok") else float("nan")
# #         else:
# #             res = self.client.poll_vector(name)
# #             if res.get("ok"):
# #                 vec = res.get("data", [])
# #                 val = self._reduce_vector(vec, self.reducer.currentText())
# #             else:
# #                 val = float("nan")

# #         self.x.append(t)
# #         self.y.append(val)
# #         self.curve.setData(list(self.x), list(self.y))

# # # -------------------- CLI (uses its own REQ socket) --------------------
# # class CommandLine(QWidget):
# #     def __init__(self, server_addr=SERVER_ADDR_DICT[args.beam]):
# #         super().__init__()
# #         self.client = CommanderClient(server_addr)
# #         self._init_ui()

# #     def _init_ui(self):
# #         layout = QVBoxLayout()
# #         self.history = QTextEdit(); self.history.setReadOnly(True)
# #         self.prompt = QLineEdit();  self.prompt.returnPressed.connect(self.handle_command)
# #         layout.addWidget(self.history); layout.addWidget(self.prompt)
# #         self.setLayout(layout)

# #     # def handle_command(self):
# #     #     text = self.prompt.text()
# #     #     if not text.strip():
# #     #         return
# #     #     self.history.append(f"> {text}")
# #     #     self.prompt.clear()
# #     #     try:
# #     #         reply = self.client.send_raw(text)   # <-- exactly what you typed
# #     #     except Exception as e:
# #     #         reply = f"[ZMQ error] {e}"
# #     #     self.history.append(reply)
        
    
# #     def handle_command(self):
# #         text = self.prompt.text().strip()
# #         if not text: return
# #         self.history.append(f"> {text}")
# #         self.prompt.clear()
# #         # Send raw command line: everything after first word must be valid JSON for your handler
# #         # e.g., poll_telem_scalar "snr"
# #         try:
# #             # Split command and (optional) JSON argument
# #             parts = text.split(maxsplit=1)
# #             if len(parts) == 1:
# #                 cmd, arg = parts[0], None
# #             else:
# #                 cmd, arg = parts[0], parts[1]
# #             if arg is None:
# #                 msg = cmd
# #             else:
# #                 # Keep user-typed JSON verbatim
# #                 msg = f"{cmd} {arg}"
# #             self.client.sock.send_string(msg)
# #             reply = self.client.sock.recv_string()
# #             self.history.append(reply)
# #         except Exception as e:
# #             self.history.append(f"[ZMQ error] {e}")

# # # -------------------- Control panel (stub) --------------------
# # class ControlPanel(QWidget):
# #     def __init__(self,cli_widget):
# #         super().__init__()
# #         self.cli = cli_widget  # <-- keep a handle to the CLI
# #         layout = QGridLayout()
        
        
        
# #         def send_cmd(text: str):
# #             self.cli.prompt.setText(text)
# #             self.cli.handle_command()  # uses your existing logic & socket

# #         buttons = [
# #             ("Close LO", partial(send_cmd, 'close_baldr_LO ""')),
# #             ("Open LO",  partial(send_cmd, 'open_baldr_LO ""')),
# #             ("Close HO", partial(send_cmd, 'close_baldr_HO ""')),
# #             ("Open HO",  partial(send_cmd, 'open_baldr_HO ""')),
# #         ]
# #         for i, (label, fn) in enumerate(buttons):
# #             btn = QPushButton(label)
# #             btn.clicked.connect(fn)
# #             layout.addWidget(btn, i // 2, i % 2)

# #         self.setLayout(layout)


# #         # for i, (label, fn) in enumerate({
# #         #     "Close LO": lambda: print("Close LO"),
# #         #     "Open LO":  lambda: print("Open LO"),
# #         #     "Close HO": lambda: print("Close HO"),
# #         #     "Open HO":  lambda: print("Open HO"),
# #         # }.items()):
# #         #     btn = QPushButton(label); btn.clicked.connect(fn)
# #         #     layout.addWidget(btn, i // 2, i % 2)
# #         # self.setLayout(layout)

# # # -------------------- Main window --------------------
# # class MainWindow(QWidget):
# #     def __init__(self, server_addr=SERVER_ADDR_DICT[args.beam]):
# #         super().__init__()
# #         self.setWindowTitle("Baldr Telemetry GUI")
# #         self.server_addr = server_addr
# #         self.plot_windows = []
# #         self._init_ui()

# #     def _init_ui(self):
# #         main = QHBoxLayout()

# #         left = QVBoxLayout()
# #         self.cli = CommandLine(self.server_addr)   # CLI first
# #         self.ctrl = ControlPanel(self.cli)         # pass CLI into ControlPanel
# #         left.addWidget(self.ctrl)
# #         left.addWidget(self.cli)

# #         self.plot = PlotWidget(self.server_addr)
# #         self.plot.new_plot_requested.connect(self.spawn_new_plot)

# #         main.addLayout(left, 1)
# #         main.addWidget(self.plot, 2)
# #         self.setLayout(main)
        
# #     # def _init_ui(self):
# #     #     main = QHBoxLayout()

# #     #     left = QVBoxLayout()
# #     #     left.addWidget(ControlPanel())
# #     #     self.cli = CommandLine(self.server_addr)
# #     #     left.addWidget(self.cli)

# #     #     self.plot = PlotWidget(self.server_addr)
# #     #     self.plot.new_plot_requested.connect(self.spawn_new_plot)

# #     #     main.addLayout(left, 1)
# #     #     main.addWidget(self.plot, 2)
# #     #     self.setLayout(main)


# #     def _init_ui(self):
# #         main = QHBoxLayout()

# #         left = QVBoxLayout()
# #         self.cli = CommandLine(self.server_addr)          # create CLI first
# #         self.ctrl = ControlPanel(self.cli)                # pass CLI into ControlPanel
# #         left.addWidget(self.ctrl)
# #         left.addWidget(self.cli)

# #         self.plot = PlotWidget(self.server_addr)
# #         self.plot.new_plot_requested.connect(self.spawn_new_plot)

# #         main.addLayout(left, 1)
# #         main.addWidget(self.plot, 2)
# #         self.setLayout(main)
# #     def spawn_new_plot(self):
# #         w = PlotWidget(self.server_addr)
# #         w.setWindowTitle("Telemetry Plot")
# #         w.resize(500, 300)
# #         w.new_plot_requested.connect(self.spawn_new_plot)
# #         w.show()
# #         self.plot_windows.append(w)

# # # -------------------- Run --------------------
# # def run_app():
# #     app = QApplication(sys.argv)
# #     win = MainWindow()
# #     win.resize(1100, 650)
# #     win.show()
# #     sys.exit(app.exec_())

# # if __name__ == "__main__":
# #     run_app()


# # # import sys
# # # import time
# # # import zmq 
# # # import numpy as np
# # # from PyQt5.QtWidgets import (
# # #     QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
# # #     QComboBox, QTextEdit, QLineEdit, QGridLayout
# # # )
# # # from PyQt5.QtCore import QTimer, pyqtSignal
# # # import pyqtgraph as pg


# # # # Create global ZMQ client socket and context once
# # # _zmq_context = zmq.Context.instance()
# # # _zmq_socket = _zmq_context.socket(zmq.REQ)
# # # _zmq_socket.connect("tcp://127.0.0.1:5555")  # Match the server address/port

# # # def poll_telemetry(signal_name, socket=None):
# # #     """
# # #     Poll telemetry using ZMQ REQ/REP protocol.
# # #     If ZMQ fails, fallback to dummy values.
# # #     """

# # #     try:
# # #         _zmq_socket.send_string(f"status {signal_name}")
# # #         response = _zmq_socket.recv_string()
# # #         return float(response)
# # #     except Exception as e:
# # #         print(f"[poll_telemetry] ZMQ error: {e}")
# # #         return 0.0

# # #     # try:
# # #     #     _zmq_socket.send_string(signal_name)
# # #     #     response = _zmq_socket.recv_string()
# # #     #     return float(response)
# # #     # except Exception as e:
# # #     #     print(f"[poll_telemetry] ZMQ error or timeout for '{signal_name}': {e}")
# # #     #     # Fallback to dummy values
# # #     #     # t = time.time()
# # #     #     # if signal_name == "Signal 1":
# # #     #     #     return np.sin(t)
# # #     #     # elif signal_name == "Signal 2":
# # #     #     #     return np.cos(t)
# # #     #     # elif signal_name == "Signal 3":
# # #     #     #     return np.sin(t * 2) * np.exp(-0.1 * t % 10)
# # #     #     # else:
# # #     #     #     return 0.0
        
# # # class PlotWidget(QWidget):
# # #     new_plot_requested = pyqtSignal()

# # #     def __init__(self):
# # #         super().__init__()
# # #         self.init_ui()

# # #     def init_ui(self):
# # #         layout = QVBoxLayout()

# # #         # Top bar with dropdown and "+" button
# # #         top_bar = QHBoxLayout()
# # #         self.combo = QComboBox()
# # #         self.combo.addItems(["Signal 1", "Signal 2", "Signal 3"])  # Stubbed signals
# # #         self.new_plot_button = QPushButton("+")
# # #         self.new_plot_button.setFixedSize(25, 25)
# # #         self.new_plot_button.clicked.connect(self.new_plot_requested.emit)

# # #         top_bar.addWidget(self.combo)
# # #         top_bar.addStretch()
# # #         top_bar.addWidget(self.new_plot_button)

# # #         # Plotting area
# # #         self.plot = pg.PlotWidget()
# # #         self.curve = self.plot.plot(pen='y')
# # #         self.x = []
# # #         self.y = []

# # #         layout.addLayout(top_bar)
# # #         layout.addWidget(self.plot)
# # #         self.setLayout(layout)

# # #         # Timer for updating the plot
# # #         self.timer = QTimer()
# # #         self.timer.timeout.connect(self.update_plot)
# # #         self.timer.start(100)

# # #     # def update_plot(self):
# # #     #     t = time.time()
# # #     #     self.x.append(t)
# # #     #     self.y.append(np.sin(t))  # Stub data
# # #     #     self.curve.setData(self.x[-100:], self.y[-100:])

# # #     def update_plot(self):
# # #         t = time.time()
# # #         self.x.append(t)

# # #         current_signal = self.combo.currentText()
# # #         y_val = poll_telemetry(current_signal, socket = _zmq_socket)

# # #         self.y.append(y_val)
# # #         self.curve.setData(self.x[-100:], self.y[-100:])

# # # class CommandLine(QWidget):
# # #     def __init__(self):
# # #         super().__init__()
# # #         self.init_ui()

# # #     def init_ui(self):
# # #         layout = QVBoxLayout()
# # #         self.history = QTextEdit()
# # #         self.history.setReadOnly(True)
# # #         self.prompt = QLineEdit()
# # #         self.prompt.returnPressed.connect(self.handle_command)
# # #         layout.addWidget(self.history)
# # #         layout.addWidget(self.prompt)
# # #         self.setLayout(layout)

# # #     # # def handle_command(self):
# # #     # #     text = self.prompt.text()
# # #     # #     self.history.append(f"> {text}")
# # #     # #     self.prompt.clear()
# # #     # def handle_command(self):
# # #     #     text = self.prompt.text()
# # #     #     self.history.append(f"> {text}")
# # #     #     self.prompt.clear()

# # #     #     try:
# # #     #         _zmq_command_socket.send_string(text)
# # #     #         response = _zmq_command_socket.recv_string()
# # #     #         self.history.append(response)
# # #     #     except Exception as e:
# # #     #         self.history.append(f"[ZMQ error] {e}")


# # #     def handle_command(self):
# # #         text = self.prompt.text()
# # #         self.history.append(f"> {text}")
# # #         self.prompt.clear()

# # #         try:
# # #             _zmq_socket.send_string(f"{text}")
# # #             response = _zmq_socket.recv_string()
# # #             self.history.append(response)
# # #         except Exception as e:
# # #             self.history.append(f"[ZMQ error] {e}")

# # # class ControlPanel(QWidget):
# # #     def __init__(self):
# # #         super().__init__()
# # #         self.init_ui()

# # #     def init_ui(self):
# # #         layout = QGridLayout()
# # #         buttons = {
# # #             "Close LO": self.close_lo,
# # #             "Close HO": self.close_ho,
# # #             "Open LO": self.open_lo,
# # #             "Open HO": self.open_ho
# # #         }
# # #         positions = [(0, 0), (1, 0), (0, 1), (1, 1)]
# # #         for (label, func), pos in zip(buttons.items(), positions):
# # #             btn = QPushButton(label)
# # #             btn.clicked.connect(func)
# # #             layout.addWidget(btn, *pos)
# # #         self.setLayout(layout)

# # #     def close_lo(self): print("Close LO clicked")
# # #     def close_ho(self): print("Close HO clicked")
# # #     def open_lo(self): print("Open LO clicked")
# # #     def open_ho(self): print("Open HO clicked")


# # # class MainWindow(QWidget):
# # #     def __init__(self):
# # #         super().__init__()
# # #         self.setWindowTitle("Real-Time Controller GUI")
# # #         self.plot_windows = []  # Manage references to all plot windows
# # #         self.init_ui()

# # #     def init_ui(self):
# # #         main_layout = QHBoxLayout()

# # #         # Left: Control Panel + CLI
# # #         left_layout = QVBoxLayout()
# # #         self.ctrl = ControlPanel()
# # #         self.cli = CommandLine()
# # #         left_layout.addWidget(self.ctrl)
# # #         left_layout.addWidget(self.cli)

# # #         # Right: Main PlotWidget
# # #         self.plot = PlotWidget()
# # #         self.plot.new_plot_requested.connect(self.spawn_new_plot)

# # #         main_layout.addLayout(left_layout, 1)
# # #         main_layout.addWidget(self.plot, 2)

# # #         self.setLayout(main_layout)

# # #     def spawn_new_plot(self):
# # #         new_plot = PlotWidget()
# # #         new_plot.setWindowTitle("New Plot Window")
# # #         new_plot.resize(500, 300)
# # #         new_plot.new_plot_requested.connect(self.spawn_new_plot)
# # #         new_plot.show()
# # #         self.plot_windows.append(new_plot)  # Prevent garbage collection


# # # def run_app():
# # #     app = QApplication(sys.argv)
# # #     win = MainWindow()
# # #     win.resize(1000, 600)
# # #     win.show()
# # #     sys.exit(app.exec_())


# # # if __name__ == "__main__":
# # #     run_app()


# # # # import sys
# # # # from PyQt5.QtWidgets import (
# # # #     QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
# # # #     QComboBox, QTextEdit, QLineEdit, QGridLayout
# # # # )
# # # # from PyQt5.QtCore import QTimer
# # # # import pyqtgraph as pg
# # # # import numpy as np
# # # # import time


# # # # class PlotWidget(QWidget):
# # # #     def __init__(self):
# # # #         super().__init__()
# # # #         self.init_ui()

# # # #     def init_ui(self):
# # # #         layout = QVBoxLayout()

# # # #         # Top bar with dropdown and "+" button
# # # #         top_bar = QHBoxLayout()
# # # #         self.combo = QComboBox()
# # # #         self.combo.addItems(["Signal 1", "Signal 2", "Signal 3"])  # Stub
# # # #         self.new_plot_button = QPushButton("+")
# # # #         self.new_plot_button.setFixedSize(25, 25)
# # # #         self.new_plot_button.clicked.connect(self.open_new_plot)

# # # #         top_bar.addWidget(self.combo)
# # # #         top_bar.addStretch()
# # # #         top_bar.addWidget(self.new_plot_button)

# # # #         self.plot = pg.PlotWidget()
# # # #         self.plot.setYRange(-1, 1)
# # # #         self.curve = self.plot.plot(pen='y')
# # # #         self.x = []
# # # #         self.y = []

# # # #         layout.addLayout(top_bar)
# # # #         layout.addWidget(self.plot)
# # # #         self.setLayout(layout)

# # # #         self.timer = QTimer()
# # # #         self.timer.timeout.connect(self.update_plot)
# # # #         self.timer.start(100)

# # # #     def update_plot(self):
# # # #         t = time.time()
# # # #         self.x.append(t)
# # # #         self.y.append(np.sin(t))
# # # #         self.curve.setData(self.x[-100:], self.y[-100:])

# # # #     def open_new_plot(self):
# # # #         new = PlotWidget()
# # # #         new.show()


# # # # class CommandLine(QWidget):
# # # #     def __init__(self):
# # # #         super().__init__()
# # # #         self.init_ui()

# # # #     def init_ui(self):
# # # #         layout = QVBoxLayout()
# # # #         self.history = QTextEdit()
# # # #         self.history.setReadOnly(True)
# # # #         self.prompt = QLineEdit()
# # # #         self.prompt.returnPressed.connect(self.handle_command)
# # # #         layout.addWidget(self.history)
# # # #         layout.addWidget(self.prompt)
# # # #         self.setLayout(layout)

# # # #     def handle_command(self):
# # # #         text = self.prompt.text()
# # # #         self.history.append(f"> {text}")
# # # #         self.prompt.clear()


# # # # class ControlPanel(QWidget):
# # # #     def __init__(self):
# # # #         super().__init__()
# # # #         self.init_ui()

# # # #     def init_ui(self):
# # # #         layout = QGridLayout()
# # # #         buttons = {
# # # #             "Close LO": self.close_lo,
# # # #             "Close HO": self.close_ho,
# # # #             "Open LO": self.open_lo,
# # # #             "Open HO": self.open_ho
# # # #         }
# # # #         positions = [(0, 0), (1, 0), (0, 1), (1, 1)]
# # # #         for (label, func), pos in zip(buttons.items(), positions):
# # # #             btn = QPushButton(label)
# # # #             btn.clicked.connect(func)
# # # #             layout.addWidget(btn, *pos)
# # # #         self.setLayout(layout)

# # # #     def close_lo(self): print("Close LO clicked")
# # # #     def close_ho(self): print("Close HO clicked")
# # # #     def open_lo(self): print("Open LO clicked")
# # # #     def open_ho(self): print("Open HO clicked")


# # # # class MainWindow(QWidget):
# # # #     def __init__(self):
# # # #         super().__init__()
# # # #         self.setWindowTitle("Real-Time Controller GUI")
# # # #         self.init_ui()

# # # #     def init_ui(self):
# # # #         main_layout = QHBoxLayout()

# # # #         # Left side: control + CLI
# # # #         left_layout = QVBoxLayout()
# # # #         self.ctrl = ControlPanel()
# # # #         self.cli = CommandLine()
# # # #         left_layout.addWidget(self.ctrl)
# # # #         left_layout.addWidget(self.cli)

# # # #         # Right side: plot
# # # #         self.plot = PlotWidget()

# # # #         main_layout.addLayout(left_layout, 1)
# # # #         main_layout.addWidget(self.plot, 2)

# # # #         self.setLayout(main_layout)


# # # # if __name__ == "__main__":
# # # #     app = QApplication(sys.argv)
# # # #     win = MainWindow()
# # # #     win.resize(1000, 600)
# # # #     win.show()
# # # #     sys.exit(app.exec_())
