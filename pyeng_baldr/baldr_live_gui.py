import sys
import time
import zmq 
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QComboBox, QTextEdit, QLineEdit, QGridLayout
)
from PyQt5.QtCore import QTimer, pyqtSignal
import pyqtgraph as pg


# Create global ZMQ client socket and context once
_zmq_context = zmq.Context.instance()
_zmq_socket = _zmq_context.socket(zmq.REQ)
_zmq_socket.connect("tcp://127.0.0.1:5555")  # Match the server address/port

def poll_telemetry(signal_name, socket=None):
    """
    Poll telemetry using ZMQ REQ/REP protocol.
    If ZMQ fails, fallback to dummy values.
    """

    try:
        _zmq_socket.send_string(f"status {signal_name}")
        response = _zmq_socket.recv_string()
        return float(response)
    except Exception as e:
        print(f"[poll_telemetry] ZMQ error: {e}")
        return 0.0

    # try:
    #     _zmq_socket.send_string(signal_name)
    #     response = _zmq_socket.recv_string()
    #     return float(response)
    # except Exception as e:
    #     print(f"[poll_telemetry] ZMQ error or timeout for '{signal_name}': {e}")
    #     # Fallback to dummy values
    #     # t = time.time()
    #     # if signal_name == "Signal 1":
    #     #     return np.sin(t)
    #     # elif signal_name == "Signal 2":
    #     #     return np.cos(t)
    #     # elif signal_name == "Signal 3":
    #     #     return np.sin(t * 2) * np.exp(-0.1 * t % 10)
    #     # else:
    #     #     return 0.0
        
class PlotWidget(QWidget):
    new_plot_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Top bar with dropdown and "+" button
        top_bar = QHBoxLayout()
        self.combo = QComboBox()
        self.combo.addItems(["Signal 1", "Signal 2", "Signal 3"])  # Stubbed signals
        self.new_plot_button = QPushButton("+")
        self.new_plot_button.setFixedSize(25, 25)
        self.new_plot_button.clicked.connect(self.new_plot_requested.emit)

        top_bar.addWidget(self.combo)
        top_bar.addStretch()
        top_bar.addWidget(self.new_plot_button)

        # Plotting area
        self.plot = pg.PlotWidget()
        self.curve = self.plot.plot(pen='y')
        self.x = []
        self.y = []

        layout.addLayout(top_bar)
        layout.addWidget(self.plot)
        self.setLayout(layout)

        # Timer for updating the plot
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    # def update_plot(self):
    #     t = time.time()
    #     self.x.append(t)
    #     self.y.append(np.sin(t))  # Stub data
    #     self.curve.setData(self.x[-100:], self.y[-100:])

    def update_plot(self):
        t = time.time()
        self.x.append(t)

        current_signal = self.combo.currentText()
        y_val = poll_telemetry(current_signal, socket = _zmq_socket)

        self.y.append(y_val)
        self.curve.setData(self.x[-100:], self.y[-100:])

class CommandLine(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        self.history = QTextEdit()
        self.history.setReadOnly(True)
        self.prompt = QLineEdit()
        self.prompt.returnPressed.connect(self.handle_command)
        layout.addWidget(self.history)
        layout.addWidget(self.prompt)
        self.setLayout(layout)

    # # def handle_command(self):
    # #     text = self.prompt.text()
    # #     self.history.append(f"> {text}")
    # #     self.prompt.clear()
    # def handle_command(self):
    #     text = self.prompt.text()
    #     self.history.append(f"> {text}")
    #     self.prompt.clear()

    #     try:
    #         _zmq_command_socket.send_string(text)
    #         response = _zmq_command_socket.recv_string()
    #         self.history.append(response)
    #     except Exception as e:
    #         self.history.append(f"[ZMQ error] {e}")


    def handle_command(self):
        text = self.prompt.text()
        self.history.append(f"> {text}")
        self.prompt.clear()

        try:
            _zmq_socket.send_string(f"{text}")
            response = _zmq_socket.recv_string()
            self.history.append(response)
        except Exception as e:
            self.history.append(f"[ZMQ error] {e}")

class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QGridLayout()
        buttons = {
            "Close LO": self.close_lo,
            "Close HO": self.close_ho,
            "Open LO": self.open_lo,
            "Open HO": self.open_ho
        }
        positions = [(0, 0), (1, 0), (0, 1), (1, 1)]
        for (label, func), pos in zip(buttons.items(), positions):
            btn = QPushButton(label)
            btn.clicked.connect(func)
            layout.addWidget(btn, *pos)
        self.setLayout(layout)

    def close_lo(self): print("Close LO clicked")
    def close_ho(self): print("Close HO clicked")
    def open_lo(self): print("Open LO clicked")
    def open_ho(self): print("Open HO clicked")


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time Controller GUI")
        self.plot_windows = []  # Manage references to all plot windows
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()

        # Left: Control Panel + CLI
        left_layout = QVBoxLayout()
        self.ctrl = ControlPanel()
        self.cli = CommandLine()
        left_layout.addWidget(self.ctrl)
        left_layout.addWidget(self.cli)

        # Right: Main PlotWidget
        self.plot = PlotWidget()
        self.plot.new_plot_requested.connect(self.spawn_new_plot)

        main_layout.addLayout(left_layout, 1)
        main_layout.addWidget(self.plot, 2)

        self.setLayout(main_layout)

    def spawn_new_plot(self):
        new_plot = PlotWidget()
        new_plot.setWindowTitle("New Plot Window")
        new_plot.resize(500, 300)
        new_plot.new_plot_requested.connect(self.spawn_new_plot)
        new_plot.show()
        self.plot_windows.append(new_plot)  # Prevent garbage collection


def run_app():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.resize(1000, 600)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    run_app()


# import sys
# from PyQt5.QtWidgets import (
#     QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
#     QComboBox, QTextEdit, QLineEdit, QGridLayout
# )
# from PyQt5.QtCore import QTimer
# import pyqtgraph as pg
# import numpy as np
# import time


# class PlotWidget(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.init_ui()

#     def init_ui(self):
#         layout = QVBoxLayout()

#         # Top bar with dropdown and "+" button
#         top_bar = QHBoxLayout()
#         self.combo = QComboBox()
#         self.combo.addItems(["Signal 1", "Signal 2", "Signal 3"])  # Stub
#         self.new_plot_button = QPushButton("+")
#         self.new_plot_button.setFixedSize(25, 25)
#         self.new_plot_button.clicked.connect(self.open_new_plot)

#         top_bar.addWidget(self.combo)
#         top_bar.addStretch()
#         top_bar.addWidget(self.new_plot_button)

#         self.plot = pg.PlotWidget()
#         self.plot.setYRange(-1, 1)
#         self.curve = self.plot.plot(pen='y')
#         self.x = []
#         self.y = []

#         layout.addLayout(top_bar)
#         layout.addWidget(self.plot)
#         self.setLayout(layout)

#         self.timer = QTimer()
#         self.timer.timeout.connect(self.update_plot)
#         self.timer.start(100)

#     def update_plot(self):
#         t = time.time()
#         self.x.append(t)
#         self.y.append(np.sin(t))
#         self.curve.setData(self.x[-100:], self.y[-100:])

#     def open_new_plot(self):
#         new = PlotWidget()
#         new.show()


# class CommandLine(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.init_ui()

#     def init_ui(self):
#         layout = QVBoxLayout()
#         self.history = QTextEdit()
#         self.history.setReadOnly(True)
#         self.prompt = QLineEdit()
#         self.prompt.returnPressed.connect(self.handle_command)
#         layout.addWidget(self.history)
#         layout.addWidget(self.prompt)
#         self.setLayout(layout)

#     def handle_command(self):
#         text = self.prompt.text()
#         self.history.append(f"> {text}")
#         self.prompt.clear()


# class ControlPanel(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.init_ui()

#     def init_ui(self):
#         layout = QGridLayout()
#         buttons = {
#             "Close LO": self.close_lo,
#             "Close HO": self.close_ho,
#             "Open LO": self.open_lo,
#             "Open HO": self.open_ho
#         }
#         positions = [(0, 0), (1, 0), (0, 1), (1, 1)]
#         for (label, func), pos in zip(buttons.items(), positions):
#             btn = QPushButton(label)
#             btn.clicked.connect(func)
#             layout.addWidget(btn, *pos)
#         self.setLayout(layout)

#     def close_lo(self): print("Close LO clicked")
#     def close_ho(self): print("Close HO clicked")
#     def open_lo(self): print("Open LO clicked")
#     def open_ho(self): print("Open HO clicked")


# class MainWindow(QWidget):
#     def __init__(self):
#         super().__init__()
#         self.setWindowTitle("Real-Time Controller GUI")
#         self.init_ui()

#     def init_ui(self):
#         main_layout = QHBoxLayout()

#         # Left side: control + CLI
#         left_layout = QVBoxLayout()
#         self.ctrl = ControlPanel()
#         self.cli = CommandLine()
#         left_layout.addWidget(self.ctrl)
#         left_layout.addWidget(self.cli)

#         # Right side: plot
#         self.plot = PlotWidget()

#         main_layout.addLayout(left_layout, 1)
#         main_layout.addWidget(self.plot, 2)

#         self.setLayout(main_layout)


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     win = MainWindow()
#     win.resize(1000, 600)
#     win.show()
#     sys.exit(app.exec_())