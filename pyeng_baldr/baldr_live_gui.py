import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout,
    QComboBox, QTextEdit, QLineEdit, QGridLayout
)
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import numpy as np
import time


class PlotWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        # Top bar with dropdown and "+" button
        top_bar = QHBoxLayout()
        self.combo = QComboBox()
        self.combo.addItems(["Signal 1", "Signal 2", "Signal 3"])  # Stub
        self.new_plot_button = QPushButton("+")
        self.new_plot_button.setFixedSize(25, 25)
        self.new_plot_button.clicked.connect(self.open_new_plot)

        top_bar.addWidget(self.combo)
        top_bar.addStretch()
        top_bar.addWidget(self.new_plot_button)

        self.plot = pg.PlotWidget()
        self.plot.setYRange(-1, 1)
        self.curve = self.plot.plot(pen='y')
        self.x = []
        self.y = []

        layout.addLayout(top_bar)
        layout.addWidget(self.plot)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def update_plot(self):
        t = time.time()
        self.x.append(t)
        self.y.append(np.sin(t))
        self.curve.setData(self.x[-100:], self.y[-100:])

    def open_new_plot(self):
        new = PlotWidget()
        new.show()


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

    def handle_command(self):
        text = self.prompt.text()
        self.history.append(f"> {text}")
        self.prompt.clear()


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
        self.init_ui()

    def init_ui(self):
        main_layout = QHBoxLayout()

        # Left side: control + CLI
        left_layout = QVBoxLayout()
        self.ctrl = ControlPanel()
        self.cli = CommandLine()
        left_layout.addWidget(self.ctrl)
        left_layout.addWidget(self.cli)

        # Right side: plot
        self.plot = PlotWidget()

        main_layout.addLayout(left_layout, 1)
        main_layout.addWidget(self.plot, 2)

        self.setLayout(main_layout)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.resize(1000, 600)
    win.show()
    sys.exit(app.exec_())