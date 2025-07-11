import sys
import zmq
import json
from PyQt5 import QtWidgets, QtGui, QtCore

# Load server list from sockets file
def load_servers(filename):
    servers = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            # Expecting format: name port
            parts = line.split()
            if len(parts) == 2:
                name, port = parts
                servers.append((name, int(port)))
    return servers

class ServerTab(QtWidgets.QWidget):
    def __init__(self, server_name, zmq_socket, parent=None):
        super().__init__(parent)
        self.zmq_socket = zmq_socket

        layout = QtWidgets.QVBoxLayout(self)
        self.text_area = QtWidgets.QTextEdit(self)
        self.text_area.setReadOnly(True)
        self.input_line = QtWidgets.QLineEdit(self)
        self.send_button = QtWidgets.QPushButton("Send", self)
        self.command_dropdown = QtWidgets.QComboBox(self)
        self.command_dropdown.setEditable(False)
        self.command_dropdown.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)

        hlayout = QtWidgets.QHBoxLayout()
        hlayout.addWidget(self.command_dropdown)
        hlayout.addWidget(self.input_line)
        hlayout.addWidget(self.send_button)

        layout.addWidget(self.text_area)
        layout.addLayout(hlayout)

        self.send_button.clicked.connect(self.send_command)
        self.input_line.returnPressed.connect(self.send_command)
        self.command_dropdown.activated[str].connect(self.set_input_line)

    def set_input_line(self, cmd):
        self.input_line.setText(cmd)

    def send_command(self):
        cmd = self.input_line.text().strip()
        if not cmd:
            return
        self.text_area.append(f"> {cmd}")
        try:
            self.zmq_socket.send_string(cmd)
            reply = self.zmq_socket.recv_string()
        except zmq.error.Again:
            reply = "[Error] ZMQ request timed out."
        except Exception as e:
            reply = f"[Error] {e}"
        # Try to parse as JSON and pretty-print, else just show with \n expanded
        try:
            resp = json.loads(reply)
            if isinstance(resp, dict):
                pretty = json.dumps(resp, indent=4)
                self.text_area.append(pretty)
            else:
                self.text_area.append(str(resp).replace("\\n", "\n"))
        except json.JSONDecodeError:
            self.text_area.append(reply.replace("\\n", "\n"))
        self.input_line.clear()

    def populate_commands(self):
        # Send "command_names" and populate the dropdown
        self.command_dropdown.clear()
        try:
            self.zmq_socket.send_string("command_names")
            reply = self.zmq_socket.recv_string()
            try:
                commands = json.loads(reply)
                if isinstance(commands, list):
                    self.command_dropdown.addItems(commands)
            except Exception:
                # Fallback: try to eval a python list string
                try:
                    commands = eval(reply)
                    if isinstance(commands, list):
                        self.command_dropdown.addItems([str(c) for c in commands])
                except Exception:
                    pass
        except Exception:
            pass

class UniversalClient(QtWidgets.QMainWindow):
    def __init__(self, ip_addr, servers, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Asgard DCS text interface: {ip_addr}")
        self.resize(700, 500)
        self.tabs = QtWidgets.QTabWidget(self)
        self.setCentralWidget(self.tabs)
        self.context = zmq.Context()
        self.sockets = []
        self.tab_widgets = []

        for name, port in servers:
            socket = self.context.socket(zmq.REQ)
            socket.connect(f"tcp://{ip_addr}:{port}")
            # Set ZMQ send/recv timeouts (milliseconds)
            socket.setsockopt(zmq.SNDTIMEO, 2000)
            socket.setsockopt(zmq.RCVTIMEO, 2000)
            self.sockets.append(socket)
            tab = ServerTab(name, socket)
            self.tabs.addTab(tab, name)
            self.tab_widgets.append(tab)

        self.tabs.currentChanged.connect(self.on_tab_changed)
        # Populate commands for the first tab
        if self.tab_widgets:
            self.tab_widgets[0].populate_commands()

    def on_tab_changed(self, idx):
        if 0 <= idx < len(self.tab_widgets):
            self.tab_widgets[idx].populate_commands()

def main():
    if len(sys.argv) != 2:
        print("Usage: python universal_client.py <ip_address>")
        sys.exit(1)
    ip_addr = sys.argv[1]
    servers = load_servers('sockets')
    app = QtWidgets.QApplication(sys.argv)
    client = UniversalClient(ip_addr, servers)
    client.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()