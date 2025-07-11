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

        hlayout = QtWidgets.QHBoxLayout()
        hlayout.addWidget(self.input_line)
        hlayout.addWidget(self.send_button)

        layout.addWidget(self.text_area)
        layout.addLayout(hlayout)

        self.send_button.clicked.connect(self.send_command)
        self.input_line.returnPressed.connect(self.send_command)

    def send_command(self):
        cmd = self.input_line.text().strip()
        if not cmd:
            return
        self.text_area.append(f"> {cmd}")
        try:
            self.zmq_socket.send_string(cmd)
            reply = self.zmq_socket.recv_string()
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

class UniversalClient(QtWidgets.QMainWindow):
    def __init__(self, ip_addr, servers, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Universal ZMQ Client")
        self.resize(700, 500)
        self.tabs = QtWidgets.QTabWidget(self)
        self.setCentralWidget(self.tabs)
        self.context = zmq.Context()
        self.sockets = []

        for name, port in servers:
            socket = self.context.socket(zmq.REQ)
            socket.connect(f"tcp://{ip_addr}:{port}")
            self.sockets.append(socket)
            tab = ServerTab(name, socket)
            self.tabs.addTab(tab, name)

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