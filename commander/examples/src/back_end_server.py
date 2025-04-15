"""
This back-end-server takes commands from wag, and based on a database of:
command, server, internal_command for checking what is possible:

This is the MultiDeviceServer.py

The server has to implement a nubmber of commands, and after each successful command, it should return:
{
“reply”:
{
“time” : “< timestamp >”
“content” : “OK”
,
}
}

Commands to be implemented:

PING: recieve:
{
“command” :
{
“name” : “ping”
,
“time” : “< timestamp >”
}
}


SETUP: recieve:
{
“command” :
{
“name” : “setup”
,
“time” : “< timestamp >”
“parameters” :
,
[
{
“name” : “< keyword #1 name >”
“value” : < keyword #1 value>
},
. . .
{
“name” : “< keyword #n name >”
“value” : < keyword #n value>
}
,
,
]
}

NB the setup command will be used in the asg_hdlr_only_autotest_align template, e.g. for now
we are using DCS readout, 1000Hz, and gain=10. DET.DIT=0.001, DET.NDIT=0 (i.e. no saving),
DET.NWORESET=2.

We need START (name:start) and ABORT (name:abort) commands, which are used to start and abort the readout.

Once we get to on-sky data, we need an EXPSTATUS command.
{
“command” :
{
“name” : “expstatus”
,
“time” : “< timestamp >”
}
}

The RTS command is used for RTS commands, which are lower case and generally passed directoy to servers.
{
command:
{
“name” : “< name of the command >”
,
“time” : “< timestamp >”
,
“parameters” :
[
{
“name” : “< name of parameter 1 >”
“value” : < value of parameter 1 >
,
},
. . .
{
“name” : “< name of parameter n >”
“value” : < value of parameter n >
,
}
]
}
}
e.g. "name" : "beamid", "value" : 1,2,3,4 or 0 (for all available)
"name: "server" : "heimdallr", "baldr", "cam"

If an error, then replace "OK" in the reply with “ERROR: < error description>”
"""
import zmq
import json
import time

class BackEndServer:
    def __init__(self, port=7001, \
            server_ports = {"heimdallr": 7002, "baldr": 7003, "cam": 7004}):
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{self.port}")
        print(f"BackEndServer started on port {self.port}")

    def run(self):
        while True:
            # Wait for the next request from wag
            message = self.socket.recv_json()
            print(f"Received request: {message}")

            # Process the command
            response = self.process_command(message)

            # Send the reply back to wag
            self.socket.send_json(response)

    def process_command(self, message):
        command = message.get("command", {})
        command_name = command.get("name", "").lower()

        if command_name == "ping":
            return self.create_response("OK")
        elif command_name == "setup":
            return self.setup(command)
        elif command_name == "start":
            return self.start(command)
        elif command_name == "abort":
            return self.abort(command)
        elif command_name == "expstatus":
            return self.expstatus(command)
        elif command_name == "rts":
            return self.rts(command)
        else:
            return self.create_response(f"ERROR: Unknown command '{command_name}'")

    def create_response(self, content):
        return {
            "reply": {
                "time": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()),
                "content": content,
            }
        }

    def setup(self, command):
        # Implement setup logic here
        parameters = command.get("parameters", [])
        # Validate and process parameters as needed
        return self.create_response("OK")

    def start(self, command):
        # Implement start logic here
        return self.create_response("OK")

    def abort(self, command):
        # Implement abort logic here
        return self.create_response("OK")

    def expstatus(self, command):
        # Implement expstatus logic here
        return self.create_response("OK")

    def rts(self, command):
        # Implement RTS logic here
        parameters = command.get("parameters", [])
        # Not implemented
        print("RTS command received: ", parameters)
        # Validate and process parameters as needed
        return self.create_response("OK")