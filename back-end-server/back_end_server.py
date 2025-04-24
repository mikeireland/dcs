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

Sockets:
cam_server      6667
DM_server       6666
hdlr            6660
hdlr_align      6661
baldr           6662
"""
import zmq
import json
import datetime
import time

class BackEndServer:
    def __init__(self, port=7004, \
            server_ports = {"hdlr": 6660, "hdlr_align": 6661, "baldr": 6662, "cam_server": 6667, "DM_server": 6666}):
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{self.port}")
        print(f"BackEndServer started on port {self.port}")
        # Initialize server ports and connections to other servers
        # A connection to "localhost" should work, but we need to test
        # that this is OK when servers binding to *.
        self.server_ports = server_ports
        self.servers = {}
        for server_name, port in self.server_ports.items():
            server = self.context.socket(zmq.REQ)
            try:
                server.connect(f"tcp://mimir:{port}")
                self.servers[server_name] = server
                print(f"Connected to {server_name} on port {port}")
            except zmq.ZMQError as e:
                print(f"Failed to connect to {server_name} on port {port}: {e}")
                self.servers[server_name] = None
                # Handle connection error (e.g., retry, log, etc.)
                continue
        print("All server connections initialized.")

    def run(self):
        while True:
            # Wait for the next request from wag
            message = self.socket.recv().decode("ascii")
            if (message[-1]=='\0'): 
                message = message[:-1]
            # Parse the message as JSON
            try:
                message = json.loads(message)
            except json.JSONDecodeError as e:
                print(f"Failed to decode JSON: {e}")
                self.socket.send_json(self.create_response("ERROR: Invalid JSON format"))
                import pdb; pdb.set_trace()
            
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
        elif command_name == "heimdallr_bb_align":
            return self.heimdallr_bb_align(command)
        elif command_name == "fringe_search":
            return self.fringe_search(command)
        else:
            return self.create_response(f"ERROR: Unknown command '{command_name}'")

    def create_response(self, content):
        # Get the current UTC time
        current_utc_time = datetime.datetime.utcnow()
        # Format the UTC time
        return {
            "reply": {
                "time": current_utc_time.strftime("%Y-%m-%dT%H:%M:%S"),
                "content": content,
            }
        }

    def setup(self, command):
        # Implement setup logic here
        parameters = command.get("parameters", [])
        # Validate and process parameters as needed
        for param in parameters:
            name = param.get("name")
            value = param.get("value")
            print(f"Setup parameter: {name} = {value}")
            # Add logic to handle each parameter as needed
            # DIT, NDIT, NWORESET, etc.
            if name == "DET.DIT":
                # Handle DIT parameter
                fps = 1 / value
                self.servers["cam_server"].send_string(f"set_fps {fps:.1f}")
                # !!! Could the next line be recv_string? 
                print(self.servers["cam_server"].recv().decode("ascii"))
            elif name == "DET.NWORESET":
                try:
                    value = int(value)
                except ValueError:
                    return self.create_response(f"ERROR: NWORESET value must be an integer")
                if value <= 2:
                    self.servers["cam_server"].send_string(f'cli "set mode globalresetcds"')
                    print(self.servers["cam_server"].recv().decode("ascii"))
                elif value < 500:
                    self.servers["cam_server"].send_string(f'cli "set mode globalresetbursts"')
                    print(self.servers["cam_server"].recv().decode("ascii"))
                    self.servers["cam_server"].send_string(f'cli "set nbreadworeset {value}"')
                    print(self.servers["cam_server"].recv().decode("ascii"))
                else:
                    return self.create_response(f"ERROR: NWORESET value {value} is higher than the max (500)")
            elif name == "DET.GAIN":
                try:
                    value = int(value)
                except ValueError:
                    return self.create_response(f"ERROR: GAIN value must be an integer")
                if value < 0 or value > 100:
                    return self.create_response(f"ERROR: GAIN value {value} is out of range (0-100)")
                self.servers["cam_server"].send_string(f'set_gain {value}')
                print(self.servers["cam_server"].recv().decode("ascii"))
            elif name == "DET.NDIT":
                print("DIT command should set the number of integrations in a save file - not implemented")
            else:
                # Handle other parameters as needed
                print(f"Unknown parameter: {name} = {value}")
                return self.create_response(f"ERROR: Unknown parameter '{name}'")
                    
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

    def heimdallr_bb_align(self, command):
        # Implement RTS logic here
        parameters = command.get("parameters", [])
        # Not implemented
        print("heimdallr_bb_align command received: ", parameters)
        time.sleep(2)
        # Validate and process parameters as needed
        return self.create_response("OK")
    
    def fringe_search(self, command):
        # Implement RTS logic here
        parameters = command.get("parameters", [])
        # Not implemented
        print("fringe_search command received: ", parameters)
        time.sleep(2)
        # Validate and process parameters as needed
        return self.create_response("OK")
    
if __name__ == "__main__":
    server = BackEndServer()
    server.run()
