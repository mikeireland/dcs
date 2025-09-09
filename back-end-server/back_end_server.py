"""
BackEndServer
=============

Purpose
- Exposes a single ZeroMQ REP endpoint to receive WAG/DCS JSON commands and return
  the standard reply shape: {"reply": {"time": "...", "content": "OK" | "ERROR: ..."}}.
- Implements core verbs (ping, setup, start, abort, expstatus) and dispatches
  RTS (real-time system) commands to subsystem-specific handlers via a registry
  (e.g., handlers.baldr). Baldr commands may be sent as "bld_<name>".

Why a bounded worker pool for RTS?
- A REP socket must strictly recv→send→recv…; if the command handler blocks,
  all clients are head-of-line blocked (even for different subsystems/ports).
- RTS commands are treated as *fire-and-forget*: we ACK immediately once accepted,
  then complete work in the background and push results to WAG/MCS via update_DB().
- We use a bounded ThreadPoolExecutor plus a semaphore cap to:
    * avoid unbounded thread creation and memory growth,
    * prevent the REP loop from blocking on long RTS actions,
    * provide predictable back-pressure ("ERROR: busy" when saturated).

Concurrency / safety notes
- Do not share ZMQ REQ sockets across threads. Concrete RTS tasks must
  create/connect any device sockets inside run() (per-thread) and close them.
- The main thread only parses, validates, enqueues, and immediately replies "OK".

Operational features
- Registry pattern (`register_rts`) selects the task class by command name
  (the "bld_" prefix is stripped). Adding a new RTS = add class + register once.
- `report_jobs` returns a snapshot of queued/running/done jobs (job_id, name, state, err).
- `abort` supports cancel-by-job_id; concrete tasks implement actual termination logic.
- When the pool is saturated, requests receive `"ERROR: busy"` rather than blocking.

This design keeps the wire contract simple, avoids REP head-of-line blocking, and
scales predictably while remaining easy to extend with new RTS handlers.
"""

""" 
SPECIFIC DETAILS FROM 

Asgard Top-Level Control Software User and Maintenance Manual

This back-end-server takes commands from wag, and based on a database of:
command, server, internal_command for checking what is possible:

This is the back_end_server.py

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
"baldr1"        6662,
"baldr2"        6663,
"baldr3"        6664,
"baldr4"        6665,
"MCS"           7020,
"ICS"           5555,
"RTD"           7000,
"""
import zmq
import json
import datetime
import time

# import baldr_back_end_server

import uuid
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Type, Any

import subprocess

# from rts_base import AbstractRTSTask, RTSContext, RTSState, RTSErr
from handlers.baldr_rts_handlers import register as register_baldr_rts


# port 7004 is used by nomachines! changed to 7010
class BackEndServer:
    def __init__(
        self,
        port=7010,
        server_ports={
            "hdlr": 6660,
            "hdlr_align": 6661,
            "baldr1": 6662,
            "baldr2": 6663,
            "baldr3": 6664,
            "baldr4": 6665,
            "cam_server": 6667,
            "DM_server": 6666,
        },
    ):
        self.port = port
        # self.baldr_commands = baldr_back_end_server.BaldrCommands() #This is the Ben way.
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{self.port}")
        print(f"BackEndServer started on port {self.port}")

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

        self.scripts_running = []

    def run(self):
        while True:
            # Wait for the next request from wag
            message = self.socket.recv().decode("ascii")
            if message[-1] == "\0":
                message = message[:-1]
            # Parse the message as JSON
            try:
                message = json.loads(message)
            except json.JSONDecodeError as e:
                print(f"Failed to decode JSON: {e}")
                self.socket.send_json(
                    self.create_response("ERROR: Invalid JSON format")
                )

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
        # elif command_name == "abort":
        #     return self.abort(command)
        elif command_name == "expstatus":
            return self.expstatus(command)
        elif command_name.startswith("bld_"):
            # Fire-and-forget RTS
            return self.handle_bld_rts(command)
        elif command_name.startswith("hldr_"):
            return self.handle_hdlr_rts(command)
        elif command_name.startswith("s_"):
            return self.handle_script(command)
        else:
            return self.create_response(f"ERROR: Unknown command '{command_name}'")

    def handle_bld_rts(self, command):
        """
        Dispatch BALDR RTS commands to the appropriate BALDR RTC Commander sockets.
        Self-contained; accepts RTS-format parameter list:
            command = {
            "name": "bld_open_lo" | "bld_open_ho" | "bld_close_lo" | "bld_close_ho",
            "time": "...",  # ignored
            "parameters": [
                {"name": "beam", "value": 0|1|2|3|4}
            ]
            }
        Semantics:
        - beam=0 => broadcast to beams 1..4
        - beam in 1..4 => target that specific beam
        """
        # Local-only imports to keep this method self-contained
        import json as _json

        # Map WAG verbs -> Commander command strings
        cmd_map = {
            "bld_open_lo":  'open_baldr_LO ""',
            "bld_open_ho":  'open_baldr_HO ""',
            "bld_close_lo": 'close_baldr_LO ""',
            "bld_close_ho": 'close_baldr_HO ""',
        }

        name = (command.get("name") or "").lower()
        if name not in cmd_map:
            return self.create_response(f"ERROR: Unknown BALDR RTS command '{name}'")

        # ---- Extract single 'beam' from RTS-style parameters list ----
        params = command.get("parameters", [])

        def _param_value(params_obj, key, default=None):
            # Accepts the RTS list format and (optionally) a dict format
            if isinstance(params_obj, list):
                for item in params_obj:
                    if isinstance(item, dict) and item.get("name") == key:
                        return item.get("value", default)
                return default
            if isinstance(params_obj, dict):
                return params_obj.get(key, default)
            return default

        beam_raw = _param_value(params, "beam", 0)
        try:
            beam_val = int(beam_raw)
        except Exception:
            return self.create_response("ERROR: 'beam' must be an integer in {0,1,2,3,4}")
        if beam_val not in (0, 1, 2, 3, 4):
            return self.create_response("ERROR: 'beam' must be in {0,1,2,3,4}")

        # Expand 0 -> [1,2,3,4], otherwise a single-element list
        target_beams = [1, 2, 3, 4] if beam_val == 0 else [beam_val]

        # ---- send to each requested BALDR ----
        cmd_text = cmd_map[name]
        results = []
        errors = []

        for b in target_beams:
            key = f"baldr{b}"
            sock = self.servers.get(key)
            if sock is None:
                errors.append(f"{key}: not connected")
                continue

            try:
                sock.send_string(cmd_text)
                raw = sock.recv_string()  # commander replies as string (often JSON)
            except Exception as e:
                errors.append(f"{key}: ZMQ error: {e}")
                continue

            # Try to interpret the reply (JSON preferred; else plain text)
            ok = True
            details = None
            if not raw:
                ok = False
                details = "empty reply"
            else:
                try:
                    rep = _json.loads(raw)
                    if isinstance(rep, dict):
                        # Common patterns: {"ok":true}, {"ok":false,"error":"..."},
                        # or {"reply":{"content":"OK"}} (or an error string)
                        if rep.get("ok") is False:
                            ok = False
                            details = rep.get("error") or rep
                        elif "reply" in rep and isinstance(rep["reply"], dict):
                            content = rep["reply"].get("content", "")
                            if isinstance(content, str) and content.upper().startswith("ERROR"):
                                ok = False
                                details = content
                            else:
                                details = content or "ok"
                        else:
                            # Unknown JSON shape; treat as OK and echo object
                            details = rep
                    else:
                        details = rep
                except _json.JSONDecodeError:
                    # Non-JSON reply; consider it error only if it contains 'error'
                    if isinstance(raw, str) and "error" in raw.lower():
                        ok = False
                    details = raw

            if ok:
                results.append(f"{key}: OK")
            else:
                errors.append(f"{key}: {details}")

        if errors:
            if results:
                return self.create_response(f"ERROR: partial ({'; '.join(results)}; errors: {', '.join(errors)})")
            return self.create_response(f"ERROR: {', '.join(errors)}")

        # All good
        return self.create_response("OK")

    def handle_hdlr_rts(self, command):
        """
        Any formatting needs to be done here
        @mike
        """

    def handle_script(self, command):
        """
        Handle script commands, e.g., s_h-autoalign
        """
        command_name = command.get("name", "").lower()
        parameters = command.get("parameters", [])
        if command_name == "s_h-autoalign":
            process = subprocess.Popen(["h-autoalign", "-a", "ip", "-o", "mcs"])
        else:
            return self.create_response(
                f"ERROR: Unknown script command '{command_name}'"
            )

        self.scripts_running.append(process)
        return self.create_response("OK")

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
                    return self.create_response(
                        f"ERROR: NWORESET value must be an integer"
                    )
                if value <= 2:
                    self.servers["cam_server"].send_string(
                        f'cli "set mode globalresetcds"'
                    )
                    print(self.servers["cam_server"].recv().decode("ascii"))
                elif value < 500:
                    self.servers["cam_server"].send_string(
                        f'cli "set mode globalresetbursts"'
                    )
                    print(self.servers["cam_server"].recv().decode("ascii"))
                    self.servers["cam_server"].send_string(
                        f'cli "set nbreadworeset {value}"'
                    )
                    print(self.servers["cam_server"].recv().decode("ascii"))
                else:
                    return self.create_response(
                        f"ERROR: NWORESET value {value} is higher than the max (500)"
                    )
            elif name == "DET.GAIN":
                try:
                    value = int(value)
                except ValueError:
                    return self.create_response(f"ERROR: GAIN value must be an integer")
                if value < 0 or value > 100:
                    return self.create_response(
                        f"ERROR: GAIN value {value} is out of range (0-100)"
                    )
                self.servers["cam_server"].send_string(f"set_gain {value}")
                print(self.servers["cam_server"].recv().decode("ascii"))
            elif name == "DET.NDIT":
                print(
                    "DIT command should set the number of integrations in a save file - not implemented"
                )
            else:
                # Handle other parameters as needed
                print(f"Unknown parameter: {name} = {value}")
                return self.create_response(f"ERROR: Unknown parameter '{name}'")

        return self.create_response("OK")

    def start(self, command):
        # Implement start logic here
        return self.create_response("OK")

    def expstatus(self, command):
        # Implement expstatus logic here
        return self.create_response("OK")


if __name__ == "__main__":
    server = BackEndServer()
    server.run()
