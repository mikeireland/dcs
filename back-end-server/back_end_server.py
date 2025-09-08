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

from rts_base import AbstractRTSTask, RTSContext, RTSState, RTSErr
from handlers.baldr_rts_handlers import register as register_baldr_rts




# port 7004 is used by nomachines! changed to 7010
class BackEndServer:
    def __init__(self, port=7010, \
            server_ports = {"hdlr": 6660, "hdlr_align": 6661, "baldr1": 6662,"baldr2":6663,"baldr3":6664,"baldr4":6665, "cam_server": 6667, "DM_server": 6666}):
        self.port = port
        #self.baldr_commands = baldr_back_end_server.BaldrCommands() #This is the Ben way.
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

         # ---- RTS registry and execution bounds ----
        self.rts_registry: Dict[str, Type[AbstractRTSTask]] = {}
        self.rts_ctx = RTSContext(mcs_notify=None)  # plug a MCS notifier when ready


        """
        We use a bounded ThreadPoolExecutor plus a semaphore cap to:
            avoid unbounded thread creation and memory growth,
            prevent the REP loop from blocking on long RTS actions,
            provide predictable back-pressure ("ERROR: busy" when saturated).

            this was implemented to allow python scripts to be run directly from the back_end_server
            perhaps I should have another bld_alignment server the back_end_server talks to to run
            these scripts?

            currently this will only say busy if there are more than N running or queued things
        """
        # Bounded thread-pool for RTS tasks 
        self.max_workers = 16          # tune per host
        self.max_inflight = 64         # overall in-flight bound (queued + running)
        self._inflight_sem = threading.Semaphore(self.max_inflight)
        self.executor = ThreadPoolExecutor(max_workers=self.max_workers, thread_name_prefix="rts")

        # Track jobs if you later want abort/status (optional)
        self.jobs: Dict[str, tuple[AbstractRTSTask, object]] = {}

        #Register handlers AFTER the registry exists
        register_baldr_rts(self)


    def register_rts(self, name: str, cls: Type[AbstractRTSTask]) -> None:
        """Register an RTS command class by its short name (without 'bld_' prefix)."""
        self.rts_registry[name.lower()] = cls

    def _submit_bounded(self, fn, *args, **kwargs):# only for rts commands 
        """Try to submit work if under the inflight cap; else return None (busy)."""
        if not self._inflight_sem.acquire(blocking=False):
            return None
        fut = self.executor.submit(fn, *args, **kwargs)
        fut.add_done_callback(lambda f: self._inflight_sem.release())
        return fut

    def _run_task(self, task: AbstractRTSTask): # only for rts commands 
        """Worker body: run + update DB, set final state on success/failure."""
        try:
            task.state = int(RTSState.RUNNING)
            task.run(task.command.get("parameters", []))
            task.update_DB()
            if task.state != int(RTSState.ABORTED) and task.err == int(RTSErr.OK):
                task.state = int(RTSState.DONE)
        except Exception as e:
            task.state = int(RTSState.FAILED)
            task.err = int(RTSErr.RUNTIME)
            task.metadata["error"] = str(e)

    def prune_jobs(self, keep_last: int = 500) -> int:
        # for rts commands we keep log of jobs. for now we just set a hard limit on this length
        # later we should classify them as active or not and only keep a working class record of 
        # active jobs, and we can just pipe all jobs to some logfile that is not held internally 
        # in this class. TO DO later (if needed)
        n = len(self.jobs)
        if n <= keep_last:
            return 0
        to_remove = n - keep_last
        removed = 0
        # dict preserves insertion order (oldest first)
        for jid in list(self.jobs.keys())[:to_remove]:
            self.jobs.pop(jid, None)
            removed += 1
        return removed

    def handle_rts(self, command: Dict[str, Any]) -> Dict[str, Any]:
        raw = (command.get("name") or "").lower()
        short = raw[4:] if raw.startswith("bld_") else raw
        TaskClass = self.rts_registry.get(short)
        if not TaskClass:
            return self.create_response(f"ERROR: Unknown RTS command '{raw}'")
        task = TaskClass(command, self.rts_ctx)
        if not task.ok_to_run():
            return self.create_response(f"ERROR: {task.metadata.get('error', 'invalid')}")
        job_id = uuid.uuid4().hex[:8]
        task.metadata["job_id"] = job_id
        fut = self._submit_bounded(self._run_task, task)
        if fut is None:
            return self.create_response("ERROR: busy")
        self.jobs[job_id] = (task, fut)  #### 

        ### we need better logging - maybe pipe to logFile and only keep activate jobs running locally
        # for now we just prune forcefully! 
        self.prune_jobs(keep_last = 500) 

        return self.create_response("OK")

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
        elif command_name == 'report_jobs':
            return self.report_jobs(command)
        elif command_name.startswith("bld_") or (command_name in self.rts_registry):
            # Fire-and-forget RTS
            return self.handle_rts(command)
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

    # def abort(self, command):
    #     # Implement abort logic here
    #     return self.create_response("OK")

    def abort_job(self, job_id: str) -> Dict[str, any]:
        item = self.jobs.get(job_id)
        if not item:
            return self.create_response("ERROR: unknown job_id")
        task, fut = item
        try:
            task.abort()
            return self.create_response("OK")
        except Exception as e:
            return self.create_response(f"ERROR: {e}")
        
    def abort(self, command):
        params = command.get("parameters", []) or []
        job_id = None
        for p in params:
            if (p.get("name") or "").lower() == "job_id":
                job_id = str(p.get("value"))
                break
        if not job_id:
            return self.create_response("ERROR: missing job_id")
        return self.abort_job(job_id)
    

    # despite giving instant feedback jobs may be still running in background (.e.g calibration IM). 
    # Here we can check their status
    def report_jobs(self, command):
        # 
        params = command.get("parameters", []) or []
        filter_job = None
        for p in params:
            if (p.get("name") or "").lower() == "job_id":
                filter_job = str(p.get("value"))
                break

        # take a snapshot to avoid racing with worker updates
        snapshot = list(self.jobs.items())

        jobs = []
        for job_id, (task, fut) in snapshot:
            if filter_job and job_id != filter_job:
                continue
            if fut.running():
                status = "running"
            elif fut.done():
                status = "done"
            else:
                status = "queued"

            jobs.append({
                "job_id": job_id,
                "name": (task.command.get("name") or "").lower(),
                "state": int(task.state),         # RTSState as int
                "err": int(task.err),             # RTSErr as int
                "status": status,                 # human-friendly
                "created_utc": task.metadata.get("created_utc"),
                "meta": {
                    "command_time": task.metadata.get("command_time"),
                },
            })

        # build the full reply here (do not call create_response)
        current_utc_time = datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S")
        return {
            "reply": {
                "time": current_utc_time,
                "content": "OK",
                "jobs": jobs,          # <- attached here
            }
        }

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
