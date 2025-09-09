# baldr_wag_client.py
import json, time, socket
import logging
import os
from dataclasses import dataclass, asdict, fields
from typing import Any, Dict, List, Optional, Tuple
import zmq
from datetime import datetime, timezone

# --- Logging setup: file and console ---
def _setup_logging():
    log_dir = os.path.expanduser("~/logs/mcs/")
    os.makedirs(log_dir, exist_ok=True)
    log_name = time.strftime("mcs_%Y%m%d_%H%M%S.log")
    log_path = os.path.join(log_dir, log_name)
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    fh = logging.FileHandler(log_path)
    fh.setLevel(logging.INFO)
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s %(levelname)s: %(message)s')
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)
    logger.handlers = []
    logger.addHandler(fh)
    logger.addHandler(ch)
    logger.info(f"Logging started. Log file: {log_path}")

_setup_logging()
# baldr_wag_client.py
import json, time, socket
import logging
import os
from dataclasses import dataclass, asdict, fields
from typing import Any, Dict, List, Optional, Tuple
import zmq
from datetime import datetime, timezone

"""
Following protocol described in 
Top-Level Control Software
User and Maintenance Manual
sec 8.7.2

example (to be discussed with team) of baldr_mcs_client which
is a lightweight Python bridge that polls Baldr/Heimdallr or other
ZMQ status and reads/writes the corresponding shared parameters 
on WAG’s Module Communication Server (TCP 7020), keeping OLDB in 
sync for operations and GUIs. 

Sockets:
cam_server      6667
DM_server       6666
hdlr            6660
hdlr_align      6661
baldr           6662

"""


class ZmqReq:
    """
    An adapter for a ZMQ REQ socket (client).
    """

    def __init__(self, endpoint: str, timeout_ms: int = 1500):
        self.ctx = zmq.Context.instance()
        self.s = self.ctx.socket(zmq.REQ)
        self.s.RCVTIMEO = timeout_ms
        self.s.SNDTIMEO = timeout_ms
        self.s.connect(endpoint)

    def send_payload(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        self.s.send_string(json.dumps(payload, sort_keys=True))

        try:
            res = self.s.recv().decode("ascii")[:-1]
            return json.loads(res)
        except zmq.error.Again:
            return None


class ZmqRep:
    """
    An adapter for a ZMQ REP socket (server).
    """

    def __init__(self, endpoint: str):
        self.ctx = zmq.Context.instance()
        self.s = self.ctx.socket(zmq.REP)
        self.s.bind(endpoint)

    def recv_payload(self) -> Optional[Dict[str, Any]]:
        try:
            msg = self.s.recv_string()
            return json.loads(msg)
        except zmq.error.Again:
            return None

    def send_payload(self, payload: Dict[str, Any]) -> bool:
        try:
            self.s.send_string(json.dumps(payload))
            return True
        except zmq.error.Again:
            return False


# ---------------- MCS client ----------------
class MCSClient:
    def __init__(
        self,
        dcs_endpoints: dict,
        script_endpoint: str,
        publish_endpoint: str,
        sleep_time: float = 1.0,
    ):
        self.publish_z = ZmqReq(publish_endpoint)
        logging.info(f"REQ publish set up on {publish_endpoint}")

        self.script_z = ScriptAdapter(script_endpoint)
        logging.info(f"ScriptAdapter(REP) set up on {script_endpoint}")

        self.dcs_adapters = {}
        for dcs_name, endpoint in dcs_endpoints.items():
            self.dcs_adapters[dcs_name] = CppServerAdapter(endpoint)

        self.requester = "mimir"

        self.sleep_time = sleep_time

    def __init__(
        self,
        dcs_endpoints: dict,
        script_endpoint: str,
        publish_endpoint: str,
        sleep_time: float = 1.0,
    ):
    self._setup_logging()
    self.publish_z = ZmqReq(publish_endpoint)
    logging.info(f"REQ publish set up on {publish_endpoint}")

    self.script_z = ScriptAdapter(script_endpoint)
    logging.info(f"ScriptAdapter(REP) set up on {script_endpoint}")

        self.dcs_adapters = {}
        for dcs_name, endpoint in dcs_endpoints.items():
            self.dcs_adapters[dcs_name] = CppServerAdapter(endpoint)

        self.requester = "mimir"

        self.sleep_time = sleep_time

    def _send(self, body: Dict[str, Any]) -> Tuple[bool, str]:
        rep = self.publish_z.send_payload(body)
        if not rep or "reply" not in rep:
            return False, "no-reply"
        content = rep["reply"].get("content", "ERROR")
        return (content == "OK" or content != "ERROR"), str(content)

    def run(self):
        """
        check if any new data has arrive from the scripts, and
        also poll the cpp databases for new data.
        """
        while True:
            # TODO: think about ways of making all publish in parallel
            self.script_z.fetch()
            self.publish_script_data()

            # self.publish_baldr_to_wag()
            # self.publish_hdlr_databases_to_wag()

            # TODO: database classes to know if they are different each call,
            # and only publish the different data...

            time.sleep(self.sleep_time)
            print("Sleeping for new commands...")

    def publish_bld_databases_to_wag(self):
        adapter_names = [f"BLD{idx}" for idx in range(1, 5)]
        for adapter in adapter_names:
            self.dcs_adapters[adapter].fetch()

        # BLD: for each BaldrTscopeStatus, need to change to a list of values
        # for each parameter (per adapter)
        bld_parameters = fields(BaldrTscopeStatus)

        body = self.ESO_format([])

        # append to body["parameter"]
        for param in bld_parameters:
            values = [self.dcs_adapters[x][param] for x in adapter_names]
            prop = {}
            prop["name"] = f"bld_{param}"
            prop["range"] = "(0:3)"
            prop["value"] = values
            body["parameter"].append(prop)

        ok, msg = self._send(body)

        if not ok:
            logging.warning(f"failed to write script data to wag: {msg}")

    def publish_hdlr_databases_to_wag(self):
        self.dcs_adapters["HDLR"].fetch()

        Hdlr_parameters = fields(BaldrTscopeStatus)

        body = self.ESO_format([])

        # append to body["parameter"]
        for param in Hdlr_parameters:
            values = self.dcs_adapters["HDLR"][param]  # is already a list
            prop = {}
            prop["name"] = f"hdlr_{param}"
            prop["range"] = "(0:3)"
            prop["value"] = values
            body["parameter"].append(prop)

        ok, msg = self._send(body)
        if not ok:
            logging.warning(f"failed to write script data to wag: {msg}")

    def publish_script_data(self):
        if self.script_z.has_new_data:
            data = self.script_z.read_data()
            #TODO: if script data has range
        else:
            return
        if data is None or not isinstance(data, list) or len(data) == 0:
            return

        for i, item in enumerate(data):
            if not isinstance(item, dict) or len(item) != 1:
                logging.warning(f"ignoring malformed script data item: {item}")
                continue
            key = list(item.keys())[0]
            value = item[key]
            data[i] = {"name": key, "value": value}

        # need to append the requester field to each item
        for item in data:
            item["requester"] = self.requester

        # for any lists in data, need to add the "range" field
        for item in data:
            if isinstance(item.get("value"), (list, tuple)):
                item["range"] = "(0:3)"

        # write all fields to MCS in a single message
        body = self.ESO_format(data)

        ok, msg = self._send(body)

        if not ok:
            logging.warning(f"failed to write script data to wag: {msg}")

    def ESO_format(self, content):
        return {
            "command": {
                "name": "write",
                "time": self.ts(),
                "parameter": content,
                "requester": self.requester,
            }
        }

    def publish_baldr_to_wag(self):
        """
        example - publish all of baldr.

        Need to fetch for all beams, then group each paramaeter into a list of size 4
        then send that in one packet to wag
        """
        data = []

        for beam_idx in range(1, 5):
            st = self.dcs_adapters[f"BLD{beam_idx}"].fetch()
            if not st:
                logging.warning(f"no Baldr status for beam {beam_idx}")
                return
            data.append(st)

        # write all fields to MCS in a single message
        body = self.ESO_format([])

        # populate the parameter list with all fields from the dataclass,
        # each field will be a list of 4 values (one per beam)
        param_list = []
        for field in asdict(data[0]).keys():
            values = [getattr(d, field) for d in data]
            param_list.append(
                {
                    "name": f"bld_{field}",
                    "value": values,
                    "requester": self.requester,
                }
            )

        # update body
        body["command"]["parameter"] = param_list

        ok, msg = self._send(body)
        if not ok:
            logging.warning(f"failed to write baldr1_status to wag: {msg}")

    @staticmethod
    def ts():
        current_utc_time = datetime.now(timezone.utc)
        # Format the UTC time
        return current_utc_time.strftime("%Y-%m-%dT%H:%M:%S")


# ---------------- Server adapters ----------------
@dataclass
class BaldrTscopeStatus:
    """
    data class for all "unique per telescope" status fields
    """

    TT_state: int
    HO_state: int
    mode: str
    phasemask: str
    frequency: float
    configured: int
    ctrl_type: str
    complete: bool
    config_file: str
    inj_enabled: int
    auto_loop: int
    close_on_snr: float
    open_on_snr: float
    close_on_strehl: float
    open_on_strehl: float
    TT_offsets: int
    x_pup_offset: float
    y_pup_offset: float


class CppServerAdapter:
    def __init__(self, endpoint: str):
        self.z = ZmqReq(endpoint)

    def fetch(self) -> Optional[Any]:
        raise NotImplementedError


class BaldrAdapter(CppServerAdapter):
    """
    Talks to Baldr ZMQ server ("tcp://host:6662") and returns a BaldrStatus.
    """

    def __init__(self, host="127.0.0.1", port=6662):
        super().__init__(f"tcp://{host}:{port}")

        self.cur_status = None

    def fetch(self) -> Optional[BaldrTscopeStatus]:
        """
        Queries Baldr Cpp servers. Expected return of the form:
        {
            "ok": True,
            "status": {
                "TT_state": int,
                ... etc
            }
        }
        """
        # Example: ask a 'status' command.. this needs to be defined in the baldr or heim commander functs
        rep = self.z.send_payload({"cmd": "status"})

        if not rep or rep.get("ok") is False:
            return None

        # ---- Map server reply -> BaldrStatus  ----
        # Below assumes the server returns a dict with keys matching your fields.
        try:
            st = rep["status"]

            self.cur_status = BaldrTscopeStatus(
                TT_state=int(st["TT_state"]),
                HO_state=int(st["HO_state"]),
                mode=st["mode"],
                phasemask=st["phasemask"],
                frequency=float(st["frequency"]),
                configured=int(st["configured"]),
                ctrl_type=st["ctrl_type"],
                complete=bool(st["complete"]),
                config_file=st["config_file"],
                inj_enabled=int(st["inj_enabled"]),
                auto_loop=int(st["auto_loop"]),
                close_on_snr=float(st["close_on_snr"]),
                open_on_snr=float(st["open_on_snr"]),
                close_on_strehl=float(st["close_on_strehl"]),
                open_on_strehl=float(st["open_on_strehl"]),
                TT_offsets=int(st["TT_offsets"]),
                x_pup_offset=float(st["x_pup_offset"]),
                y_pup_offset=float(st["y_pup_offset"]),
            )
        except KeyError:
            return None


@dataclass
class HeimdallrStatus:
    hdlr_x_offset: list[float]
    hdlr_y_offset: list[float]
    hdlr_complete: bool


class HeimdallrAdapter(CppServerAdapter):
    """
    Talks to Heimdallr ZMQ server ("tcp://host:6660") and returns a HeimdallrStatus.
    """

    def __init__(self, host="127.0.0.1", port=6660):
        super().__init__(f"tcp://{host}:{port}")

    def fetch(self) -> Optional[HeimdallrStatus]:
        rep = self.z.send_payload({"cmd": "status"})
        if not rep or rep.get("ok") is False:
            return None

        try:
            st = rep["status"]
            return HeimdallrStatus(
                hdlr_x_offset=[float(x) for x in st["hdlr_x_offset"]],
                hdlr_y_offset=[float(y) for y in st["hdlr_y_offset"]],
                hdlr_complete=bool(st["hdlr_complete"]),
            )
        except KeyError:
            return None


class ScriptAdapter:
    """
    A dedicated channel for talking to DCS command scripts that have been run
    This differs from the C++ servers in that it has be the server side of a ZMQ REQ/REP pair
    and the scripts are the clients. When the data is dumped by the client, this class
    just returns "ack" to the script and the data is saved in MCS.

    The expectation for the data format is a list of dicts like:
    [
        {"name": "param1", "value": val1},
        {"name": "param2", "value": val2},
        ...
    ]
    noting that the MCS will append the requester field automatically.
    """

    def __init__(self, endpoint: str):
        self.z = ZmqRep(endpoint)

        self.poller = zmq.Poller()
        self.poller.register(self.z.s, zmq.POLLIN)

        self.data = {}
        self.has_new_data = False

    def read_data(self):
        self.has_new_data = False
        return self.data

    def fetch(self) -> Optional[Dict[str, Any]]:
        socks = dict(self.poller.poll(10))
        inputready = []
        if self.z.s in socks and socks[self.z.s] == zmq.POLLIN:
            inputready.append(self.z.s)
        for s in inputready:  # loop through our array of sockets/inputs
            msg = self.socket_funct(s)
            logging.info(f"Received message: {msg}")
            self.handle_message(msg)
            self.has_new_data = True

    def socket_funct(self, s):
        try:
            message = s.recv_string()
            return message
        except zmq.ZMQError as e:
            # logging.error(f"ZMQ Error: {e}")
            return -1

    def handle_message(self, msg):
        msg = dict(json.loads(msg))
        # Acknowledge receipt
        self.z.send_payload({"ok": True})
        if not msg:
            return None
        
        if msg.get("origin") == "s_h-autoalign":
            self.data = msg.get("data", {})
        elif msg.get("origin") != "s_bld_pup_autoalign_sky":
            # Save the data for later processing
            self.data = msg.get("data", {})


# ---------------- Main publish loop ----------------
if __name__ == "__main__":
    mcs = MCSClient(
        dcs_endpoints={
            # "BLD1": "tcp://192.168.100.2:6662",
            # "BLD2": "tcp://192.168.100.2:6663",
            # "BLD3": "tcp://192.168.100.2:6664",
            # "BLD4": "tcp://192.168.100.2:6665",
            # "HDLR": "tcp://192.168.100.2:6660",
        },
        script_endpoint="tcp://192.168.100.2:7019",
        publish_endpoint="tcp://192.168.100.1:7050",
        sleep_time=1.0,
    )

    mcs.run()
