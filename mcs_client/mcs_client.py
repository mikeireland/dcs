# baldr_wag_client.py
import json, time, socket
from dataclasses import dataclass, asdict
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


def normalize_to_1d_and_shape(arr):
    """
    Accepts: [x0, x1, ...]  or  [[r0c0, r0c1, ...], [r1c0, ...], ...]
    Returns: (flat_list, (rows, cols))
    """
    if isinstance(arr, (list, tuple)) and all(
        not isinstance(v, (list, tuple)) for v in arr
    ):
        # 1 x N
        return [float(v) for v in arr], (1, len(arr))

    if isinstance(arr, (list, tuple)) and all(
        isinstance(r, (list, tuple)) for r in arr
    ):
        rows = len(arr)
        cols = len(arr[0]) if rows else 0
        # sanity: rectangular
        for r in arr:
            if len(r) != cols:
                raise ValueError("gains matrix is ragged; expected equal-length rows")
        flat = [float(x) for row in arr for x in row]  # row-major
        return flat, (rows, cols)

    raise TypeError("gains must be a list or list-of-lists")


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
        print("payload", payload)
        print("dumped payload", json.dumps(payload, sort_keys=True))
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
    ):
        self.publish_z = ZmqReq(publish_endpoint)
        print(f"REQ publish set up on {publish_endpoint}")

        self.script_z = ScriptAdapter(script_endpoint)
        print(f"ScriptAdapter(REP) set up on {script_endpoint}")

        self.dcs_adapters = {}
        for dcs_name, endpoint in dcs_endpoints.items():
            self.dcs_adapters[dcs_name] = CppServerAdapter(endpoint)

        self.requester = "mimir"

    def _send(self, body: Dict[str, Any]) -> Tuple[bool, str]:
        rep = self.publish_z.send_payload(body)
        if not rep or "reply" not in rep:
            return False, "no-reply"
        content = rep["reply"].get("content", "ERROR")
        return (content == "OK" or content != "ERROR"), str(content)

    def publish_script_data(self):
        if self.script_z.has_new_data:
            data = self.script_z.read_data()
        else:
            return
        if data is None or not isinstance(data, list) or len(data) == 0:
            return
        
        # need to append the requester field to each item
        for item in data:
            item["requester"] = self.requester

        # for any lists in data, need to add the "range" field
        for item in data:
            print(type(item.get("value")))
            if isinstance(item.get("value"), (list, tuple)):
                item["range"] = "(0:3)"

        # write all fields to MCS in a single message
        body = {
            "command": {
                "name": "write",
                "time": self.ts(),
                "parameter": data,
            }
        }   

        print(f" sending the following to wag")
        print(body)

        ok, msg = self._send(body)

        if not ok:
            print(f"WARN: failed to write script data to wag: {msg}")

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
                print(f"WARN: no Baldr status for beam {beam_idx}")
                return

            data.append(st)

        # write all fields to MCS in a single message
        body = {
            "command": {
                "name": "write",
                "time": self.ts(),
                "parameter": [
                    # each field will go here
                ],
            }
        }

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
            print(f"WARN: failed to write baldr1_status to wag: {msg}")

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
        self.has_new_data=False
        return self.data

    def fetch(self) -> Optional[Dict[str, Any]]:
        socks = dict(self.poller.poll(10))
        inputready = []
        if self.z.s in socks and socks[self.z.s] == zmq.POLLIN:
            inputready.append(self.z.s)
        for s in inputready:  # loop through our array of sockets/inputs
            msg = self.socket_funct(s)
            print(f"Received message: {msg}")
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
        if not msg or msg.get("cmd") != "dump":
            return None

        # Acknowledge receipt
        self.z.send_payload({"ok": True, "msg": "ack"})

        # Save the data for later processing
        self.data = msg.get("data", {})



# ---------------- Main publish loop ----------------


if __name__ == "__main__":
    mcs = MCSClient(
        dcs_endpoints={
            # "BLD1": "tcp://192.168.100.2:7019",
            # "BLD2": "tcp://192.168.100.2:7020",
            # "BLD3": "tcp://192.168.100.2:7021",
            # "BLD4": "tcp://192.168.100.2:7022",
        },
        script_endpoint="tcp://192.168.100.2:7019",
        publish_endpoint="tcp://192.168.100.1:7050",
    )

    while True:
        print("fetching data...")
        mcs.script_z.fetch()

        mcs.publish_script_data()
        # mcs.publish_baldr_to_wag()
        time.sleep(1)
