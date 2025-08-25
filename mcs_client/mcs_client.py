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


# ---------------- ZMQ helpers ----------------
def ts():
    # ISO-8601 UTC for MCS "time" fields (doc shows generic "<timestamp>")
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")

def normalize_to_1d_and_shape(arr):
    """
    Accepts: [x0, x1, ...]  or  [[r0c0, r0c1, ...], [r1c0, ...], ...]
    Returns: (flat_list, (rows, cols))
    """
    if isinstance(arr, (list, tuple)) and all(not isinstance(v, (list, tuple)) for v in arr):
        # 1 x N
        return [float(v) for v in arr], (1, len(arr))

    if isinstance(arr, (list, tuple)) and all(isinstance(r, (list, tuple)) for r in arr):
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
    def __init__(self, endpoint: str, timeout_ms: int = 1500):
        self.ctx = zmq.Context.instance()
        self.s = self.ctx.socket(zmq.REQ)
        self.s.RCVTIMEO = timeout_ms
        self.s.SNDTIMEO = timeout_ms
        self.s.connect(endpoint)

    def ask(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        self.s.send_string(json.dumps(payload))
        try:
            return json.loads(self.s.recv_string())
        except zmq.error.Again:
            return None

# ---------------- MCS client (WAG) ----------------
class MCSClient:
    def __init__(self, host: str = "wag", port: int = 7020, requester: str = "mimir"):
        self.requester = requester
        self.z = ZmqReq(f"tcp://{host}:{port}")

    def _send(self, body: Dict[str, Any]) -> Tuple[bool, str]:
        rep = self.z.ask(body)
        if not rep or "reply" not in rep:
            return False, "no-reply"
        content = rep["reply"].get("content", "ERROR") 
        return (content == "OK" or content != "ERROR"), str(content)

    def write_scalar(self, name: str, value: Any) -> Tuple[bool, str]:
        # Some agomcs versions accept 'value' with scalar writes; if not, define your scalar as length-1 vector.
        msg = {
            "command": {
                "name": "write",
                "time": ts(),
                "parameter": {
                    "name": name,
                    "value": value,
                    "requester": self.requester
                }
            }
        }
        return self._send(msg)  # Reply "OK"/"ERROR". See 8.7.2. ASGARD Top-Level Control Software v.4.2

    def write_vector(self, name: str, values: List[Any], i0: int = 0) -> Tuple[bool, str]:
        msg = {
            "command": {
                "name": "write",
                "time": ts(),
                "parameter": {
                    "name": name,
                    "range": f"({i0}:{i0+len(values)-1})",
                    "value": values,
                    "requester": self.requester
                }
            }
        }
        return self._send(msg)  # Vector write with range/value. See 8.7.2.  [oai_citation:15‡ASGARD Top-Level Control Software v.4.2.pdf](file-service://file-1tWRLXu5uuX5KwAVLhsy2R)

    def read_scalar(self, name: str):
        """Return (ok, value|error_str)."""
        msg = {
            "command": {
                "name": "read",
                "time": ts(),
                "parameter": {"name": name}
            }
        }
        rep = self.z.ask(msg)
        if not rep or "reply" not in rep:
            return False, "no-reply"
        content = rep["reply"].get("content", "ERROR")
        return (content != "ERROR"), content  # spec: content is the value or "ERROR"

    def read_vector(self, name: str, i0: int, j0: int):
        """Return (ok, [values]|error_str). Indices are inclusive."""
        msg = {
            "command": {
                "name": "read",
                "time": ts(),
                "parameter": {
                    "name": name,
                    "range": f"({i0}:{j0})"
                }
            }
        }
        rep = self.z.ask(msg)
        if not rep or "reply" not in rep:
            return False, "no-reply"
        content = rep["reply"].get("content", "ERROR")
        return (content != "ERROR"), content

# ---------------- Server adapters ----------------
@dataclass
class BaldrStatus:
    loop_state: str              # "open"/"close"
    mode: str                    # "bright"/"faint"
    phasemask: str               # "J1".."H5"
    frequency: float             # 1000.0Hz
    configured: int              # 0/1
    ctrl_type: str               # "PID"/"Leaky"/"Kalman"
    gains: List[float]           # flattened, depends on control type
    gains_shape: Tuple[int, int] # (rows, cols)
    strehl_est: float
    flux: List[int]              # len 140
    busy: int                    # 0/1
    config_file: str
    setpoint_lo: List[float]
    setpoint_ho: List[float]
    oloff_lo: List[float]
    oloff_ho: List[float]

class BaldrAdapter:
    """
    Talks to Baldr ZMQ server ("tcp://host:6662") and returns a BaldrStatus.
    
    """
    def __init__(self, host="127.0.0.1", port=6662):
        self.z = ZmqReq(f"tcp://{host}:{port}")

    def fetch(self) -> Optional[BaldrStatus]:
        # Example: ask a 'status' command.. this needs to be defined in the baldr or heim commander functs 
        rep = self.z.ask({"cmd": "status"})
        if not rep or rep.get("ok") is False:
            return None

        # ---- Map server reply -> BaldrStatus  ----
        # Below assumes the server returns a dict with keys matching your fields.
    
        try:
            st = rep["status"]
            try:
                # gains is 1D or 2D depending on controller type. 
                # by default I flatten it and store to original shape
                raw_gains = st["gains"]  # may be 1D or 2D
                flat_gains, (R, C) = normalize_to_1d_and_shape(raw_gains)
            except:
                return None
            
            return BaldrStatus(
                loop_state   = st["loop_state"],
                mode         = st["mode"],
                phasemask    = st["phasemask"],
                frequency    = float( st['frequency'] ),
                configured   = int(st["configured"]),
                ctrl_type    = st["ctrl_type"],
                gains        = flat_gains,
                gains_shape  = (R, C),
                strehl_est   = float(st["strehl_est"]),
                flux         = list(map(int, st["flux"])),
                busy         = int(st["busy"]),
                config_file  = st["config_file"],
                setpoint_lo  = list(map(float, st["setpoint_lo"])),
                setpoint_ho  = list(map(float, st["setpoint_ho"])),
                oloff_lo     = list(map(float, st["oloffset_lo"])),
                oloff_ho     = list(map(float, st["oloffset_ho"]))
            )
        except KeyError:
            return None

# ---------------- Main publish loop ----------------
def publish_baldr_to_wag(
    baldr_host="127.0.0.1", baldr_port=6662,
    wag_host="wag", wag_port=7020, requester="mimir"
):
    adat = BaldrAdapter(baldr_host, baldr_port)
    mcs  = MCSClient(wag_host, wag_port, requester=requester)

    st = adat.fetch()
    if not st:
        print("WARN: no Baldr status")
        return

    
    # Write strings/ints/floats
    ok1 = mcs.write_scalar("BALDR.LOOP_STATE",  st.loop_state)
    ok2 = mcs.write_scalar("BALDR.MODE",        st.mode)
    ok3 = mcs.write_scalar("BALDR.PHASEMASK",   st.phasemask)
    ok4 = mcs.write_scalar("BALDR.FREQUENCY",   st.frequency)
    ok5 = mcs.write_scalar("BALDR.CONFIGURED",  st.configured)
    ok6 = mcs.write_scalar("BALDR.CTRL_TYPE",   st.ctrl_type)
    ok7 = mcs.write_scalar("BALDR.STREHL_EST",  st.strehl_est)
    ok8 = mcs.write_scalar("BALDR.BUSY",        st.busy)
    ok9 = mcs.write_scalar("BALDR.CONFIG_FILE", st.config_file)

    # Write vectors (declare sizes in agmcfgMCS.cfg)
    mcs.write_vector("BALDR.GAINS",       st.gains,       0)
    mcs.write_vector("BALDR.GAINS_SHAPE", st.gains_shape, 0)
    mcs.write_vector("BALDR.FLUX",        st.flux,        0)
    mcs.write_vector("BALDR.SETPOINT_LO", st.setpoint_lo, 0)
    mcs.write_vector("BALDR.SETPOINT_HO", st.setpoint_ho, 0)
    mcs.write_vector("BALDR.OLOFF_LO",    st.oloff_lo,    0)
    mcs.write_vector("BALDR.OLOFF_HO",    st.oloff_ho,    0)

if __name__ == "__main__":
    # e/g
    publish_baldr_to_wag()