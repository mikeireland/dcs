"""
A script to save the FT performance, namely:
- The complex value at the centre of each splodge, from Frantz's code
- The dm pistons and dl offloads from Heimdallr

Logs to a text file, running indefinitely until interrupted.
"""

import zmq
import json
import numpy as np
import time
import argparse
from typing import Optional
import os

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

    def send_payload(self, payload, is_str=False, decode_ascii=True) -> Optional[dict]:
        if not is_str:
            self.s.send_string(json.dumps(payload, sort_keys=True))
        else:
            self.s.send_string(payload)
        try:
            if decode_ascii:
                res = self.s.recv().decode("ascii")[:-1]
            else:
                res = self.s.recv_string()
            return json.loads(res)
        except zmq.error.Again:
            return None
        except json.decoder.JSONDecodeError:
            return None


h_z = ZmqReq("tcp://192.168.100.2:6660")

keys_of_interest = [
    "gd_snr",
    "pd_snr",
    "gd_bl",
    "pd_tel",
    "gd_tel",
    "dm_piston",
]


def log_ft_performance(log_path="ft_performance_log.txt", rate_hz=1000):
    # Write header only if file is empty
    write_header = True
    try:
        with open(log_path, "r") as f_check:
            if f_check.read(1):
                write_header = False
    except FileNotFoundError:
        pass
    with open(log_path, "a") as f:
        if write_header:
            f.write(
                "# timestamp gd_snr pd_snr gd_bl pd_tel gd_tel dm_piston (all values space-separated, 3 decimal places)\n"
            )
        last_cnt = 0
        while True:
            t0 = time.time()
            reply = h_z.send_payload("status", is_str=True, decode_ascii=False)
            if reply:
                if "cnt" in reply:
                    cnt = reply["cnt"]
                    if last_cnt == cnt:
                        continue
                    last_cnt = cnt
                # Timestamp to ms precision
                timestamp = "{:.3f}".format(t0)
                # Flatten all key values into a single line, 3 decimal places
                values = []
                for k in keys_of_interest:
                    v = reply.get(k)
                    if isinstance(v, (list, np.ndarray)):
                        values.extend(
                            [
                                "{:.3f}".format(float(x) if x is not None else np.nan)
                                for x in v
                            ]
                        )
                    else:
                        try:
                            values.append("{:.3f}".format(float(v)))
                        except Exception:
                            values.append(str(v))
                line = "{} {}".format(timestamp, " ".join(values))
                f.write(line + "\n")
                f.flush()
            time.sleep(max(0, (1.0 / rate_hz) - (time.time() - t0)))


# Example usage:
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="FT performance data logging script")

    parser.add_argument("--rate", type=int, default=1000, help="Sample rate in Hz")
    args = parser.parse_args()
    # time in UTC
    cur_datetime = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    fname = f"ft_performance_{cur_datetime}.log"
    year_month_day = time.strftime("%Y%m%d", time.gmtime())
    pth = f"/data/{year_month_day}"
    full_pth = os.path.join(pth, fname)
    log_ft_performance(log_path=full_pth, rate_hz=args.rate)
