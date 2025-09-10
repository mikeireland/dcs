"""
A script to save the FT performance, namely:
- The complex value at the centre of each splodge, from Frantz's code
- The dm pistons and dl offloads from Heimdallr

Done at a ~1kHz rate, saved to a .npz file for later analysis.
"""

import zmq
import json
import numpy as np
from typing import Any, Dict, Optional
import time


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

    def send_payload(
        self, payload: Dict[str, Any], is_str=False, decode_ascii=True
    ) -> Optional[Dict[str, Any]]:
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


def collect_ft_performance(duration_sec=60, rate_hz=1000):
    keys_of_interest = [
        "gd_snr",
        "pd_snr",
    ]

    # Initial call to get array shapes
    first_reply = h_z.send_payload("status", is_str=True, decode_ascii=False)
    if not first_reply:
        raise RuntimeError("Initial status reply failed.")

    n_samples = duration_sec * rate_hz
    # Determine the shape for each key
    key_shapes = {k: np.shape(first_reply[k]) for k in keys_of_interest}
    # Pre-allocate arrays for each key
    data = {
        k: np.zeros((n_samples,) + key_shapes[k], dtype=np.array(first_reply[k]).dtype)
        for k in keys_of_interest
    }

    start_time = time.perf_counter()
    for i in range(n_samples):
        t0 = time.perf_counter()
        reply = h_z.send_payload("status", is_str=True, decode_ascii=False)
        if reply:
            for k in keys_of_interest:
                data[k][i] = reply[k]
        elapsed = time.perf_counter() - t0
        sleep_time = max(0, (1.0 / rate_hz) - elapsed)
        time.sleep(sleep_time)
        if (time.perf_counter() - start_time) > duration_sec:
            break

    np.savez("ft_performance_data.npz", **data)


def test_max_rate(n_samples=10000):
    keys_of_interest = [
        "gd_snr",
        "pd_snr",
    ]
    first_reply = h_z.send_payload("status", is_str=True, decode_ascii=False)
    if not first_reply:
        raise RuntimeError("Initial status reply failed.")
    key_shapes = {k: np.shape(first_reply[k]) for k in keys_of_interest}
    data = {
        k: np.zeros((n_samples,) + key_shapes[k], dtype=np.array(first_reply[k]).dtype)
        for k in keys_of_interest
    }
    timestamps = np.zeros(n_samples, dtype=np.float64)

    start = time.perf_counter()
    for i in range(n_samples):
        reply = h_z.send_payload("status", is_str=True, decode_ascii=False)
        timestamps[i] = time.time()
        if reply:
            for k in keys_of_interest:
                data[k][i] = reply[k]
    elapsed = time.perf_counter() - start
    print(
        f"Collected {n_samples} samples in {elapsed:.3f} s ({n_samples/elapsed:.1f} Hz)"
    )


# Example usage:
if __name__ == "__main__":
    # collect_ft_performance()
    test_max_rate()
