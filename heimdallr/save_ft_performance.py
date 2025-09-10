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
import argparse
import threading


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


def collect_ft_performance(
    duration_sec=60, rate_hz=1000, save_path="ft_performance_data.npz"
):
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

    np.savez(save_path, **data)


def test_max_rate(n_samples=10000, save_path="ft_speedtest_data.npz"):
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
    np.savez(save_path, **data, timestamps=timestamps)


def query_server(zmq_req, key_list, n_samples, out_dict, label):
    first_reply = zmq_req.send_payload("status", is_str=True, decode_ascii=False)
    key_shapes = {k: np.shape(first_reply[k]) for k in key_list}
    data = {
        k: np.zeros((n_samples,) + key_shapes[k], dtype=np.array(first_reply[k]).dtype)
        for k in key_list
    }
    timestamps = np.zeros(n_samples, dtype=np.float64)
    for i in range(n_samples):
        reply = zmq_req.send_payload("status", is_str=True, decode_ascii=False)
        timestamps[i] = time.time()
        if reply:
            for k in key_list:
                data[k][i] = reply[k]
    out_dict[label] = (data, timestamps)


def collect_multi_server(
    endpoints,
    key_lists,
    duration_sec=60,
    rate_hz=1000,
    save_path="ft_multiserver_data.npz",
):
    n_samples = duration_sec * rate_hz
    results = {}
    threads = []
    for i, (endpoint, key_list) in enumerate(zip(endpoints, key_lists)):
        zmq_req = ZmqReq(endpoint)
        label = f"server{i+1}"
        t = threading.Thread(
            target=query_server, args=(zmq_req, key_list, n_samples, results, label)
        )
        threads.append(t)
        t.start()
    for t in threads:
        t.join()
    # Flatten results for saving
    save_dict = {}
    for label, (data, timestamps) in results.items():
        for k, arr in data.items():
            save_dict[f"{label}_{k}"] = arr
        save_dict[f"{label}_timestamps"] = timestamps
    np.savez(save_path, **save_dict)


# Example usage:
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="FT performance data collection script"
    )
    parser.add_argument(
        "mode",
        choices=["speedtest", "single", "multi"],
        help="Operating mode: speedtest, single, or multi-server",
    )
    parser.add_argument(
        "--save_path", type=str, default=None, help="Path to save the .npz file"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=60,
        help="Duration in seconds (single/multi mode)",
    )
    parser.add_argument(
        "--rate", type=int, default=1000, help="Sample rate in Hz (single/multi mode)"
    )
    parser.add_argument(
        "--n_samples",
        type=int,
        default=10000,
        help="Number of samples (speedtest mode)",
    )
    parser.add_argument(
        "--endpoints",
        nargs="*",
        help="Endpoints for multi-server mode (space separated)",
    )
    parser.add_argument(
        "--keys",
        nargs="*",
        help="Keys for each server in multi-server mode, comma-separated per server (e.g. key1,key2 key3,key4)",
    )

    args = parser.parse_args()

    if args.mode == "speedtest":
        save_path = args.save_path or "ft_speedtest_data.npz"
        test_max_rate(n_samples=args.n_samples, save_path=save_path)
    elif args.mode == "single":
        save_path = args.save_path or "ft_performance_data.npz"
        collect_ft_performance(
            duration_sec=args.duration, rate_hz=args.rate, save_path=save_path
        )
    elif args.mode == "multi":
        if not args.endpoints or not args.keys:
            raise ValueError(
                "In multi mode, you must provide --endpoints and --keys arguments."
            )
        if len(args.endpoints) != len(args.keys):
            raise ValueError("Number of endpoints and keys must match.")
        key_lists = [k.split(",") for k in args.keys]
        save_path = args.save_path or "ft_multiserver_data.npz"
        collect_multi_server(
            args.endpoints,
            key_lists,
            duration_sec=args.duration,
            rate_hz=args.rate,
            save_path=save_path,
        )
