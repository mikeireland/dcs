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
        self, payload, is_str=False, decode_ascii=True
    ) -> Optional[dict]:
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
    with open(log_path, "a") as f:
        while True:
            t0 = time.time()
            reply = h_z.send_payload("status", is_str=True, decode_ascii=False)
            if reply:
                # Timestamp to ms precision
                timestamp = "{:.3f}".format(t0)
                # Flatten all key values into a single line
                values = []
                for k in keys_of_interest:
                    v = reply.get(k)
                    if isinstance(v, (list, np.ndarray)):
                        values.extend([str(x) for x in v])
                    else:
                        values.append(str(v))
                line = "{} {}".format(timestamp, " ".join(values))
                f.write(line + "\n")
                f.flush()
            time.sleep(max(0, (1.0 / rate_hz) - (time.time() - t0)))


# Example usage:
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="FT performance data logging script"
    )
    parser.add_argument(
        "--log_path", type=str, default="ft_performance_log.txt", help="Path to log file"
    )
    parser.add_argument(
        "--rate", type=int, default=1000, help="Sample rate in Hz"
    )
    args = parser.parse_args()
    log_ft_performance(log_path=args.log_path, rate_hz=args.rate)
    time.sleep(sleep_time)
        # if (time.perf_counter() - start_time) > duration_sec:
        #     break

    np.savez(save_path, **data, timestamps=timestamps)


def test_max_rate(n_samples=10000, save_path="ft_speedtest_data.npz"):

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
    if save_path is not None:
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
        # Allow save_path to be None (no file saved)
        test_max_rate(n_samples=args.n_samples, save_path=args.save_path)
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
