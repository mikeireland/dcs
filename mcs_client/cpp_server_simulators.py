import zmq
import json
import random
import time
import multiprocessing


def baldr_status():
    return {
        "TT_state": random.randint(0, 2),
        "HO_state": random.randint(0, 2),
        "mode": random.choice(["MODE1", "MODE2", "MODE3"]),
        "phasemask": random.choice(["PM1", "PM2"]),
        "frequency": random.uniform(0.0, 100.0),
        "configured": random.randint(0, 1),
        "ctrl_type": random.choice(["AUTO", "MANUAL"]),
        "complete": bool(random.getrandbits(1)),
        "config_file": random.choice(["file1.cfg", "file2.cfg"]),
        "inj_enabled": random.randint(0, 1),
        "auto_loop": random.randint(0, 1),
        "close_on_snr": random.uniform(0.0, 10.0),
        "open_on_snr": random.uniform(0.0, 10.0),
        "close_on_strehl": random.uniform(0.0, 1.0),
        "open_on_strehl": random.uniform(0.0, 1.0),
        "TT_offsets": random.randint(-10, 10),
        "x_pup_offset": random.uniform(-5.0, 5.0),
        "y_pup_offset": random.uniform(-5.0, 5.0),
    }


def hdlr_status():
    return {
        "hdlr_x_offset": [random.uniform(-5.0, 5.0) for _ in range(4)],
        "hdlr_y_offset": [random.uniform(-5.0, 5.0) for _ in range(4)],
        "hdlr_complete": bool(random.getrandbits(1)),
    }


def baldr_server(endpoint):
    ctx = zmq.Context()
    s = ctx.socket(zmq.REP)
    s.bind(endpoint)
    print(f"Baldr simulator running on {endpoint}")
    while True:
        try:
            msg = s.recv_string(zmq.NOBLOCK)
        except zmq.Again:
            time.sleep(0.01)
            continue
        try:
            req = json.loads(msg)
            print(endpoint, req)
        except Exception:
            s.send_string(json.dumps({"ok": False, "error": "bad json"}))
            continue
        if req.get("cmd") == "status":
            s.send_string(json.dumps({"ok": True, "status": baldr_status()}))
        else:
            s.send_string(json.dumps({"ok": False, "error": "unknown cmd"}))


def hdlr_server(endpoint):
    ctx = zmq.Context()
    s = ctx.socket(zmq.REP)
    s.bind(endpoint)
    print(f"Heimdallr simulator running on {endpoint}")
    while True:
        try:
            msg = s.recv_string(zmq.NOBLOCK)
        except zmq.Again:
            time.sleep(0.01)
            continue
        try:
            req = json.loads(msg)
            print(req)
        except Exception:
            s.send_string(json.dumps({"ok": False, "error": "bad json"}))
            continue
        if req.get("cmd") == "status":
            s.send_string(json.dumps({"ok": True, "status": hdlr_status()}))
        else:
            s.send_string(json.dumps({"ok": False, "error": "unknown cmd"}))


def spawn_all():
    # Endpoints as in mcs_client.py main
    endpoints = {
        "BLD1": "tcp://192.168.100.2:6662",
        "BLD2": "tcp://192.168.100.2:6663",
        "BLD3": "tcp://192.168.100.2:6664",
        "BLD4": "tcp://192.168.100.2:6665",
        "HDLR": "tcp://192.168.100.2:6650",
    }
    procs = []
    for name, ep in endpoints.items():
        if name.startswith("BLD"):
            p = multiprocessing.Process(target=baldr_server, args=(ep,))
        elif name == "HDLR":
            p = multiprocessing.Process(target=hdlr_server, args=(ep,))
        else:
            continue
        p.start()
        procs.append(p)
    print("All simulators started. Press Ctrl+C to exit.")
    try:
        for p in procs:
            p.join()
    except KeyboardInterrupt:
        print("Shutting down simulators...")
        for p in procs:
            p.terminate()


if __name__ == "__main__":
    spawn_all()
