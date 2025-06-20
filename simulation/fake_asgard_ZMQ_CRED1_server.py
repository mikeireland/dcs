#!/usr/bin/env python3

import zmq
import threading
import time

# --- Shared State ---
camera_state = {
    "fps": 1000.0,
    "gain": 1,
    "status": "operational",
    "mode": "globalresetcds",
    "resetwidth": 20,
    "nbreadworeset": 1,
    "cropping": False,
    "cropping rows": "0-255",
    "cropping columns": "0-319"
}

mds_state = {
    "SBB": True
}

# --- Camera Command Handler ---
def handle_camera_command(cmd: str) -> str:
    #print(f"recieved: {cmd}")
    cmd = cmd.strip().lower()

    if cmd.startswith("cli "):
        cmd = cmd[4:].strip().strip('"')

    if cmd in ("fps", "fps raw"):
        return f"{camera_state['fps']:.2f} fps"
    elif cmd in ("gain", "gain raw"):
        return f"{camera_state['gain']:.2f}"
    elif cmd in ("status", "status raw"):
        return camera_state["status"]
    elif cmd in ("mode", "mode raw"):
        return camera_state["mode"]
    elif cmd in ("resetwidth", "resetwidth raw"):
        return str(camera_state["resetwidth"])
    elif cmd in ("nbreadworeset", "nbreadworeset raw"):
        return str(camera_state["nbreadworeset"])
    elif cmd in ("cropping", "cropping raw"):
        return "active" if camera_state["cropping"] else "inactive"
    elif cmd in ("cropping rows", "cropping rows raw"):
        return camera_state["cropping rows"]
    elif cmd in ("cropping columns", "cropping columns raw"):
        return camera_state["cropping columns"]

    elif cmd.startswith("set fps "):
        try:
            val = float(cmd.split()[-1])
            camera_state["fps"] = val
            return f"fps set to {val}"
        except Exception:
            return "Invalid fps value"
    elif cmd.startswith("set gain "):
        try:
            val = float(cmd.split()[-1])
            camera_state["gain"] = val
            return f"gain set to {val}"
        except Exception:
            return "Invalid gain value"
    elif cmd.startswith("set mode "):
        camera_state["mode"] = cmd[len("set mode "):]
        return f"mode set to {camera_state['mode']}"
    elif cmd.startswith("set resetwidth "):
        try:
            val = int(cmd.split()[-1])
            camera_state["resetwidth"] = val
            return f"resetwidth set to {val}"
        except Exception:
            return "Invalid resetwidth value"
    elif cmd.startswith("set nbreadworeset "):
        try:
            val = int(cmd.split()[-1])
            camera_state["nbreadworeset"] = val
            return f"nbreadworeset set to {val}"
        except Exception:
            return "Invalid nbreadworeset value"
    elif cmd == "set cropping on":
        camera_state["cropping"] = True
        return "cropping enabled"
    elif cmd == "set cropping off":
        camera_state["cropping"] = False
        return "cropping disabled"
    elif cmd.startswith("set cropping rows "):
        camera_state["cropping rows"] = cmd[len("set cropping rows "):]
        return f"cropping rows set to {camera_state['cropping rows']}"
    elif cmd.startswith("set cropping columns "):
        camera_state["cropping columns"] = cmd[len("set cropping columns "):]
        return f"cropping columns set to {camera_state['cropping columns']}"

    return f"Unknown or unsupported command: {cmd}"

# --- MDS Command Handler ---
def handle_mds_command(cmd: str) -> str:
    cmd = cmd.strip().lower()
    if cmd == "on sbb":
        mds_state["SBB"] = True
        return "OK"
    elif cmd == "off sbb":
        mds_state["SBB"] = False
        return "OK"
    else:
        return "NOK"

# --- Camera ZMQ Thread ---
def camera_server():
    ctx = zmq.Context()
    socket = ctx.socket(zmq.REP)
    socket.bind("tcp://127.0.0.1:6667")
    print("[FakeCameraServer] Listening on tcp://127.0.0.1:6667")
    while True:
        try:
            msg = socket.recv_string()
            print(f"[CameraZMQ] Received: {msg}")
            reply = handle_camera_command(msg)
            socket.send_string(f'"{reply}\\r\\n"')
        except Exception as e:
            print(f"[Camera ERROR] {e}")
            socket.send_string('"Error\\r\\n"')

# --- MDS ZMQ Thread ---
def mds_server():
    ctx = zmq.Context()
    socket = ctx.socket(zmq.REP)
    socket.bind("tcp://127.0.0.1:5555")
    print("[FakeMDSServer] Listening on tcp://127.0.0.1:5555")
    while True:
        try:
            msg = socket.recv_string()
            print(f"[MDSZMQ] Received: {msg}")
            reply = handle_mds_command(msg)
            socket.send_string(reply)
        except Exception as e:
            print(f"[MDS ERROR] {e}")
            socket.send_string("NOK")

# --- Run both servers ---
if __name__ == "__main__":
    threading.Thread(target=camera_server, daemon=True).start()
    threading.Thread(target=mds_server, daemon=True).start()

    print("[Simulation] Fake Camera + MDS server running...")
    while True:
        time.sleep(1)


""" 
# client to test eg.
import zmq

# --- Test Camera Server ---
ctx = zmq.Context()
cam_socket = ctx.socket(zmq.REQ)
cam_socket.connect("tcp://127.0.0.1:6667")
cam_socket.send_string('cli "gain"')
print("Camera reply:", cam_socket.recv_string())

# --- Test MDS Server ---
mds_socket = ctx.socket(zmq.REQ)
mds_socket.connect("tcp://127.0.0.1:5555")
mds_socket.send_string("on SBB")
print("MDS reply (on):", mds_socket.recv_string())
mds_socket.send_string("off SBB")
print("MDS reply (off):", mds_socket.recv_string())

"""