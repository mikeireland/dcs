import zmq
import numpy as np
import time

# ### A toy server to provide a basic test for the baldr_love_gui.py with no dependancies 


ctx = zmq.Context()
socket = ctx.socket(zmq.REP)
socket.bind("tcp://*:5555")

print("Unified ZMQ server on port 5555")

while True:
    try:
        msg = socket.recv_string() 
        print("Received:", msg)

        if msg.startswith("status "):
            signal = msg[7:]
            print( signal )
            t = time.time()
            if signal == "Signal 1":
                val = np.sin(t)
            elif signal == "Signal 2":
                val = np.cos(t)
            elif signal == "Signal 3":
                val = np.sin(t * 2) * np.exp(-0.1 * t % 10)
            else:
                val = 0.0
            socket.send_string(str(val))
        elif msg.startswith("testcmd "):
            socket.send_string(f"ACK testcmd: Executed '{msg}'")
        else: #elif msg.startswith("COMMAND "):
            #command = msg[8:]
            print("Executing command:", msg)
            socket.send_string(f"ACK: Executed '{msg}'")
        #else:
        #    socket.send_string("ERR: Unknown request")

    except Exception as e:
        socket.send_string(f"Error: {e}")

# context = zmq.Context()
# socket = context.socket(zmq.REP)
# socket.bind("tcp://*:5555")

# print("ZMQ test server running on tcp://*:5555...")

# while True:
#     try:
#         msg = socket.recv_string()
#         t = time.time()
#         if msg == "Signal 1":
#             value = np.sin(t)
#         elif msg == "Signal 2":
#             value = np.cos(t)
#         elif msg == "Signal 3":
#             value = np.sin(t * 2) * np.exp(-0.1 * t % 10)
#         else:
#             value = 0.0
#         socket.send_string(str(value))
#     except KeyboardInterrupt:
#         break


