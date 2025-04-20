#!/usr/bin/env python

''' ---------------------------------------------------------------------------
Simple ZMQ "shell" interface to send configuration commands to the  Multi-DM
driver.

This is for testing purposes only. Higher-level wavefront control software
will directly interact with the server using the same ZMQ messaging mechanism.

I picked the arbirary 6666 (an extra-long number of the beast) port number
since I suspect that 5555 is already used by other programs on Mimir!

-Frantz.
--------------------------------------------------------------------------- '''

import zmq

server_port = 6666
context = zmq.Context()
socket = context.socket(zmq.REQ)
# socket.connect(f"tcp://mimir:{server_port}")
socket.connect(f"tcp://127.0.0.1:{server_port}")

print(f"ZMQ shell interface to talk to the DM server on port {server_port}")

cmd_sz = 10  # finite size command with blanks filled

while True:
    cmd = input(f"ZMQ ({server_port}) >> ")
    out_cmd = cmd + (cmd_sz - 1 - len(cmd)) * " " # fill the blanks
    socket.send_string(out_cmd)
    
    #  Get the reply.
    resp = socket.recv().decode("ascii")
    print(f"== Reply: [{resp}]")

    if resp == "BYE!":
        break
