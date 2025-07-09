#!/usr/bin/env python

''' ---------------------------------------------------------------------------
Simple ZMQ "shell" interface to send configuration commands to the CRED1
image fetching program

This is for testing purposes only. Higher-level wavefront control software
will directly interact with the server using the same ZMQ messaging mechanism.

I picked the arbirary 6667 (an extra-long number of the beast) port number
since I suspect that 5555 is already used by other programs on Mimir!

-Frantz.
--------------------------------------------------------------------------- '''

import zmq
import json

server_port = 6667
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect(f"tcp://mimir:{server_port}")
import datetime

print(f"ZMQ shell interface to talk to the cam server on port {server_port}")
logfile = "/home/asg/Progs/repos/dcs/asgard-cred1-server/command_log.log"

cmd_sz = 10  # finite size command with blanks filled


while True:
    cmd = input(f"ZMQ ({server_port}) >> ")
    out_cmd = cmd + (cmd_sz - 1 - len(cmd)) * " " # fill the blanks

    now = datetime.datetime.utcnow()

    with open(logfile, "a") as log:
        log.write(f"{now.year:4d}{now.month:02d}{now.day:02d} - T{now.hour:02d}:{now.minute:02d}:{now.second:02d} : {out_cmd}\n")

    socket.send_string(out_cmd)

    #  Get the reply.
    resp = socket.recv().decode("ascii")
    # print(f"== Reply: [{resp}]")
    try: 
    	resp = json.loads(resp)
    	if isinstance(resp,dict):
    		print(json.dumps(resp, indent=4))
    	else:
    		print(resp)
    except json.JSONDecodeError:
    	print(resp)
    if resp == "BYE!":
        break
