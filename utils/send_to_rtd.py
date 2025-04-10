"""
Send a shared memory array to the RTD on wag2 via a push socket.

As a first test, we will create random data rather than using shared memory.

Data are binary, with a 64 byte header, followed by the data itself.
The header contains:

Camera name (32 bytes, terminated by null character).
Display type (2 bytes ; unsigned short in C).
X = size in pixels (2 bytes ; short in C).
Y = size in pixels (2 bytes ; short in C).
Pixel data type (1 byte ; char in C).
Filler (25 bytes ; to have a 64-byte header).

"""

import numpy as np
import zmq
import time
import signal
from xaosim.shmlib import shm
import sys

# The first argument is the name of the camera, the second is the shared memory name.
# we use argv to get the name of the camera and the shared memory name.
if len(sys.argv) > 1:
    camera_name = sys.argv[1]
else:
    print("No camera name provided. Using default: 'test'")
    camera_name = "test"
    
if len(sys.argv) > 2:
    shm_name = sys.argv[2]
    simulate_image=False
else:
    simulate_image = True
    print("No shared memory name provided. Simulating image only.")

# Set some parameters we'll later get from the command line
display_type = 0

if simulate_image: 
    # Also, some data we should get from the image itself.
    size_x = 100
    size_y = 100
    pixel_data_type = 16 # 16 bits per pixel, signed integer
else:
    # We should get the size from the shared memory object.
    SHM = shm(shm_name)
    imsize = SHM.get_data().shape
    size_x = imsize[1]
    size_y = imsize[0]
    # Based on the data type, we set the pixel_data_type variable.
    # The documentation says:
    #The possible values for the pixel data type are:
    # 8= unsigned integer, 1 byte (unsigned char in C).
    #16 = signed integer, 2 bytes (short in C).ß
    # -16 = unsigned integer, 2 bytes (unsigned 32 = signed integer, 4 bytes (int in C).
    # -32 = real, 4 bytes (float in C).
    # 64 = signed integer, 8 bytes (long in C).
    # -64 = real, 8 bytes (double in C).
    dtype = SHM.get_data().dtype
    if dtype == np.uint8:
        pixel_data_type = 8
    elif dtype == np.int16:
        pixel_data_type = 16
    elif dtype == np.uint16:    
        pixel_data_type = -16
    elif dtype == np.int32:
        pixel_data_type = 32
    elif dtype == np.float32:
        pixel_data_type = -32
    elif dtype == np.float64:
        pixel_data_type = -64
    else:
        raise ValueError("Data type not supported.")
    
# Create a 64 byte header
header = np.zeros(64, dtype=np.uint8)
header[0:len(camera_name)] = np.frombuffer(camera_name.encode('utf-8'), dtype=np.uint8)
header[32:34] = np.frombuffer(np.array(display_type, dtype=np.uint16), dtype=np.uint8)
header[34:36] = np.frombuffer(np.array(size_x, dtype=np.int16), dtype=np.uint8)
header[36:38] = np.frombuffer(np.array(size_y, dtype=np.int16), dtype=np.uint8)
header[38] = np.frombuffer(np.array(pixel_data_type, dtype=np.uint8), dtype=np.uint8)
header[39:64] = 0 # Filler

# Create a context for the ZeroMQ socket
context = zmq.Context()

# Create a PUSH socket to send data
socket = context.socket(zmq.PUSH)

# Connect to the RTD server on wag2
socket.connect("tcp://wag2:7000")

exiting=False
# Set the signal handler to handle SIGINT (Ctrl+C) gracefully
def signal_handler(sig, frame):
    global exiting
    print("Exiting gracefully...")
    socket.close()
    context.destroy()
    exiting = True

signal.signal(signal.SIGINT, signal_handler)
# Set the socket to non-blocking mode to avoid blocking on send
socket.setsockopt(zmq.LINGER, 0)

# Execute as an infinite loop
while not exiting:
    if simulate_image:
        #Create random data, with a square in the middle.
        data = np.random.randint(-100, 100, size=(size_y, size_x), dtype=np.int16)
        data[40:60, 40:60] += 1000
    else:
        # Get the data from the shared memory object
        if len(imsize)==3:
            data = SHM.get_latest_data_slice()
        else:
            data = SHM.get_data()
    
    # Create a message with the header and the data. After the flatting 
    # the data, we need to copy the memory to the right size.
    #message = np.concatenate((header, data.flatten()))
    
    # Send the message to the RTD server
    retval = socket.send(header.tobytes() + data.flatten().tobytes(), zmq.NOBLOCK)
    if retval == -1:
        print("Failed to send message, socket might be closed.")
    
    # Sleep for a bit to avoid flooding the server
    time.sleep(.1)

# Note: This code is a simplified example and does not include error handling or cleanup.
# In a real application, you should handle exceptions and ensure that the socket and context are properly closed.
# Also, this code assumes that the RTD server is running and listening on the specified address.
