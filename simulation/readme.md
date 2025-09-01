Simulation mode of Asgard/Baldr RTC. 

A python package (baldrApp) is used to simulate Baldr - a Zernike Wavefront Sensor.

This simulator also sets up and simulates the CRED1 Camera server and Multi-Device Server (MDS) in a semi-consistent way to the real system (as of Aug 2025) with ZMQ commincation to keep track of system states and writing frames to shared memory structures. It then plugs directly into the Baldr RTC to test algorithms and performance.  This has been developed in a way that is independent of any hardware dependancies.

tested on Ubuntu 20.04 

To set up:
in Simulation folder set up a virtual environemnt 

> cd dcs/simulation
> python3.12 -m venv venv

Or setup the environment with whatever python you want. But note this was tested with Python 3.12.4

Then activate virtual environment 

> source venv/bin/activate

Then install requirements 

> pip install -r requirements.txt

then we have to build the MDM server and shm_creator_sim C++ programs. 

> sudo apt install nlohmann-json3-dev

> cd dcs/simulation

then build directly using libImageStreamIO source:

> g++ -std=c++17 shm_creator_sim.cpp ../libImageStreamIO/ImageStreamIO.c \
  -I../libImageStreamIO -pthread -o shm_creator_sim

> g++ -O2 -std=c++17 -o sim_mdm_server sim_mdm_server.cpp ../libImageStreamIO/ImageStreamIO.c \
  -I../libImageStreamIO -pthread


you may want to check depedent file exists in /dcs/asgard-cred1-server/cred1_split.json

To start the simulation servers: 

> ./heimbal_simulation_servers.sh start

To see the status of the servers and simulator 

> ./heimbal_simulation_servers.sh status

To stop the simulation servers: 

> ./heimbal_simulation_servers.sh stop

To see camera images updating once servers are running:

> shmview /dev/shm/cred1.im.shm

To make the baldr program (If it hasn't already been done)

> cd dcs/baldr
> make clean
> make

To start the Baldr RTC (this requires configuration files in usr/etc/local/baldr/..): 
> cd dcs/baldr
> ./baldr --beam 1 --mask J3 --mode bright --socket tcp://*:6662

A GUI to see the baldr telemetry and command the RTC while RTC is running: 
> cd dcs/pyeng_baldr
> python baldr_live_gui.py

Any issues contact Ben : benjamin.courtney-barrer@anu.edu.au



