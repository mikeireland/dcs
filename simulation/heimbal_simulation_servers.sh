#!/bin/bash
# NOTE: FOR SOME REASON THIS SCRIPT DOESN'T WORK - BUT IF YOU 
# RUN EACH PROGRAM INDIVIDUALLY IT SHOULD WORK!
# Trap Ctrl+C (SIGINT) to kill all background jobs
trap 'echo "Stopping all processes..."; kill 0' SIGINT

# Start shm_creator_sim
./shm_creator_sim &
PID1=$!
echo "[INFO] Started shm_creator_sim (PID $PID1)"
sleep 2

# Start sim_mdm_server
./sim_mdm_server &
PID2=$!
echo "[INFO] Started sim_mdm_server (PID $PID2)"
sleep 2

# Start fake ZMQ camera server
python3 fake_asgard_ZMQ_CRED1_server.py &
PID3=$!
echo "[INFO] Started fake ZMQ server (PID $PID3)"
sleep 2

# Activate virtual environment
#source venv/bin/activate
#echo "[INFO] Activated virtual environment"

# Start the interactive simulation
# python3 baldr_sim.py

# Wait for all background jobs (until Ctrl+C)
wait $PID1 $PID2 $PID3
