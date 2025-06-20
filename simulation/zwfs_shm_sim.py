from xaosim.shmlib import shm
import numpy as np
import matplotlib.pyplot as plt
import glob
import os 
class SimDM:
    def __init__(self, dmid=1):
        self.dmid = dmid
        self.shms = []
        self.shm0 = None
        self.setup_shm()

    def setup_shm(self):
        shmfs = sorted(glob.glob(f"/dev/shm/dm{self.dmid}disp*.im.shm"))
        shmf0 = f"/dev/shm/dm{self.dmid}.im.shm"
        self.nch = len(shmfs)

        self.shms = [shm(f, nosem=False) for f in shmfs]

        if self.nch > 0 and os.path.exists(shmf0):
            self.shm0 = shm(shmf0, nosem=False)
        else:
            print(f"[WARNING] SHM for dm{self.dmid} not found. Is the sim server running?")

dm = SimDM(dmid=1)
# check data ch 1 
dm.shms[1].get_data()

# check data on combined channel
dm.shm0.get_data()

# set data on ch 1
dm.shms[1].set_data( np.eye(12, dtype=np.uint16) )

# post semaphore to update it on combined ch 
dm.shm0.post_sems(1)

# check combined channel is updated
dm.shm0.get_data()



from xaosim.shmlib import shm
import numpy as np
import time

# Replace with actual SHM key names
dm_shm_name = "dm1"
cam_shm_name = "cred1"

# Open DM SHM (read-only)
dm_shm = shm(dm_shm_name)  # remove readonly=True
dm_data = dm_shm.get_data()
print("DM data shape:", dm_data.shape)

# Create camera SHM (image with same shape, dtype float32)
height, width = dm_data.shape[-2:]  # last 2 dims if 3D
cam_shape = (height, width)
cam_dtype = "float32"
cam = shm()
cam.create(cam_shm_name, cam_shape, dtype=cam_dtype, atype="float")

# Main loop: simulate writing frames
while True:
    frame = np.zeros(cam_shape, dtype=np.float32)
    cam.set_data(frame)
    cam.post_sems()
    print("Wrote frame to camera SHM")
    time.sleep(0.1)
