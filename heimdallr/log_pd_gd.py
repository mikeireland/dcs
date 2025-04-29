"""
Rapidly log time-tagged data from the DM as well as fringe-based delays.
"""
import ZMQ_control_client as Z
import numpy as np
import time

if __name__ == '__main__':
	with open('log.txt', 'a') as f:
		f.write('time, piston1, piston2, piston3, piston4, pd1, pd2, pd3, pd4, gd1, gd2, gd3, gd4\n')
		while(True):
			# get the status
			status = Z.send('status')
			# update the plots
			piston = status['dm_piston']
			pd = status['pd_tel']
			v2 = status['v2_K1']
			gd = status['gd_tel']
			f.write(f"{time.time():.4f}, {piston[0]:.3f}, {piston[1]:.3f}, {piston[2]:.3f}, {piston[3]:.3f}, {pd[0]:.3f}, {pd[1]:.3f}, {pd[2]:.3f}, {pd[3]:.3f}, {gd[0]:.3f}, {gd[1]:.3f}, {gd[2]:.3f}, {gd[3]:.3f}\n")
			# sleep for a very short time.
			time.sleep(0.0015)
			
