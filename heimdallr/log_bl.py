"""
Rapidly log time-tagged data from the DM as well as fringe-based delays.
"""
import ZMQ_control_client as Z
import numpy as np
import time

if __name__ == '__main__':
	with open('log.txt', 'a') as f:
		f.write('time, piston1, piston2, piston3, piston4, pd1, pd2, pd3, pd4,bld1, bld2, bld3, bld4, bld5, bld6, pd_av1, pd_av2, pd_av3, pd_av4, pd_av5, pd_av6, pd_avf1, pd_avf2, pd_avf3, pd_avf4, pd_avf5, pd_avf6\n')
		while(True):
			# get the status
			status = Z.send('status')
			# update the plots
			piston = status['dm_piston']
			pd = status['pd_tel']
			bld = status['pd_bl']
			pd_av = status['pd_av']
			pd_avf = status['pd_av_filtered']
			f.write(f"{time.time():.4f}, {piston[0]:.3f}, {piston[1]:.3f}, {piston[2]:.3f}, {piston[3]:.3f}, {pd[0]:.3f}, \
{pd[1]:.3f}, {pd[2]:.3f}, {pd[3]:.3f}, {bld[0]:.3f}, {bld[1]:.3f}, {bld[2]:.3f}, {bld[3]:.3f}, {bld[4]:.3f}, {bld[5]:.3f}, \
{pd_av[0]:.3f}, {pd_av[1]:.3f}, {pd_av[2]:.3f}, {pd_av[3]:.3f}, {pd_av[4]:.3f}, {pd_av[5]:.3f}, \
{pd_avf[0]:.3f}, {pd_avf[1]:.3f}, {pd_avf[2]:.3f}, {pd_avf[3]:.3f}, {pd_avf[4]:.3f}, {pd_avf[5]:.3f}\n")
			# sleep for a very short time.
			time.sleep(0.0015)
			
