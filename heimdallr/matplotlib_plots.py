"""
Various methods of drawing scrolling plots.
"""
import ZMQ_control_client as Z
import matplotlib.pyplot as plt
import numpy as np

plt.ion()  # interactive mode on


status = Z.send('status')
v2_K1 = np.zeros((100, 6))  # create an array of zeros
pd_tel = np.zeros((100, 4))  # create an array of zeros
gd_tel = np.zeros((100, 4))  # create an array of zeros
dm = np.zeros((100, 4))  
offload = np.zeros((100, 4))

def updatev2():
    v2_K1[:-1] = v2_K1[1:]  
    v2_K1[-1] = status['v2_K1']
    plt.figure(1)
    plt.clf()  # clear the current figure
    plt.plot(v2_K1)
    plt.xlabel('samples')
    plt.ylabel('V^2')

def updatepd():
    pd_tel[:-1] = pd_tel[1:]  
    pd_tel[-1] = status['pd_tel']
    plt.figure(2)
    plt.clf()  # clear the current figure
    plt.plot(pd_tel)
    plt.xlabel('samples')
    plt.ylabel('Phase Delay (wavelengths)')

def updategd():
    gd_tel[:-1] = gd_tel[1:]  
    gd_tel[-1] = status['gd_tel']
    plt.figure(5)
    plt.clf()  # clear the current figure
    plt.plot(gd_tel)
    plt.xlabel('samples')
    plt.ylabel('Group Delay (wavelengths)')
    
def updatepiston():
    dm[:-1] = dm[1:]  
    dm[-1] = status['dm_piston']
    plt.figure(3)
    plt.clf()  # clear the current figure
    plt.plot(dm)
    plt.xlabel('samples')
    plt.ylabel('Mirror Piston (fractional stoke)')
    
def updateoffload():
    offload[:-1] = offload[1:]  
    offload[-1] = status['dl_offload']
    plt.figure(4)
    plt.clf()  # clear the current figure
    plt.plot(offload)
    plt.xlabel('samples')
    plt.ylabel('Offloaded Piston (microns)')


# update all plots
def update():
    updatev2()
    updatepd()
    updategd()
    updatepiston()
    updateoffload()


if __name__ == '__main__':
    while(True):
        # get the status
        status = Z.send('status')
        # update the plots
        update()
        # sleep for a while to allow the plot to update
        plt.pause(.1)
