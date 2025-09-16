import os
import numpy as np 
import matplotlib.pyplot as plt 
import pyzelda.utils.zernike as zernike
from mpl_toolkits.axes_grid1 import make_axes_locatable
import datetime
import matplotlib.patheffects as pe
import time 
from matplotlib.widgets import Slider
import matplotlib.animation as animation
import math
from astropy.io import fits 
import pandas as pd
from scipy.interpolate import interp1d
import subprocess
from scipy.integrate import quad
from scipy import ndimage
import scipy.ndimage as ndimage
from scipy.spatial import distance
from scipy import signal
from scipy.ndimage import binary_erosion
from scipy.interpolate import RegularGridInterpolator
from scipy.ndimage import distance_transform_edt
from scipy.optimize import curve_fit
from scipy.optimize import leastsq
from scipy.ndimage import gaussian_filter,  median_filter
from scipy.ndimage import label, find_objects
import glob
import re
from pathlib import Path
from fractions import Fraction

import itertools
import corner

#####
# ANYTHING THAT RELIES ON THESE SDK'S NEEDS
# TO BE UPDATED WITH  SHM
#####
# import sys 
# sys.path.insert(1, '/opt/FirstLightImaging/FliSdk/Python/demo/')
# sys.path.insert(1,'/opt/Boston Micromachines/lib/Python3/site-packages/')
# import FliSdk_V2 
# import FliCredThree
# import FliCredTwo
# import FliCredOne

# import bmc
# ============== UTILITY FUNCTIONS



def convert_to_serializable(obj):
    """
    Recursively converts NumPy arrays and other non-serializable objects to serializable forms.
    Also converts dictionary keys to standard types (str, int, float).
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()  # Convert NumPy arrays to lists
    elif isinstance(obj, np.integer):
        return int(obj)  # Convert NumPy integers to Python int
    elif isinstance(obj, np.floating):
        return float(obj)  # Convert NumPy floats to Python float
    elif isinstance(obj, dict):
        return {str(key): convert_to_serializable(value) for key, value in obj.items()}  # Ensure keys are strings
    elif isinstance(obj, list):
        return [convert_to_serializable(item) for item in obj]
    else:
        return obj  # Base case: return the object itself if it doesn't need conversion



def recursive_update(orig, new):
    """
    Recursively update dictionary 'orig' with 'new' without overwriting sub-dictionaries.
    """
    for key, value in new.items():
        if (key in orig and isinstance(orig[key], dict) 
            and isinstance(value, dict)):
            recursive_update(orig[key], value)
        else:
            orig[key] = value
    return orig

    
# Function to run script
def run_script(command):
    """
    Run an external python script using subprocess.
    """
    try:

        # Ensure stdout and stderr are properly closed
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as process:
            
            stdout, stderr = process.communicate()
            if process.returncode != 0:
                print(f"Script failed: {stderr}")
                return False
        return True  # Script succeeded
    except Exception as e:
        print(f"Error running script: {e}")
        return False


def run_script_with_output(command):
    """
    Run an external script using subprocess and capture its output 
    """
    try:

        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as process:
            
            output = []
            for line in process.stdout:
                print(line.strip())  # Stream output in real-time to UI
                output.append(line.strip())

            stderr_output = process.stderr.read().strip()
            print(f'process return code {process.returncode}')
            ### This always fails even when script runs fine.. even when using sys.exit(0) I dont understand
            #if process.returncode != 0:
            #    st.error(f"Script failed: {stderr_output}")
            #    return False, output
        return True, output  # Script succeeded

    except Exception as e:
        print(f"Error running script: {e}")
        return False, []




def find_calibration_files(mode, gain, target_fps, base_dir="MASTER_DARK", time_diff_thresh=datetime.timedelta(1), fps_diff_thresh=100):
    """
    For finding claibration files in DCS (on mimir:/home/asg/Progs/repos/dcs/calibration_frames/products/)

    Search through the calibration directories from base_dir (default= MASTER_DARK) to findfiles matching a given mode and gain,
    and further filter by time difference (from current time) and fps difference constraints.
    
    Parameters:
      mode (str): The calibration mode to match.
      gain (int or str): The gain value to match.
      target_fps (float): The target fps value.
      time_diff_thresh (datetime.timedelta): (default 1 day) Maximum allowed age (difference between now and file timestamp).
      fps_diff_thresh (float): Maximum allowed difference in fps from target_fps, .
      
    Returns:
      list of Path: A list of file paths that pass the filters.
    """
    
    try:
        gain = int(gain)
        target_fps = float(target_fps)
    except:
        raise UserWarning("input type for gain or fps is wrong.")
    
    # Define the base directory pattern.
    base_pattern = f"/home/asg/Progs/repos/dcs/calibration_frames/products/*/{base_dir}/"
    # Prepare the glob pattern: we'll insert the mode and gain in the filename. 
    # For example, if mode='A' and gain=5, we want files like:
    # master_darks_A_fps-<fps_value>_gain-5_<timestamp>.fits
    file_pattern = f"*_{mode}_fps-*_gain-{gain}_*.fits"
    
    # Combine the base path and the file pattern using glob:
    search_path = base_pattern + file_pattern
    file_list = glob.glob(search_path)
    
    # Prepare the regular expression to parse the filename.
    # Example filename:
    # master_darks_A_fps-30_gain-5_2021-05-22T13-37-02.fits
    regex = re.compile(
        r".*_(?P<mode>\w+)_fps-(?P<fps>\d+(?:\.\d+)?)_gain-(?P<gain>\d+)_"
        r"(?P<timestamp>\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2})\.fits"
    )
    
    now = datetime.datetime.now()
    accepted_files = []
    
    for file_path in file_list:
        fname = Path(file_path).name
        match = regex.fullmatch(fname)
        if not match:
            # Skip files that don't match the expected pattern
            continue

        # Extract the file parameters
        file_mode = match.group("mode")
        file_fps = float(match.group("fps"))
        file_gain = match.group("gain")  # as string, or convert to int if needed
        timestamp_str = match.group("timestamp")
        
        # Check that file_mode and file_gain match input parameters.
        # Note: You may need to adjust comparisons (e.g., case insensitive, numeric conversion) as needed.
        if file_mode != mode or str(file_gain) != str(gain):
            continue
        
        # Parse the timestamp. The expected format is: YYYY-MM-DDTHH-MM-SS
        try:
            file_timestamp = datetime.datetime.strptime(timestamp_str, "%Y-%m-%dT%H-%M-%S")
        except ValueError:
            continue  # skip if the timestamp can't be parsed
        
        # Check time difference: we want files that are not older than the threshold.
        time_diff = now - file_timestamp
        if time_diff > time_diff_thresh:
            continue
        
        # Check fps difference.
        if abs(file_fps - target_fps) > fps_diff_thresh:
            continue
        
        # If passed both filters, add to the list.
        accepted_files.append(Path(file_path))
    
    return accepted_files


def waffle_mode_2D(n=12):
    """Returns a 2D waffle mode (checkerboard pattern) of size n x n"""
    W = np.fromfunction(lambda i, j: (-1) ** (i + j), (n, n))
    return W

def construct_command_basis( basis='Zernike_pinned_edges', number_of_modes = 20, Nx_act_DM = 12, Nx_act_basis = 12, act_offset=(0,0), without_piston=True):
    """
    returns a change of basis matrix M2C to go from modes to DM commands, where columns are the DM command for a given modal basis. e.g. M2C @ [0,1,0,...] would return the DM command for tip on a Zernike basis. Modes are normalized on command space such that <M>=0, <M|M>=1. Therefore these should be added to a flat DM reference if being applied.    

    basis = string of basis to use
    number_of_modes = int, number of modes to create
    Nx_act_DM = int, number of actuators across DM diameter
    Nx_act_basis = int, number of actuators across the active basis diameter
    act_offset = tuple, (actuator row offset, actuator column offset) to offset the basis on DM (i.e. we can have a non-centered basis)
    IM_covariance = None or an interaction matrix from command to measurement space. This only needs to be provided if you want KL modes, for this the number of modes is infered by the shape of the IM matrix. 
     
    """

   
    # shorter notations
    #Nx_act = DM.num_actuators_width() # number of actuators across diameter of DM.
    #Nx_act_basis = actuators_across_diam
    c = act_offset
    # DM BMC-3.5 is 12x12 missing corners so 140 actuators , we note down corner indicies of flattened 12x12 array.
    corner_indices = [0, Nx_act_DM-1, Nx_act_DM * (Nx_act_DM-1), -1]

    bmcdm_basis_list = []
    # to deal with
    if basis == 'Zernike':
        if without_piston:
            number_of_modes += 1 # we add one more mode since we dont include piston 

        raw_basis = zernike.zernike_basis(nterms=number_of_modes, npix=Nx_act_basis )
        for i,B in enumerate(raw_basis):
            # normalize <B|B>=1, <B>=0 (so it is an offset from flat DM shape)
            Bnorm = np.sqrt( 1/np.nansum( B**2 ) ) * B
            # pad with zeros to fit DM square shape and shift pixels as required to center
            # we also shift the basis center with respect to DM if required
            if np.mod( Nx_act_basis, 2) == 0:
                pad_width = (Nx_act_DM - B.shape[0] )//2
                padded_B = shift( np.pad( Bnorm , pad_width , constant_values=(np.nan,)) , c[0], c[1])
            else:
                pad_width = (Nx_act_DM - B.shape[0] )//2 + 1
                padded_B = shift( np.pad( Bnorm , pad_width , constant_values=(np.nan,)) , c[0], c[1])[:-1,:-1]  # we take off end due to odd numebr

            flat_B = padded_B.reshape(-1) # flatten basis so we can put it in the accepted DM command format
            np.nan_to_num(flat_B,0 ) # convert nan -> 0
            flat_B[corner_indices] = np.nan # convert DM corners to nan (so lenght flat_B = 140 which corresponds to BMC-3.5 DM)

            # now append our basis function removing corners (nan values)
            bmcdm_basis_list.append( flat_B[np.isfinite(flat_B)] )

        # our mode 2 command matrix
        if without_piston:
            M2C = np.array( bmcdm_basis_list )[1:].T #remove piston mode
        else:
            M2C = np.array( bmcdm_basis_list ).T # take transpose to make columns the modes in command space.


    elif basis == 'Zernike_pinned_edges':
        """
        designed for BMC multi 3.5 DM, define zernike basis on 10x10 central grid and 
        interpolate outside of this grid, pinning the value of perimeter actuators to the
        inner perimeter value. 
        """
        nact_len = 12 # must be 12
        # alway construct with piston cause we use as a filter, we delete piston later if specified by user
        b0 = construct_command_basis( basis='Zernike', number_of_modes = number_of_modes, Nx_act_DM = nact_len, Nx_act_basis = nact_len, act_offset=(0,0), without_piston=False)

        # put values outside pupil to nan 
        btmp = np.array( [get_DM_command_in_2D( bb ) for bb in b0.T])

        # interpolate
        nan_mask = btmp[0] #util.get_DM_command_in_2D( b0.T[0] != 0 )
        nan_mask[nan_mask==0] = np.nan

        #nan_mask = np.isnan(nan_mask)
        nearest_index = distance_transform_edt(np.isnan(nan_mask), return_distances=False, return_indices=True)

        # Use the indices to replace NaNs with the nearest non-NaN values

        with_corners = np.array( [ (nan_mask * bb)[tuple(nearest_index)] for bb in btmp[:]] ).T
        #filled_data = get_DM_command_in_2D( new_flat )[tuple(nearest_index)]


        # Define the indices of the corners to be removed
        corners = [(0, 0), (0, nact_len-1), (nact_len-1, 0), (nact_len-1, nact_len-1)]
        # Convert 2D corner indices to 1D
        corner_indices = [i * 12 + j for i, j in corners]

        # Flatten the array
        bmcdm_basis_list = []
        for w in with_corners.T:
            flattened_array = w.flatten()
            filtered_array = np.delete(flattened_array, corner_indices)

            bmcdm_basis_list.append( filtered_array )

        # piston was taken care of in construction of original zernike basis  b0 = construct_command_basis(
        if without_piston:
            control_basis = [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in bmcdm_basis_list[1:]]
        else:
            control_basis = [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in bmcdm_basis_list[:]]
        M2C = np.array( control_basis ).T

             
    elif basis == 'fourier':
        # NOT TESTED YET ON REAL DM!! 
        if without_piston:
            number_of_modes += 1 # we add one more mode since we dont include piston 

        # NOTE BECAUSE WE HAVE N,M DIMENSIONS WE NEED TO ROUND UP TO SQUARE NUMBER THE MIGHT NOT = EXACTLY number_of_modes
        n = round( number_of_modes**0.5 ) + 1 # number of modes = (n-1)*(m-1) , n=m => (n-1)**2 
        control_basis_dict  = develop_Fourier_basis( n, n ,P = 2 * Nx_act_DM, Nx = Nx_act_DM, Ny = Nx_act_DM )
        
        # create raw basis as ordered list from our dictionary
        raw_basis = []
        for i in range( n-1 ):
            for j in np.arange( i , n-1 ):
                if i==j:
                    raw_basis.append( control_basis_dict[i,i] )
                else:
                    raw_basis.append( control_basis_dict[i,j] ) # get either side of diagonal 
                    raw_basis.append( control_basis_dict[j,i] )
                    
        
        bmcdm_basis_list = []
        for i,B in enumerate(raw_basis):
            B = B.reshape(-1)
            B[corner_indices] = np.nan
            bmcdm_basis_list.append( B[np.isfinite(B)] )
        # flatten & normalize each basis cmd 
        # <M|M> = 1
        if without_piston:
            control_basis = [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in bmcdm_basis_list[1:]] #remove piston mode
        else:
            control_basis = [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in bmcdm_basis_list]# take transpose to make columns the modes in command space.
        M2C = np.array( control_basis ).T 

    elif basis == 'fourier_pinned_edges':
        """
        designed for BMC multi 3.5 DM, define zernike basis on 10x10 central grid and 
        interpolate outside of this grid, pinning the value of perimeter actuators to the
        inner perimeter value. 
        """
        n = round( number_of_modes**0.5 ) + 1 # number of modes = (n-1)*(m-1) , n=m => (n-1)**2 
        actlen_tmp = 10 # must be 10 for this option! we then calculate perimeter values here! 
        control_basis_dict  = develop_Fourier_basis( n, n ,P = 2 * actlen_tmp, Nx = actlen_tmp, Ny = actlen_tmp )
                
        # create raw basis as ordered list from our dictionary
        raw_basis = []
        for i in range( n-1 ):
            for j in np.arange( i , n-1 ):
                if i==j:
                    raw_basis.append( control_basis_dict[i,i] )
                else:
                    raw_basis.append( control_basis_dict[i,j] ) # get either side of diagonal 
                    raw_basis.append( control_basis_dict[j,i] )
                    
        # pin_outer_actuators_to_inner requires 10x10 input!!! creates 12x12 with missing corner pinning outer actuators 
        bmcdm_basis_list = np.array( [pin_outer_actuators_to_inner_diameter(bb.reshape(-1)) for bb in np.array( raw_basis)] )

        # <M|M> = 1
        if without_piston:
            control_basis = [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in bmcdm_basis_list[1:]] #remove piston mode
        else:
            control_basis = [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in bmcdm_basis_list]# take transpose to make columns the modes in command space.
        
        M2C = np.array( control_basis ).T 


    elif basis == 'Zonal': 
        #hardcoded for BMC multi3.5 DM (140 actuators)
        M2C = np.eye( 140 ) # we just consider this over all actuators (so defaults to 140 modes) 
        # we filter zonal basis in the eigenvectors of the control matrix. 
    
    elif basis == 'Zonal_pinned_edges':
        # pin edges of actuator
        b = np.eye(100) #
        bmcdm_basis_list = np.array( [pin_outer_actuators_to_inner_diameter(bb) for bb in b.T] )
        # <M|M> = 1
        control_basis = np.array( [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in bmcdm_basis_list] )

        M2C = np.array( control_basis ).T

    elif basis == 'KL':         
        if without_piston:
            number_of_modes += 1 # we add one more mode since we dont include piston 

        raw_basis = zernike.zernike_basis(nterms=number_of_modes, npix=Nx_act_basis )
        b0 = np.array( [np.nan_to_num(b) for b in raw_basis] )
        cov0 = np.cov( b0.reshape(len(b0),-1) )
        U , S, UT = np.linalg.svd( cov0 )
        KL_raw_basis = ( b0.T @ U ).T # KL modes that diagonalize Zernike covariance matrix 
        for i,B in enumerate(KL_raw_basis):
            # normalize <B|B>=1, <B>=0 (so it is an offset from flat DM shape)
            Bnorm = np.sqrt( 1/np.nansum( B**2 ) ) * B
            # pad with zeros to fit DM square shape and shift pixels as required to center
            # we also shift the basis center with respect to DM if required
            if np.mod( Nx_act_basis, 2) == 0:
                pad_width = (Nx_act_DM - B.shape[0] )//2
                padded_B = shift( np.pad( Bnorm , pad_width , constant_values=(np.nan,)) , c[0], c[1])
            else:
                pad_width = (Nx_act_DM - B.shape[0] )//2 + 1
                padded_B = shift( np.pad( Bnorm , pad_width , constant_values=(np.nan,)) , c[0], c[1])[:-1,:-1]  # we take off end due to odd numebr

            flat_B = padded_B.reshape(-1) # flatten basis so we can put it in the accepted DM command format
            np.nan_to_num(flat_B,0 ) # convert nan -> 0
            flat_B[corner_indices] = np.nan # convert DM corners to nan (so lenght flat_B = 140 which corresponds to BMC-3.5 DM)

            # now append our basis function removing corners (nan values)
            bmcdm_basis_list.append( flat_B[np.isfinite(flat_B)] )

        # our mode 2 command matrix
        if without_piston:
            M2C = np.array( bmcdm_basis_list )[1:].T #remove piston mode
        else:
            M2C = np.array( bmcdm_basis_list ).T # take transpose to make columns the modes in command space.

    else:
        raise TypeError( ' input basis name invalid. Try: Zonal, Zonal_pinned_edges, Zernike, Zernike_pinned_edges, fourier, fourier_pinned_edges, KL etc ')
    
    
    return(M2C)


def get_tip_tilt_vectors( dm_model='bmc_multi3.5',nact_len=12):
    tip = np.array([[n for n in np.linspace(-1,1,nact_len)] for _ in range(nact_len)])
    tilt = tip.T
    if dm_model == 'bmc_multi3.5':
        # Define the indices of the corners to be removed
        corners = [(0, 0), (0, nact_len-1), (nact_len-1, 0), (nact_len-1, nact_len-1)]
        # Convert 2D corner indices to 1D
        corner_indices = [i * 12 + j for i, j in corners]

        # remove corners
        tip_tilt_list = []
        for i,B in enumerate([tip,tilt]):
            B = B.reshape(-1)
            B[corner_indices] = np.nan
            tip_tilt_list.append( B[np.isfinite(B)] )
        
        tip_tilt = np.array( [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in tip_tilt_list] ).T

    else:
        tip_tilt = np.array( [np.sqrt( 1/np.nansum( cb**2 ) ) * cb.reshape(-1) for cb in [tip.reshape(-1),tilt.reshape(-1)]] ).T

    return( tip_tilt ) 


def fourier_vector(n, m, P = 2*12, Nx = 12, Ny = 12):
    """
    OR we can do it with complex exponetial, in-quadrature is real part, out of quadrature is imaginary 
    Normalized <Bx|Bx>=1 , <By|By>=1

    Parameters
    ----------
    n : TYPE
        DESCRIPTION.
    m : TYPE
        DESCRIPTION.
    P : TYPE, optional
        DESCRIPTION. The default is 2*12.
    Nx : TYPE, optional
        DESCRIPTION. The default is 12.
    Ny : TYPE, optional
        DESCRIPTION. The default is 12.

    Returns
    -------
    None.

    """
    x = np.linspace(-6,6,Nx)
    y = np.linspace(-6,6,Ny)
    X,Y = np.meshgrid(x,y)
    
    
    Bx = np.exp( 1J * 2 * np.pi * n/P * X )
    if np.sum( abs(Bx) ):
        Bx *= 1/np.sum( abs(Bx)**2 )**0.5

    By = np.exp( 1J * 2 * np.pi * m/P * Y )
    if np.sum( abs(By) ):
        By *= 1/np.sum( abs(By)**2 )**0.5
    
    return( Bx, By )

def develop_Fourier_basis( n,m ,P = 2*12, Nx = 12, Ny = 12):
    """
    

    Parameters
    ----------
    n : TYPE int
        DESCRIPTION. what order in x (column) dimension do we create Fourier basis for?
    m : TYPE int
        DESCRIPTION. what order in y (row) dimension do we create Fourier basis for?

    Returns
    -------
    basis_dict - a dictionary indexed by mode order tuple (n,m) with corresponding 2D Fourier basis
    
    
    # what is logical indexing? 
    basis naturally forms 2 NxN squares, one square corresponding to odd components (sin) in x,y other to even (cos)

    for each axis dimension cnt, with even numbers corresponding to even functions (cos), odd numbers to odd functions (sin)
    therefore to recover cos or sin order we simply divide by 2 and round (//2)

    we do not count piston  
    e.g. indexing for x dimension:
    0 = np.real(F_basis_x[0] )  
    1 = np.imag(F_basis_x[0] )  
    2 = np.iamg(F_basis_x[1] ) 
    3 = np.real(F_basis_x[1] ) 

    therefore for example index (3,2)
    B_(3,2) = np.real(F_basis_x[1] ) * np.imag(F_basis_y[1] )
    first index corresponds to variation across columns (x), 
    second index corresponds to variation across rows (y)

    """
    basis_dict = {}

    for x_idx in range(0,n):
        for y_idx in range(0,m):            
            #
            x_order = x_idx//2
            y_order = y_idx//2
            
            if not ((x_idx==0) | (y_idx==0)): # otherwise we get lots of pistons 
                Bx, By = fourier_vector(x_order, y_order, P , Nx , Ny )
                    
                if not np.mod(x_idx,2): #odd number take imaginary (odd) part
                    Bx_q = np.imag( Bx )
    
                else: # even number take real (even) part
                    Bx_q = np.real( Bx )
    
                    
                if not np.mod(y_idx,2): #odd number take imaginary (odd) part
                    By_q = np.imag( By )
                    
                else: # even number take real (even) part
                    By_q = np.real( By )
            
                #if x_idx > 1:
                mode_tmp = Bx_q * By_q - np.mean(Bx_q * By_q)
                if np.sum( mode_tmp**2):
                    mode_tmp *= 1/np.sum( mode_tmp**2)**0.5 #normalized <M|M>=1
                basis_dict[(x_idx-1,y_idx-1)] =  mode_tmp


    return(basis_dict)



def pin_outer_actuators_to_inner_diameter(inner_command):
    """
    input a basis defined on 10x10 grid and this will convert it to a
    12x12 grid without corners (BMC multi3.5 DM geometry) with the outer
    perimeter actuators pinned to the inner perimeter value
    """
    if len(inner_command) != 100:
        raise ValueError("Input command must be of length 100")

    inner_command = np.array(inner_command).reshape(10, 10)
    
    # Initialize a 12x12 grid with zeros
    command_140 = np.zeros((12, 12))

    # Map the inner 10x10 command to the corresponding position in the 12x12 grid
    command_140[1:11, 1:11] = inner_command

    # Set the perimeter actuators equal to the inner adjacent values
    # Top and bottom rows
    command_140[0, 1:11] = command_140[1, 1:11]
    command_140[11, 1:11] = command_140[10, 1:11]

    # Left and right columns
    command_140[1:11, 0] = command_140[1:11, 1]
    command_140[1:11, 11] = command_140[1:11, 10]

    # Corners (set these to zero since they are missing actuators)
    corners = [(0, 0), (0, 11), (11, 0), (11, 11)]
    for corner in corners:
        command_140[corner] = 0

    # Flatten the 12x12 grid to get the final 140-length command
    command_140_flat = command_140.flatten()

    # Remove the corner actuators (i.e., elements 0, 11, 132, 143)
    indices_to_remove = [0, 11, 132, 143]
    command_140_flat = np.delete(command_140_flat, indices_to_remove)

    return command_140_flat.tolist()

def pin_to_nearest_registered_with_missing_corners(dm_shape, missing_corners, registered_indices):
    """
    Pins non-registered actuators to the closest registered actuator, excluding missing corners.

    Parameters:
    - dm_shape: Tuple (rows, cols) representing the DM grid, e.g., (12, 12).
    - missing_corners: List of indices (in the flattened array) of missing corners.
    - registered_indices: 1D array of indices corresponding to actuators with registered values.

    Returns:
    - basis: 2D array (dm_shape[0] * dm_shape[1] - len(missing_corners), len(registered_indices))
             where each non-registered actuator is pinned to its closest registered actuator.
    """
    # Create the full DM grid with flattened indices
    flattened_size = dm_shape[0] * dm_shape[1]
    
    # Generate 2D coordinates for each point on the grid
    grid_coords = np.array(np.unravel_index(np.arange(flattened_size), dm_shape)).T
    
    # Remove missing corners from the grid and flatten the remaining actuators
    valid_indices = np.setdiff1d(np.arange(flattened_size), missing_corners)
    valid_coords = grid_coords[valid_indices]

    # Extract coordinates of the registered actuators
    registered_coords = grid_coords[registered_indices]
    
    # Initialize the basis matrix for valid actuators
    basis = np.zeros((len(valid_indices), len(registered_indices)))
    
    # For each valid actuator, find the closest registered actuator
    for idx, valid_idx in enumerate(valid_indices):
        if valid_idx in registered_indices:
            # If the actuator is registered, set its basis vector to be identity
            basis[idx, registered_indices == valid_idx] = 1.0
        else:
            # If the actuator is not registered, pin it to the nearest registered actuator
            distances = distance.cdist([grid_coords[valid_idx]], registered_coords)
            nearest_idx = np.argmin(distances)
            # Pin to the nearest registered actuator
            basis[idx, nearest_idx] = 1.0
    
    #<m|m>=1
    basis_norm = np.array( [b/np.sum(b**2)**0.5 for b in basis.T] ).T
    
    
    return basis_norm

def get_theoretical_reference_pupils( wavelength = 1.65e-6 ,F_number = 21.2, mask_diam = 1.2, coldstop_diam=None, eta=0, diameter_in_angular_units = True, get_individual_terms=False, phaseshift = np.pi/2 , padding_factor = 4, debug= True, analytic_solution = True ) :
    """
    get theoretical reference pupil intensities of ZWFS with / without phasemask 
    

    Parameters
    ----------
    wavelength : TYPE, optional
        DESCRIPTION. input wavelength The default is 1.65e-6.
    F_number : TYPE, optional
        DESCRIPTION. The default is 21.2.
    mask_diam : phase dot diameter. TYPE, optional
            if diameter_in_angular_units=True than this has diffraction limit units ( 1.22 * f * lambda/D )
            if  diameter_in_angular_units=False than this has physical units (m) determined by F_number and wavelength
        DESCRIPTION. The default is 1.2.
    coldstop_diam : diameter in lambda / D of focal plane coldstop
    eta : ratio of secondary obstruction radius (r_2/r_1), where r2 is secondary, r1 is primary. 0 meams no secondary obstruction
    diameter_in_angular_units : TYPE, optional
        DESCRIPTION. The default is True.
    get_individual_terms : Type optional
        DESCRIPTION : if false (default) with jsut return intensity, otherwise return P^2, abs(M)^2 , phi + mu
    phaseshift : TYPE, optional
        DESCRIPTION. phase phase shift imparted on input field (radians). The default is np.pi/2.
    padding_factor : pad to change the resolution in image plane. TYPE, optional
        DESCRIPTION. The default is 4.
    debug : TYPE, optional
        DESCRIPTION. Do we want to plot some things? The default is True.
    analytic_solution: TYPE, optional
        DESCRIPTION. use analytic formula or calculate numerically? The default is True.
    Returns
    -------
    Ic, reference pupil intensity with phasemask in 
    P, reference pupil intensity with phasemask out 

    """
    pupil_radius = 1  # Pupil radius in meters

    # Define the grid in the pupil plane
    N = 2**9+1  # for parity (to not introduce tilt) works better ODD!  # Number of grid points (assumed to be square)
    L_pupil = 2 * pupil_radius  # Pupil plane size (physical dimension)
    dx_pupil = L_pupil / N  # Sampling interval in the pupil plane
    x_pupil = np.linspace(-L_pupil/2, L_pupil/2, N)   # Pupil plane coordinates
    y_pupil = np.linspace(-L_pupil/2, L_pupil/2, N) 
    X_pupil, Y_pupil = np.meshgrid(x_pupil, y_pupil)
    
    


    # Define a circular pupil function
    pupil = (np.sqrt(X_pupil**2 + Y_pupil**2) > eta*pupil_radius) & (np.sqrt(X_pupil**2 + Y_pupil**2) <= pupil_radius)

    # Zero padding to increase resolution
    # Increase the array size by padding (e.g., 4x original size)
    N_padded = N * padding_factor
    if (N % 2) != (N_padded % 2):  
        N_padded += 1  # Adjust to maintain parity
        
    pupil_padded = np.zeros((N_padded, N_padded))
    #start_idx = (N_padded - N) // 2
    #pupil_padded[start_idx:start_idx+N, start_idx:start_idx+N] = pupil

    start_idx_x = (N_padded - N) // 2
    start_idx_y = (N_padded - N) // 2  # Explicitly ensure symmetry

    pupil_padded[start_idx_y:start_idx_y+N, start_idx_x:start_idx_x+N] = pupil

    # Perform the Fourier transform on the padded array (normalizing for the FFT)
    pupil_ft = np.fft.fftshift(np.fft.fft2(np.fft.ifftshift(pupil_padded)))
    
    # Compute the Airy disk scaling factor (1.22 * λ * F)
    airy_scale = 1.22 * wavelength * F_number

    # Image plane sampling interval (adjusted for padding)
    L_image = wavelength * F_number / dx_pupil  # Total size in the image plane
    dx_image_padded = L_image / N_padded  # Sampling interval in the image plane with padding
    

    if diameter_in_angular_units:
        x_image_padded = np.linspace(-L_image/2, L_image/2, N_padded) / airy_scale  # Image plane coordinates in Airy units
        y_image_padded = np.linspace(-L_image/2, L_image/2, N_padded) / airy_scale
    else:
        x_image_padded = np.linspace(-L_image/2, L_image/2, N_padded)  # Image plane coordinates in Airy units
        y_image_padded = np.linspace(-L_image/2, L_image/2, N_padded) 
        
    X_image_padded, Y_image_padded = np.meshgrid(x_image_padded, y_image_padded)

    if diameter_in_angular_units:
        mask = np.sqrt(X_image_padded**2 + Y_image_padded**2) <= mask_diam / 4
    else: 
        mask = np.sqrt(X_image_padded**2 + Y_image_padded**2) <= mask_diam / 4
        
    if coldstop_diam is not None:
        coldmask = np.sqrt(X_image_padded**2 + Y_image_padded**2) <= coldstop_diam / 4
    else:
        coldmask = np.ones(X_image_padded.shape)

    pupil_ft = np.fft.fft2(np.fft.ifftshift(pupil_padded))  # Remove outer fftshift
    pupil_ft = np.fft.fftshift(pupil_ft)  # Shift only once at the end

    psi_B = coldmask * pupil_ft
                            
    b = np.fft.fftshift( np.fft.ifft2( mask * psi_B ) ) 

    
    if debug: 
        
        psf = np.abs(pupil_ft)**2  # Get the PSF by taking the square of the absolute value
        psf /= np.max(psf)  # Normalize PSF intensity
        
        if diameter_in_angular_units:
            zoom_range = 3  # Number of Airy disk radii to zoom in on
        else:
            zoom_range = 3 * airy_scale 
            
        extent = (-zoom_range, zoom_range, -zoom_range, zoom_range)

        fig,ax = plt.subplots(1,1)
        ax.imshow(psf, extent=(x_image_padded.min(), x_image_padded.max(), y_image_padded.min(), y_image_padded.max()), cmap='gray')
        ax.contour(X_image_padded, Y_image_padded, mask, levels=[0.5], colors='red', linewidths=2, label='phasemask')
        #ax[1].imshow( mask, extent=(x_image_padded.min(), x_image_padded.max(), y_image_padded.min(), y_image_padded.max()), cmap='gray')
        #for axx in ax.reshape(-1):
        #    axx.set_xlim(-zoom_range, zoom_range)
        #    axx.set_ylim(-zoom_range, zoom_range)
        ax.set_xlim(-zoom_range, zoom_range)
        ax.set_ylim(-zoom_range, zoom_range)
        ax.set_title( 'PSF' )
        ax.legend() 
        #ax[1].set_title('phasemask')


    
    # if considering complex b 
    # beta = np.angle(b) # complex argunment of b 
    # M = b * (np.exp(1J*theta)-1)**0.5
    
    # relabelling
    theta = phaseshift # rad , 
    P = pupil_padded.copy() 
    
    if analytic_solution :
        
        M = abs( b ) * np.sqrt((np.cos(theta)-1)**2 + np.sin(theta)**2)
        mu = np.angle((np.exp(1J*theta)-1) ) # np.arctan( np.sin(theta)/(np.cos(theta)-1) ) #
        
        phi = np.zeros( P.shape ) # added aberrations 
        
        # out formula ----------
        #if measured_pupil!=None:
        #    P = measured_pupil / np.mean( P[P > np.mean(P)] ) # normalize by average value in Pupil
        
        Ic = ( P**2 + abs(M)**2 + 2* P* abs(M) * np.cos(phi + mu) ) #+ beta)
        if not get_individual_terms:
            return( P, Ic )
        else:
            return( P, abs(M) , phi+mu )
    else:
        
        # phasemask filter 
        
        T_on = 1
        T_off = 1
        H = T_off*(1 + (T_on/T_off * np.exp(1j * theta) - 1) * mask  ) 
        
        Ic = abs( np.fft.fftshift( np.fft.ifft2( H * psi_B ) ) ) **2 
    
        return( P, Ic )




def interpolate_pupil_to_measurement(original_pupil, original_image, M, N, m, n, x_c, y_c, new_radius):
    """
    Interpolate the pupil onto a new grid, centering the original pupil at (x_c, y_c) 
    and giving it a specified radius in the new grid.
    
    Parameters:
    - pupil: Original MxN pupil array.
    - original_image: original image (i.e intensity with phasemask in) corresponding to the pupil (phasemask out)
    - M, N: Size of the original grid.
    - n, m: Size of the new grid.
    - x_c, y_c: Center of the pupil in the new grid (in pixels).
    - new_radius: The desired radius of the pupil in the new grid (in pixels).
    
    Returns:
    - new_pupil: The pupil interpolated onto the new grid (nxm).
    """
    # Original grid coordinates (centered at the middle)
    x_orig = np.linspace(-M/2, M/2, M)
    y_orig = np.linspace(-N/2, N/2, N)
    #X_orig, Y_orig = np.meshgrid(x_orig, y_orig)
    
    # Create the new grid coordinates (centered)
    x_new = np.linspace(-m/2, m/2, m)  # New grid should also be centered
    y_new = np.linspace(-n/2, n/2, n)
    X_new, Y_new = np.meshgrid(x_new, y_new)

    # Find the actual radius of the original pupil in terms of grid size (not M/2)
    orig_radius = np.sum( original_pupil/np.pi )**0.5 #np.sqrt((X_orig**2 + Y_orig**2).max())

    # Map new grid coordinates to the original grid
    scale_factor = new_radius / orig_radius  # Correct scaling factor based on actual original radius
    X_new_mapped = (X_new - x_c + m/2) / scale_factor + M/2
    Y_new_mapped = (Y_new - y_c + n/2) / scale_factor + N/2

    # Perform interpolation using map_coordinates
    new_pupil = ndimage.map_coordinates(original_image, [Y_new_mapped.ravel(), X_new_mapped.ravel()], order=1, mode='constant', cval=0)
    
    # Reshape the interpolated result to the new grid size
    new_pupil = new_pupil.reshape(n, m)

    return new_pupil




def filter_exterior_annulus(pupil_mask, inner_radius, outer_radius):
    """
    Generate a boolean mask that filters pixels exterior to the circular pupil
    but within the specified inner and outer radii.
    """
    # Get the image shape
    ny, nx = pupil_mask.shape

    # Compute the pupil center (mean of True pixels)
    y_indices, x_indices = np.where(pupil_mask)
    center_x = np.mean(x_indices)
    center_y = np.mean(y_indices)

    # Generate a coordinate grid
    X, Y = np.meshgrid(np.arange(nx), np.arange(ny))

    # Compute the Euclidean distance of each pixel from the pupil center
    distance_from_center = np.sqrt((X - center_x) ** 2 + (Y - center_y) ** 2)

    # Create an annular mask where pixels are within the given inner and outer radius
    annular_mask = (distance_from_center >= inner_radius) & (distance_from_center <= outer_radius)

    return annular_mask


def remove_boundary(mask):
    """
    Remove boundary pixels from a boolean mask.
    
    Parameters:
        mask (np.ndarray): 2D boolean array.
        
    Returns:
        np.ndarray: New mask where any True pixel that touches a False pixel (in its 3x3 neighborhood)
                    has been set to False.
    """
    # Create a 3x3 structure element that considers all 8 neighbors.
    structure = np.ones((3, 3), dtype=bool)
    # binary_erosion returns a new mask where a pixel is True only if all pixels in its neighborhood
    # (defined by structure) were True in the original mask.
    return binary_erosion(mask, structure=structure)


def get_secondary_mask(image, center):
    """
    Create a boolean mask with the same shape as `image` that is True 
    for a 3x3 patch centered at the given (x,y) coordinate (floats)
    and False elsewhere. x,y is rounded to nearet integer

    Designed for identifying pixels shaddowed by secondary obstruction. 
    Use detect_pupil() function to get the center! 
    
    Parameters:
        image (np.ndarray): 2D array (image).
        center (tuple): (x, y) coordinate (floats) of the patch center.
                        x is column, y is row.
    
    Returns:
        mask (np.ndarray): Boolean mask array with True in the 3x3 patch.
    """
    # Initialize a boolean mask of the same shape as the image with all False
    mask = np.zeros_like(image, dtype=bool)
    
    # Unpack the center coordinates and round to nearest integer
    x, y = center
    col = int(round(x))
    row = int(round(y))
    
    # Set the 3x3 patch to True.
    # Note: This simple example assumes the patch is fully contained in the image.
    mask[row-1:row+2, col-1:col+2] = True
    
    return mask



def detect_circle(image, sigma=2, threshold=0.5, plot=False):
    """
    Detects a circular pupil in a given image using edge detection and circle fitting.

    Returns:
        (center_x, center_y, radius) of the detected pupil.
    """
    

    # Normalize and smooth the image
    image = image / np.max(image)
    smoothed_image = gaussian_filter(image, sigma=sigma)

    # Compute gradient-based edge detection
    grad_x = np.gradient(smoothed_image, axis=1)
    grad_y = np.gradient(smoothed_image, axis=0)
    edges = np.sqrt(grad_x**2 + grad_y**2)

    # Threshold the edges
    binary_edges = edges > (threshold * edges.max())

    # Get edge coordinates
    y, x = np.nonzero(binary_edges)

    # Initial guess: Center is mean position of edge points
    center_x, center_y = np.mean(x), np.mean(y)
    radius = np.sqrt(((x - center_x) ** 2 + (y - center_y) ** 2).mean())

    if plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots()
        ax.imshow(image, cmap="gray", origin="upper")
        circle = plt.Circle((center_x, center_y), radius, color='red', fill=False)
        ax.add_patch(circle)
        ax.scatter(center_x, center_y, color='blue', marker='+')
        plt.title("Detected Pupil")
        plt.show()

    return int(center_x), int(center_y), int(radius)




def percentile_based_detect_pupils(
    image, percentile=80, min_group_size=50, buffer=20, square_region=True, plot=True
):
    """
    Detects circular pupils by identifying regions with grouped pixels above a given percentile.

    Parameters:
        image (2D array): Full grayscale image containing multiple pupils.
        percentile (float): Percentile of pixel intensities to set the threshold (default 80th).
        min_group_size (int): Minimum number of adjacent pixels required to consider a region.
        buffer (int): Extra pixels to add around the detected region for cropping.
        plot (bool): If True, displays the detected regions and coordinates.

    Returns:
        list of tuples: Cropping coordinates [(x_start, x_end, y_start, y_end), ...].
    """
    # Normalize the image
    image = image / image.max()

    # Calculate the intensity threshold as the 80th percentile
    threshold = np.percentile(image, percentile)

    # Create a binary mask where pixels are above the threshold
    binary_image = image > threshold

    # Label connected regions in the binary mask
    labeled_image, num_features = label(binary_image)

    # Extract regions and filter by size
    regions = find_objects(labeled_image)
    pupil_regions = []
    for region in regions:
        y_slice, x_slice = region
        # Count the number of pixels in the region
        num_pixels = np.sum(labeled_image[y_slice, x_slice] > 0)
        if num_pixels >= min_group_size:
            # Add a buffer around the region for cropping
            y_start = max(0, y_slice.start - buffer)
            y_end = min(image.shape[0], y_slice.stop + buffer)
            x_start = max(0, x_slice.start - buffer)
            x_end = min(image.shape[1], x_slice.stop + buffer)

            if square_region:
                max_delta = np.max( [x_end - x_start,y_end - y_start] )                
                pupil_regions.append((x_start, x_start+max_delta, y_start, y_start+max_delta))
            else:
                pupil_regions.append((x_start, x_end, y_start, y_end))
            
    if plot:
        # Plot the original image with bounding boxes
        plt.figure(figsize=(10, 10))
        plt.imshow(image, cmap="gray", origin="upper")
        for x_start, x_end, y_start, y_end in pupil_regions:
            rect = plt.Rectangle(
                (x_start, y_start),
                x_end - x_start,
                y_end - y_start,
                edgecolor="red",
                facecolor="none",
                linewidth=2,
            )
            plt.gca().add_patch(rect)
        plt.title(f"Detected Pupils: {len(pupil_regions)}")
        plt.savefig('delme.png')
        plt.show()
        

    return pupil_regions

def detect_pupil(image, sigma=2, threshold=0.5, plot=True, savepath=None):
    """
    Detects an elliptical pupil (with possible rotation) in a cropped image using edge detection 
    and least-squares fitting. Returns both the ellipse parameters and a pupil mask.

    This is a generalized version of detect circle function 
    The ellipse is modeled by:

        ((x - cx)*cos(theta) + (y - cy)*sin(theta))^2 / a^2 +
        (-(x - cx)*sin(theta) + (y - cy)*cos(theta))^2 / b^2 = 1

    Parameters:
        image (2D array): Cropped grayscale image containing a single pupil.
        sigma (float): Standard deviation for Gaussian smoothing.
        threshold (float): Threshold factor for edge detection.
        plot (bool): If True, displays the image with the fitted ellipse overlay.
        savepath (str): If provided, the plot is saved to this path.

    Returns:
        (center_x, center_y, a, b, theta, pupil_mask)
          where (center_x, center_y) is the ellipse center,
                a and b are the semimajor and semiminor axes,
                theta is the rotation angle in radians,
                pupil_mask is a 2D boolean array (True = inside ellipse).
    """
    # Normalize the image
    image = image / image.max()
    
    # Smooth the image
    smoothed_image = gaussian_filter(image, sigma=sigma)
    
    # Compute gradients (Sobel-like edge detection)
    grad_x = np.gradient(smoothed_image, axis=1)
    grad_y = np.gradient(smoothed_image, axis=0)
    edges = np.sqrt(grad_x**2 + grad_y**2)
    
    # Threshold edges to create a binary mask
    binary_edges = edges > (threshold * edges.max())
    
    # Get edge pixel coordinates
    y_coords, x_coords = np.nonzero(binary_edges)
    
    # Initial guess: center from mean, radius from average distance, and theta = 0.
    def initial_guess(x, y):
        center_x = np.mean(x)
        center_y = np.mean(y)
        r_init = np.sqrt(np.mean((x - center_x)**2 + (y - center_y)**2))
        return center_x, center_y, r_init, r_init, 0.0  # (cx, cy, a, b, theta)
    
    # Ellipse model function with rotation.
    def ellipse_model(params, x, y):
        cx, cy, a, b, theta = params
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        x_shift = x - cx
        y_shift = y - cy
        xp =  cos_t * x_shift + sin_t * y_shift
        yp = -sin_t * x_shift + cos_t * y_shift
        # Model: xp^2/a^2 + yp^2/b^2 = 1 => residual = sqrt(...) - 1
        return np.sqrt((xp/a)**2 + (yp/b)**2) - 1.0

    # Fit via least squares.
    guess = initial_guess(x_coords, y_coords)
    result, _ = leastsq(ellipse_model, guess, args=(x_coords, y_coords))
    center_x, center_y, a, b, theta = result
    
    # Create a boolean pupil mask for the fitted ellipse
    yy, xx = np.ogrid[:image.shape[0], :image.shape[1]]
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    x_shift = xx - center_x
    y_shift = yy - center_y
    xp = cos_t * x_shift + sin_t * y_shift
    yp = -sin_t * x_shift + cos_t * y_shift
    pupil_mask = (xp/a)**2 + (yp/b)**2 <= 1

    if plot:
        # Overlay for visualization
        overlay = np.zeros_like(image)
        overlay[pupil_mask] = 1
        
        plt.figure(figsize=(6, 6))
        plt.imshow(image, cmap="gray", origin="upper")
        plt.contour(binary_edges, colors="cyan", linewidths=1)
        plt.contour(overlay, colors="red", linewidths=1)
        plt.scatter(center_x, center_y, color="blue", marker="+")
        plt.title("Detected Pupil with Fitted Ellipse")
        if savepath is not None:
            plt.savefig(savepath)
        plt.show()
    
    return center_x, center_y, a, b, theta, pupil_mask


def crop_to_square(mask):
    # Find the (row, col) indices of the pupil (True values)
    indices = np.argwhere(mask)
    if indices.size == 0:
        raise ValueError("Mask has no True values.")
    
    # Determine the bounding box of the pupil
    y_min, x_min = indices.min(axis=0)
    y_max, x_max = indices.max(axis=0)
    
    # Determine the required side length for the square (largest dimension)
    height = y_max - y_min + 1
    width = x_max - x_min + 1
    side = max(width, height)
    
    # Compute the pupil center as a float for subpixel precision
    center_y = (y_min + y_max) / 2.0
    center_x = (x_min + x_max) / 2.0
    
    # Calculate the crop boundaries so that the pupil is centered.
    # Using floor when subtracting half the side helps keep the crop symmetric.
    new_y_min = int(np.floor(center_y - side / 2.0))
    new_x_min = int(np.floor(center_x - side / 2.0))
    new_y_max = new_y_min + side
    new_x_max = new_x_min + side

    # Adjust if the computed indices extend beyond the image boundaries.
    if new_y_min < 0:
        new_y_min = 0
        new_y_max = side
    if new_x_min < 0:
        new_x_min = 0
        new_x_max = side
    if new_y_max > mask.shape[0]:
        new_y_max = mask.shape[0]
        new_y_min = new_y_max - side
    if new_x_max > mask.shape[1]:
        new_x_max = mask.shape[1]
        new_x_min = new_x_max - side
        
    return new_y_min, new_y_max, new_x_min, new_x_max


def get_mask_center(mask,  method='2'):
    """
    for a 2D boolean mask we 
    detect the edge coordinates in x,y and the centers
    """

    if method == '1':
        x_edge_pixels = np.where(abs(np.diff( np.nansum( mask, axis=0)>0 ) ) > 0)
        y_edge_pixels = np.where(abs(np.diff( np.nansum( mask, axis=1)>0 ) ) > 0)

        if len( x_edge_pixels ) > 0:
            x_c = np.mean( x_edge_pixels[0] )
        else:
            x_c = None 

        if len( y_edge_pixels ) > 0:
            y_c = np.mean( y_edge_pixels[0] )
        else:
            y_c = None 

    elif method=='2':
        x_c = np.mean(  np.where( mask )[0])
        y_c = np.mean(  np.where( mask )[1])

    return( x_c, y_c )

def get_circle_DM_command(radius, Nx_act=12):
    """
    Generates a DM command that forms a circular shape of the given radius.

    Parameters:
        radius (float): Desired radius in actuator units.
        Nx_act (int, optional): Number of actuators per side of the DM (default 12).

    Returns:
        cmd (ndarray): A 140-length DM command vector with a circular shape.
    """
    # Generate actuator coordinate grid
    x = np.arange(Nx_act)
    y = np.arange(Nx_act)
    X, Y = np.meshgrid(x, y)

    # Compute distances from the center of the DM grid
    center = (Nx_act - 1) / 2  # DM is 12x12, so center is at (5.5, 5.5)
    distances = np.sqrt((X - center) ** 2 + (Y - center) ** 2)

    # Mask actuators inside the desired radius
    mask = distances <= radius

    # Flatten the mask and remove corner actuators
    mask_flattened = mask.flatten()
    corner_indices = [0, Nx_act-1, Nx_act*(Nx_act-1), Nx_act*Nx_act-1]
    mask_flattened = np.delete(mask_flattened, corner_indices)

    # Create the DM command vector of length 140
    cmd = np.zeros(140)
    cmd[mask_flattened] = 1  # Set selected actuators to 1

    return cmd


# Planck's law function for spectral radiance
def planck_law(wavelength, T):
    """Returns spectral radiance (Planck's law) at a given wavelength and temperature."""
    h = 6.62607015e-34
    c = 299792458.0
    k = 1.380649e-23
    return (2 * h * c**2) / (wavelength**5) / (np.exp(h * c / (wavelength * k * T)) - 1)

# Function to find the weighted average wavelength (central wavelength)
def find_central_wavelength(lambda_cut_on, lambda_cut_off, T):
    # Define integrands for energy and weighted wavelength
    def _integrand_energy(wavelength):
        return planck_law(wavelength, T)

    def _integrand_weighted(wavelength):
        return planck_law(wavelength, T) * wavelength

    # Integrate to find total energy and weighted energy
    total_energy, _ = quad(_integrand_energy, lambda_cut_on, lambda_cut_off)
    weighted_energy, _ = quad(_integrand_weighted, lambda_cut_on, lambda_cut_off)
    
    # Calculate the central wavelength as the weighted average wavelength
    central_wavelength = weighted_energy / total_energy
    return central_wavelength


def get_phasemask_phaseshift( wvl, depth, dot_material = 'N_1405' ):
    """
    wvl is wavelength in micrometers
    depth is the physical depth of the phasemask in micrometers
    dot material is the material of phaseshifting object

    it is assumed phasemask is in air (n=1).
    N_1405 is photoresist used for making phasedots in Sydney
    """
    print( 'reminder wvl input should be um!')
    if dot_material == 'N_1405':
        # wavelengths in csv file are in nanometers
        df = pd.read_csv('Exposed_Ma-N_1405_optical_constants.txt', sep='\s+', header=1)
        f = interp1d(df['Wavelength(nm)'], df['n'], kind='linear',fill_value=np.nan, bounds_error=False)
        n = f( wvl * 1e3 ) # convert input wavelength um - > nm
        phaseshift = 2 * np.pi/ wvl  * depth * (n -1)
        return( phaseshift )
    
    else:
        raise TypeError('No corresponding dot material for given input. Try N_1405.')


def square_spiral_scan(starting_point, step_size, search_radius):
    """
    Generates a square spiral scan pattern starting from the initial point within a given search radius and step size.
    
    Parameters:
    starting_point (tuple): The initial (x, y) point to start the spiral.
    step_size (float): The size of each step in the grid.
    search_radius (float): The maximum radius to scan in both x and y directions.

    Returns:
    list: A list of tuples where each tuple contains (x_amp, y_amp), the left/right and up/down amplitudes for the scan.
    """
    x, y = starting_point  # Start at the given initial point
    dx, dy = step_size, 0  # Initial movement to the right
    scan_points = [(x, y)]
    steps_taken = 0  # Counter for steps taken in the current direction
    step_limit = 1  # Initial number of steps in each direction

    while max(abs(x - starting_point[0]), abs(y - starting_point[1])) <= search_radius:
        for _ in range(2):  # Repeat twice: once for horizontal, once for vertical movement
            for _ in range(step_limit):
                x, y = x + dx, y + dy
                if max(abs(x - starting_point[0]), abs(y - starting_point[1])) > search_radius:
                    return scan_points
                scan_points.append((x, y))
            
            # Rotate direction (right -> up -> left -> down)
            dx, dy = -dy, dx

        # Increase step limit after a complete cycle (right, up, left, down)
        step_limit += 1

    return scan_points


def spiral_search_TT_coefficients( dr, dtheta, aoi_tp, aoi_tt, num_points, r0=0, theta0=0):
    """
    generate tip (tp) / tilt (tt) coefficients for a spiral search covering
    "num_points" samples with angular increments dtheta, radial increments dr
    aoi_tp, aoi_tilt are the anlge of incidence on the DM for tip and tilt. 

    ALL angular units should be input as radians.
    """

    coefficients = []
    theta = theta0 # initial angle
    radius = r0 # initial radius
    
    for _ in range(num_points):
        a_tp = radius * np.cos(theta) * np.cos( aoi_tp )
        a_Tt = radius * np.sin(theta) * np.cos( aoi_tt )
        coefficients.append((a_tp, a_Tt))
        
        # Increment radius and angle
        radius += dr
        theta += dtheta
    
    return coefficients


def get_DM_command_in_2D(cmd,Nx_act=12):
    # function so we can easily plot the DM shape (since DM grid is not perfectly square raw cmds can not be plotted in 2D immediately )
    #puts nan values in cmd positions that don't correspond to actuator on a square grid until cmd length is square number (12x12 for BMC multi-2.5 DM) so can be reshaped to 2D array to see what the command looks like on the DM.
    corner_indices = [0, Nx_act-1, Nx_act * (Nx_act-1), Nx_act*Nx_act]
    cmd_in_2D = list(cmd.copy())
    for i in corner_indices:
        cmd_in_2D.insert(i,np.nan)
    return( np.array(cmd_in_2D).reshape(Nx_act,Nx_act) )

def convert_12x12_to_140(arr):
    # Convert input to a NumPy array (if it isn't already)
    arr = np.asarray(arr)
    
    if arr.shape != (12, 12):
        raise ValueError("Input must be a 12x12 array.")
    
    # Flatten the array (row-major order)
    flat = arr.flatten()
    
    # The indices for the four corners in a 12x12 flattened array (row-major order):
    # Top-left: index 0
    # Top-right: index 11
    # Bottom-left: index 11*12 = 132
    # Bottom-right: index 143 (11*12 + 11)
    corner_indices = [0, 11, 132, 143]
    
    # Delete the corner elements from the flattened array
    vector = np.delete(flat, corner_indices)
    
    return vector

def circle(radius, size, circle_centre=(0, 0), origin="middle"):
    
    """
    Adopted from AO tools with edit that we can use size as a tuple of row_size, col_size to include rectangles 
    
    Create a 2-D array: elements equal 1 within a circle and 0 outside.

    The default centre of the coordinate system is in the middle of the array:
    circle_centre=(0,0), origin="middle"
    This means:
    if size is odd  : the centre is in the middle of the central pixel
    if size is even : centre is in the corner where the central 4 pixels meet

    origin = "corner" is used e.g. by psfAnalysis:radialAvg()

    Examples: ::

        circle(1,5) circle(0,5) circle(2,5) circle(0,4) circle(0.8,4) circle(2,4)
          00000       00000       00100       0000        0000          0110
          00100       00000       01110       0000        0110          1111
          01110       00100       11111       0000        0110          1111
          00100       00000       01110       0000        0000          0110
          00000       00000       00100

        circle(1,5,(0.5,0.5))   circle(1,4,(0.5,0.5))
           .-->+
           |  00000               0000
           |  00000               0010
          +V  00110               0111
              00110               0010
              00000

    Parameters:
        radius (float)       : radius of the circle
        size (int)           : tuple of row  and column size of the 2-D array in which the circle lies
        circle_centre (tuple): coords of the centre of the circle
        origin (str)  : where is the origin of the coordinate system
                               in which circle_centre is given;
                               allowed values: {"middle", "corner"}

    Returns:
        ndarray (float64) : the circle array
    """
	
    size_row , size_col = size
    # (2) Generate the output array:
    C = np.zeros((size_row, size_col))

    # (3.a) Generate the 1-D coordinates of the pixel's centres:
    # coords = np.linspace(-size/2.,size/2.,size) # Wrong!!:
    # size = 5: coords = array([-2.5 , -1.25,  0.  ,  1.25,  2.5 ])
    # size = 6: coords = array([-3. , -1.8, -0.6,  0.6,  1.8,  3. ])
    # (2015 Mar 30; delete this comment after Dec 2015 at the latest.)

    # Before 2015 Apr 7 (delete 2015 Dec at the latest):
    # coords = np.arange(-size/2.+0.5, size/2.-0.4, 1.0)
    # size = 5: coords = array([-2., -1.,  0.,  1.,  2.])
    # size = 6: coords = array([-2.5, -1.5, -0.5,  0.5,  1.5,  2.5])

    coords_r = np.arange(0.5, size_row, 1.0)
    coords_c = np.arange(0.5, size_col, 1.0)
    # size = 5: coords = [ 0.5  1.5  2.5  3.5  4.5]
    # size = 6: coords = [ 0.5  1.5  2.5  3.5  4.5  5.5]

    # (3.b) Just an internal sanity check:
    if len(coords_r) != size_row:
        print('opps')

    # (3.c) Generate the 2-D coordinates of the pixel's centres:
    x, y = np.meshgrid(coords_c, coords_r)

    # (3.d) Move the circle origin to the middle of the grid, if required:
    if origin == "middle":
        x -= size_col / 2.
        y -= size_row / 2.

    # (3.e) Move the circle centre to the alternative position, if provided:
    x -= circle_centre[0]
    y -= circle_centre[1]

    # (4) Calculate the output:
    # if distance(pixel's centre, circle_centre) <= radius:
    #     output = 1
    # else:
    #     output = 0
    mask = x * x + y * y <= radius * radius
    C[mask] = 1

    # (5) Return:
    return C


def shift(xs, n, m, fill_value=np.nan):
    # shifts a 2D array xs by n rows, m columns and fills the new region with fill_value

    e = xs.copy()
    if n!=0:
        if n >= 0:
            e[:n,:] = fill_value
            e[n:,:] =  e[:-n,:]
        else:
            e[n:,:] = fill_value
            e[:n,:] =  e[-n:,:]
   
       
    if m!=0:
        if m >= 0:
            e[:,:m] = fill_value
            e[:,m:] =  e[:,:-m]
        else:
            e[:,m:] = fill_value
            e[:,:m] =  e[:,-m:]
    return e


def line_intersection(line1, line2):
    """
    find intersection of lines given by their endpoints, 
       line1 = (A,B)
       line2 = (C,D)
       where A=[x1_1, y1_1], B=[x1_2,y1_2], are end points of line1 
             C=[x2_1, y2_1], D=[x2_2, y2_2], are end points of line2
        
    """
 
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return( x, y )



def get_reference_images(zwfs, phasemask, theta_degrees=11.8, number_of_frames=256, compass = True, compass_origin=None, savefig='tmp/delme.png' ):
    """
    see document in Asgard/03 modules/Baldr/Baldr_detector_reference_coordinates_calibration.docx 
    for description of x,y coordinate conventions in DM plance etc.
    +x is right facing DM, +y up facing DM. 
    measured to be 11.8 degrees in DM plane - this is default
    """

    dx, dy = 200, 200  #offsets to apply to phasemask

    I0 =  np.mean(zwfs.get_some_frames(number_of_frames = number_of_frames, apply_manual_reduction = True ) , axis=0 )

    if compass_origin==None:
        x_pos, y_pos = 0.85 * I0.shape[0], 0.15 * I0.shape[0] #  origin of compass default
    phasemask.move_relative( [dx,dy] ) # move out 
    time.sleep(0.1)

    N0 = np.mean(zwfs.get_some_frames(number_of_frames = 256, apply_manual_reduction = True ) , axis=0 )
    
    phasemask.move_relative( [-dx,-dy] ) # move back in
    time.sleep(0.1)

    im_list = [I0/np.max(N0) , N0/np.max(N0) ]
    xlabel_list = [None, None]
    ylabel_list = [None, None]
    title_list = [r'$I_0$', r'$N_0$']
    cbar_label_list = ['Intensity (Normalized)', 'Intensity (Normalized)'] 
    #fig_path + 'delme.png' #f'mode_reconstruction_images/phase_reconstruction_example_mode-{mode_indx}_basis-{phase_ctrl.config["basis"]}_ctrl_modes-{phase_ctrl.config["number_of_controlled_modes"]}ctrl_act_diam-{phase_ctrl.config["dm_control_diameter"]}_readout_mode-12x12.png'

    n = len(im_list)
    fs = 15
    fig = plt.figure(figsize=(5*n, 5))

    for a in range(n) :
        ax1 = fig.add_subplot(int(f'1{n}{a+1}'))
        im1 = ax1.imshow(  im_list[a] , vmin =  np.min(im_list[-1]), vmax = np.max([np.max(im_list[-1]), np.max(im_list[0])]) )


        ax1.set_title( title_list[a] ,fontsize=fs)
        ax1.set_xlabel( xlabel_list[a] ,fontsize=fs) 
        ax1.set_ylabel( ylabel_list[a] ,fontsize=fs) 
        ax1.tick_params( labelsize=fs ) 

        

        divider = make_axes_locatable(ax1)
        cax = divider.append_axes('bottom', size='5%', pad=0.05)
        cbar = fig.colorbar( im1, cax=cax, orientation='horizontal')
        cbar.set_label( cbar_label_list[a], rotation=0,fontsize=fs)
        cbar.ax.tick_params(labelsize=fs)

        if (a==0) & compass:
            # Convert theta from degrees to radians
            theta = np.radians(theta_degrees)
            
            # Define the base vectors (unit vectors along y and x axis)
            y_vector = 0.2 * im_list[a].shape[0] * np.array([0, 1])
            x_vector = -0.2 * im_list[a].shape[0] * np.array([1, 0])
            
            # Create the rotation matrix
            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                        [np.sin(theta),  np.cos(theta)]])
            
            # Rotate the vectors
            rotated_y_vector = rotation_matrix @ y_vector
            rotated_x_vector = rotation_matrix @ x_vector
            
            # Plot the arrows at the specified coordinates

            ax1.quiver(x_pos, y_pos, rotated_y_vector[0], rotated_y_vector[1], angles='xy', scale_units='xy', scale=1, color='r', label='y')
            ax1.quiver(x_pos, y_pos, rotated_x_vector[0], rotated_x_vector[1], angles='xy', scale_units='xy', scale=1, color='r', label='x')
            
            # Add labels at the end of the arrows
            ax1.text(x_pos + 1.2*rotated_y_vector[0], y_pos + 1.2*rotated_y_vector[1], r'$x$', fontsize=12, ha='right',color='r')
            ax1.text(x_pos + 1.2*rotated_x_vector[0], y_pos + 1.2*rotated_x_vector[1], r'$y$', fontsize=12, ha='right',color='r')
        
        ax1.xaxis.tick_top()

    if savefig!=None: 
        plt.savefig( savefig , bbox_inches='tight', dpi=300)

    return(I0, N0)
        

def shape_dm_manually(zwfs, compass = True , initial_cmd = None ,number_of_frames=5, apply_manual_reduction=True, theta_degrees=11.8, flip_dm=True, savefig=None):
    if initial_cmd == None:
        cmd =  zwfs.dm_shapes['flat_dm'].copy()
    else:
        assert len(cmd) == 140
        assert np.max(cmd) < 1
        assert np.min(cmd) > 0
        cmd = initial_cmd


    try: 
        # try send the initial command
        zwfs.dm.send_data( cmd  )
    except:
        raise TypeError('For some reason we cannot send the initial_cmd to the DM. Check it!')

    # get an initial image 
    initial_img = np.mean(zwfs.get_some_frames(number_of_frames = number_of_frames, apply_manual_reduction = apply_manual_reduction ) , axis=0 )

    if compass:
        x_pos, y_pos = 0.85 * initial_img.shape[0], 0.15 * initial_img.shape[1] # compass origin
    e0=True
    e1=False
    while e0 :
        act =  input('input actuator number (1-140) to move, "e" to exit') 
        if act == 'e':
            e0 =False
        else:
            try:
                act = int( act ) - 1 # subtract one since array index starts at 0
            except:
                print('actuator must be integer between 1-140, or enter "e" to exit')
            if (act < 1) or (act > 140):
                print('actuator must be integer between 1-140, or enter "e" to exit')
            else:
                e1 = True
        while e1 :
            dc =  input('input relative movement of actuator (-0.5 - 0.5) a hint is "0.05", or "e" to exit') 
            if dc == 'e':
                e1 =False
            else:
                try:
                    dc = float( dc )
                except:
                    print('relative movement of actuator must be float between (-1 - 1), or enter "e" to exit')
                #if (dc < 0) or (dc > 1):
                #    print('relative movement of actuator must be float between (-1 - 1), or enter "e" to exit')
                 
                # apply relative displacement to specified actuator
                if (cmd[act]+dc < 0) or (cmd[act]+dc > 1):
                    print(f'current value {cmd[act]+dc} hitting limits of actuator (0 - 1), \ntry move in opposite dirrection or lower amplitude,\nor enter "e" to exit')
                else:
                    cmd[act] += dc 
                    #send the command 
                    zwfs.dm.send_data( cmd ) 
                    time.sleep(0.1)
                    # look at the new image 
                    new_img = np.mean(zwfs.get_some_frames(number_of_frames = number_of_frames, apply_manual_reduction = apply_manual_reduction ) , axis=0 )

                    # plotting results
                    if flip_dm:
                        cmd2plot = np.flipud( get_DM_command_in_2D( cmd ) )
                    else:
                        cmd2plot = get_DM_command_in_2D( cmd )  
                    im_list = [cmd2plot ,new_img , initial_img ]
                    xlabel_list = [None, None, None]
                    ylabel_list = [None, None, None]
                    title_list = ['current DM\ncommand',r'current image', r'initial image']
                    cbar_label_list = ['DM units','Intensity (Normalized)', 'Intensity (Normalized)'] 
                    #fig_path + 'delme.png' #f'mode_reconstruction_images/phase_reconstruction_example_mode-{mode_indx}_basis-{phase_ctrl.config["basis"]}_ctrl_modes-{phase_ctrl.config["number_of_controlled_modes"]}ctrl_act_diam-{phase_ctrl.config["dm_control_diameter"]}_readout_mode-12x12.png'

                    n = len(im_list)
                    fs = 15
                    fig = plt.figure(figsize=(5*n, 5))

                    for a in range(n) :
                        ax1 = fig.add_subplot(int(f'1{n}{a+1}'))
                        if a != 0:
                            im1 = ax1.imshow(  im_list[a] , vmin = np.min(im_list[-1]), vmax = np.max(im_list[-1]))
                        else: # DM command
                            im1 = ax1.imshow(  im_list[a]  )
                            n_rows, n_cols = im_list[a].shape
                            #Annotate each square with the flattened index
                            for i in range(n_rows):
                               for j in range(n_cols):
                                    # Calculate the flattened index
                                    
                                    if flip_dm:
                                        flattened_index = (n_rows - 1 - i) * n_cols + j
                                    else:
                                        flattened_index = i * n_cols + j    
                                    # Add the flattened index as text in the plot. make index between 1-140 so add 1
                                    ax1.text(j, i, f'{flattened_index}', va='center', ha='center', color='white')

                        ax1.set_title( title_list[a] ,fontsize=fs)
                        ax1.set_xlabel( xlabel_list[a] ,fontsize=fs) 
                        ax1.set_ylabel( ylabel_list[a] ,fontsize=fs) 
                        ax1.tick_params( labelsize=fs ) 

                        

                        divider = make_axes_locatable(ax1)
                        cax = divider.append_axes('bottom', size='5%', pad=0.05)
                        cbar = fig.colorbar( im1, cax=cax, orientation='horizontal')
                        cbar.set_label( cbar_label_list[a], rotation=0,fontsize=fs)
                        cbar.ax.tick_params(labelsize=fs)

                        if (a==1) & compass:
                            # Convert theta from degrees to radians
                            theta = np.radians(theta_degrees)
                            
                            # Define the base vectors (unit vectors along y and x axis)
                            y_vector = 0.2 * im_list[a].shape[0] * np.array([0, 1])
                            x_vector = -0.2 * im_list[a].shape[0] * np.array([1, 0])
                            
                            # Create the rotation matrix
                            rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                                        [np.sin(theta),  np.cos(theta)]])
                            
                            # Rotate the vectors
                            rotated_y_vector = rotation_matrix @ y_vector
                            rotated_x_vector = rotation_matrix @ x_vector
                            
                            # Plot the arrows at the specified coordinates

                            ax1.quiver(x_pos, y_pos, rotated_y_vector[0], rotated_y_vector[1], angles='xy', scale_units='xy', scale=1, color='r', label='y')
                            ax1.quiver(x_pos, y_pos, rotated_x_vector[0], rotated_x_vector[1], angles='xy', scale_units='xy', scale=1, color='r', label='x')
                            
                            # Add labels at the end of the arrows
                            ax1.text(x_pos + 1.2*rotated_y_vector[0], y_pos + 1.2*rotated_y_vector[1], r'$x$', fontsize=12, ha='right',color='r')
                            ax1.text(x_pos + 1.2*rotated_x_vector[0], y_pos + 1.2*rotated_x_vector[1], r'$y$', fontsize=12, ha='right',color='r')
                        
                        ax1.xaxis.tick_top()

                    if savefig!=None: 
                        plt.savefig( savefig , bbox_inches='tight', dpi=300)

    return( cmd )


def watch_camera(zwfs, frames_to_watch = 10, time_between_frames=0.01,cropping_corners=None) :
  
    print( f'{frames_to_watch} frames to watch with ~{time_between_frames}s wait between frames = ~{5*time_between_frames*frames_to_watch}s watch time' )

    #t0= datetime.datetime.now() 
    plt.figure(figsize=(15,15))
    plt.ion() # turn on interactive mode 
    #FliSdk_V2.Start(camera)     
    seconds_passed = 0
    if type(cropping_corners)==list: 
        x1,x2,y1,y2 = cropping_corners #[row min, row max, col min, col max]

    for i in range(int(frames_to_watch)): 
        
        a = zwfs.get_image()
        if type(cropping_corners)==list: 
            plt.imshow(a[x1:x2,y1:y2])
        else: 
            plt.imshow(a)
        plt.pause( time_between_frames )
        #time.sleep( time_between_frames )
        plt.clf() 
    """
    while seconds_passed < seconds_to_watch:
        a=FliSdk_V2.GetRawImageAsNumpyArray(camera,-1)
        plt.imshow(a)
        plt.pause( time_between_frames )
        time.sleep( time_between_frames )
        plt.clf() 
        t1 = datetime.datetime.now() 
        seconds_passed = (t1 - t0).seconds"""

    #FliSdk_V2.Stop(camera) 
    plt.ioff()# turn off interactive mode 
    plt.close()





def bin_phase_screen(phase_screen, out_size=12):
    """
    Rebin (average) a square 2D phase screen into an out_size x out_size array.
    
    If the phase_screen shape is not exactly divisible by out_size,
    the phase_screen is cropped to the largest size that is a multiple of out_size.
    
    Parameters:
      phase_screen (np.ndarray): 2D array (square) representing the phase screen.
      out_size (int): Desired output size (default: 12)
      
    Returns:
      np.ndarray: A (out_size x out_size) binned phase screen.
    """
    m, n = phase_screen.shape
    if m != n:
        raise ValueError("Phase screen must be square.")
    
    # Crop the array if necessary
    new_dim = (m // out_size) * out_size  # largest multiple of out_size <= m
    if new_dim < m:
        phase_screen = phase_screen[:new_dim, :new_dim]
    
    block_size = new_dim // out_size
    # Reshape so that each block is block_size x block_size and average each block.
    binned = phase_screen.reshape(out_size, block_size, out_size, block_size).mean(axis=(1,3))
    return binned

def create_phase_screen_cmd_for_DM(scrn,  scaling_factor=0.1, drop_indicies = None, plot_cmd=False):
    """
    ### only works for factors of 12 * n , better to use  bin_phase_screen(
    aggregate a scrn (aotools.infinitephasescreen object) onto a DM command space. phase screen is normalized by
    between +-0.5 and then scaled by scaling_factor. Final DM command values should
    always be between -0.5,0.5 (this should be added to a flat reference so flat reference + phase screen should always be bounded between 0-1). phase screens are usually a NxN matrix, while DM is MxM with some missing pixels (e.g. 
    corners). drop_indicies is a list of indicies in the flat MxM DM array that should not be included in the command space. 
    """

    #print('----------\ncheck phase screen size is multiple of DM\n--------')
    
    Nx_act = 12 #number of actuators across DM diameter
    
    scrn_array = ( scrn.scrn - np.min(scrn.scrn) ) / (np.max(scrn.scrn) - np.min(scrn.scrn)) - 0.5 # normalize phase screen between -0.5 - 0.5 
    
    size_factor = int(scrn_array.shape[0] / Nx_act) # how much bigger phase screen is to DM shape in x axis. Note this should be an integer!!
    
    # reshape screen so that axis 1,3 correspond to values that should be aggregated 
    scrn_to_aggregate = scrn_array.reshape(scrn_array.shape[0]//size_factor, size_factor, scrn_array.shape[1]//size_factor, size_factor)
    
    # now aggreagate and apply the scaling factor 
    scrn_on_DM = scaling_factor * np.mean( scrn_to_aggregate, axis=(1,3) ).reshape(-1) 

    #If DM is missing corners etc we set these to nan and drop them before sending the DM command vector
    #dm_cmd =  scrn_on_DM.to_list()
    if drop_indicies != None:
        for i in drop_indicies:
            scrn_on_DM[i]=np.nan
             
    if plot_cmd: #can be used as a check that the command looks right!
        fig,ax = plt.subplots(1,2,figsize=(12,6))
        im0 = ax[0].imshow( scrn_on_DM.reshape([Nx_act,Nx_act]) )
        ax[0].set_title('DM command (averaging offset)')
        im1 = ax[1].imshow(scrn.scrn)
        ax[1].set_title('original phase screen')
        plt.colorbar(im0, ax=ax[0])
        plt.colorbar(im1, ax=ax[1]) 
        plt.show() 

    dm_cmd =  list( scrn_on_DM[np.isfinite(scrn_on_DM)] ) #drop non-finite values which should be nan values created from drop_indicies array
    return(dm_cmd) 






def block_sum(ar, fact): # sums over subwindows of a 2D array
    # ar is the 2D array, fact is the factor to reduce it by 
    # obviously  fact should be factor of ar.shape 
    assert isinstance(fact, int), type(fact)
    sx, sy = ar.shape
    X, Y = np.ogrid[0:sx, 0:sy]
    regions = sy//fact * (X//fact) + Y//fact
    res = ndimage.sum(ar, labels=regions, index=np.arange(regions.max() + 1))
    res.shape = (sx//fact, sy//fact)
    return res




# ========== PLOTTING STANDARDS 

def nice_heatmap_subplots( im_list , xlabel_list=None, ylabel_list=None, title_list=None, cbar_label_list=None, fontsize=15, cbar_orientation = 'bottom', axis_off=True, vlims=None, savefig=None):

    n = len(im_list)
    fs = fontsize
    fig = plt.figure(figsize=(5*n, 5))

    for a in range(n) :
        ax1 = fig.add_subplot(int(f'1{n}{a+1}'))

        if vlims is not None:
            im1 = ax1.imshow(  im_list[a] , vmin = vlims[a][0], vmax = vlims[a][1])
        else:
            im1 = ax1.imshow(  im_list[a] )
        if title_list is not None:
            ax1.set_title( title_list[a] ,fontsize=fs)
        if xlabel_list is not None:
            ax1.set_xlabel( xlabel_list[a] ,fontsize=fs) 
        if ylabel_list is not None:
            ax1.set_ylabel( ylabel_list[a] ,fontsize=fs) 
        ax1.tick_params( labelsize=fs ) 

        if axis_off:
            ax1.axis('off')
        divider = make_axes_locatable(ax1)
        if cbar_orientation == 'bottom':
            cax = divider.append_axes('bottom', size='5%', pad=0.05)
            cbar = fig.colorbar( im1, cax=cax, orientation='horizontal')
                
        elif cbar_orientation == 'top':
            cax = divider.append_axes('top', size='5%', pad=0.05)
            cbar = fig.colorbar( im1, cax=cax, orientation='horizontal')
                
        else: # we put it on the right 
            cax = divider.append_axes('right', size='5%', pad=0.05)
            cbar = fig.colorbar( im1, cax=cax, orientation='vertical')  
        
        if cbar_label_list is not None:
            cbar.set_label( cbar_label_list[a], rotation=0,fontsize=fs)
        cbar.ax.tick_params(labelsize=fs)
    if savefig is not None:
        plt.savefig( savefig , bbox_inches='tight', dpi=300) 





def display_images_with_slider(image_lists, plot_titles=None, cbar_labels=None):
    """
    Displays multiple images or 1D plots from a list of lists with a slider to control the shared index.
    
    Parameters:
    - image_lists: list of lists where each inner list contains either 2D arrays (images) or 1D arrays (scalars).
                   The inner lists must all have the same length.
    - plot_titles: list of strings, one for each subplot. Default is None (no titles).
    - cbar_labels: list of strings, one for each colorbar. Default is None (no labels).
    """
    
    # Check that all inner lists have the same length
    assert all(len(lst) == len(image_lists[0]) for lst in image_lists), "All inner lists must have the same length."
    
    # Number of rows and columns based on the number of plots
    num_plots = len(image_lists)
    ncols = math.ceil(math.sqrt(num_plots))  # Number of columns for grid
    nrows = math.ceil(num_plots / ncols)     # Number of rows for grid
    
    num_frames = len(image_lists[0])

    # Create figure and axes
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(5 * ncols, 5 * nrows))
    plt.subplots_adjust(bottom=0.2)

    # Flatten axes array for easier iteration
    axes = axes.flatten() if num_plots > 1 else [axes]

    # Store the display objects for each plot (either imshow or line plot)
    img_displays = []
    line_displays = []
    
    # Get max/min values for 1D arrays to set static axis limits
    max_values = [max(lst) if not isinstance(lst[0], np.ndarray) else None for lst in image_lists]
    min_values = [min(lst) if not isinstance(lst[0], np.ndarray) else None for lst in image_lists]

    for i, ax in enumerate(axes[:num_plots]):  # Only iterate over the number of plots
        # Check if the first item in the list is a 2D array (an image) or a scalar
        if isinstance(image_lists[i][0], np.ndarray) and image_lists[i][0].ndim == 2:
            # Use imshow for 2D data (images)
            img_display = ax.imshow(image_lists[i][0], cmap='viridis')
            img_displays.append(img_display)
            line_displays.append(None)  # Placeholder for line plots
            
            # Add colorbar if it's an image
            cbar = fig.colorbar(img_display, ax=ax)
            if cbar_labels and i < len(cbar_labels) and cbar_labels[i] is not None:
                cbar.set_label(cbar_labels[i])

        else:
            # Plot the list of scalar values up to the initial index
            line_display, = ax.plot(np.arange(len(image_lists[i])), image_lists[i], color='b')
            line_display.set_data(np.arange(1), image_lists[i][:1])  # Start with only the first value
            ax.set_xlim(0, len(image_lists[i]))  # Set x-axis to full length of the data
            ax.set_ylim(min_values[i], max_values[i])  # Set y-axis to cover the full range
            line_displays.append(line_display)
            img_displays.append(None)  # Placeholder for image plots

        # Set plot title if provided
        if plot_titles and i < len(plot_titles) and plot_titles[i] is not None:
            ax.set_title(plot_titles[i])

    # Remove any unused axes
    for ax in axes[num_plots:]:
        ax.remove()

    # Slider for selecting the frame index
    ax_slider = plt.axes([0.2, 0.05, 0.65, 0.03], facecolor='lightgoldenrodyellow')
    frame_slider = Slider(ax_slider, 'Frame', 0, num_frames - 1, valinit=0, valstep=1)

    # Update function for the slider
    def update(val):
        index = int(frame_slider.val)  # Get the selected index from the slider
        for i, (img_display, line_display) in enumerate(zip(img_displays, line_displays)):
            if img_display is not None:
                # Update the image data for 2D data
                img_display.set_data(image_lists[i][index])
            if line_display is not None:
                # Update the line plot for scalar values (plot up to the selected index)
                line_display.set_data(np.arange(index), image_lists[i][:index])
        fig.canvas.draw_idle()  # Redraw the figure

    # Connect the slider to the update function
    frame_slider.on_changed(update)

    plt.show()



def display_images_as_movie(image_lists, plot_titles=None, cbar_labels=None, save_path="output_movie.mp4", fps=5):
    """
    Creates an animation from multiple images or 1D plots from a list of lists and saves it as a movie.
    
    Parameters:
    - image_lists: list of lists where each inner list contains either 2D arrays (images) or 1D arrays (scalars).
                   The inner lists must all have the same length.
    - plot_titles: list of strings, one for each subplot. Default is None (no titles).
    - cbar_labels: list of strings, one for each colorbar. Default is None (no labels).
    - save_path: path where the output movie will be saved.
    - fps: frames per second for the output movie.
    """
    
    # Check that all inner lists have the same length
    assert all(len(lst) == len(image_lists[0]) for lst in image_lists), "All inner lists must have the same length."
    
    # Number of rows and columns based on the number of plots
    num_plots = len(image_lists)
    ncols = math.ceil(math.sqrt(num_plots))  # Number of columns for grid
    nrows = math.ceil(num_plots / ncols)     # Number of rows for grid
    
    num_frames = len(image_lists[0])

    # Create figure and axes
    fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(6 * ncols, 6 * nrows))
    plt.subplots_adjust(bottom=0.2)

    # Flatten axes array for easier iteration
    axes = axes.flatten() if num_plots > 1 else [axes]

    # Store the display objects for each plot (either imshow or line plot)
    img_displays = []
    line_displays = []
    
    # Get max/min values for 1D arrays to set static axis limits
    max_values = [max(lst) if not isinstance(lst[0], np.ndarray) else None for lst in image_lists]
    min_values = [min(lst) if not isinstance(lst[0], np.ndarray) else None for lst in image_lists]

    for i, ax in enumerate(axes[:num_plots]):  # Only iterate over the number of plots
        # Check if the first item in the list is a 2D array (an image) or a scalar
        if isinstance(image_lists[i][0], np.ndarray) and image_lists[i][0].ndim == 2:
            # Use imshow for 2D data (images)
            img_display = ax.imshow(image_lists[i][0], cmap='viridis')
            img_displays.append(img_display)
            line_displays.append(None)  # Placeholder for line plots
            
            # Add colorbar if it's an image
            cbar = fig.colorbar(img_display, ax=ax)
            if cbar_labels and i < len(cbar_labels) and cbar_labels[i] is not None:
                cbar.set_label(cbar_labels[i])

        else:
            # Plot the list of scalar values up to the initial index
            line_display, = ax.plot(np.arange(len(image_lists[i])), image_lists[i], color='b')
            line_display.set_data(np.arange(1), image_lists[i][:1])  # Start with only the first value
            ax.set_xlim(0, len(image_lists[i]))  # Set x-axis to full length of the data
            ax.set_ylim(min_values[i], max_values[i])  # Set y-axis to cover the full range
            line_displays.append(line_display)
            img_displays.append(None)  # Placeholder for image plots

        # Set plot title if provided
        if plot_titles and i < len(plot_titles) and plot_titles[i] is not None:
            ax.set_title(plot_titles[i])

    # Remove any unused axes
    for ax in axes[num_plots:]:
        ax.remove()

    # Function to update the frames
    def update_frame(frame_idx):
        for i, (img_display, line_display) in enumerate(zip(img_displays, line_displays)):
            if img_display is not None:
                # Update the image data for 2D data
                img_display.set_data(image_lists[i][frame_idx])
            if line_display is not None:
                # Update the line plot for scalar values (plot up to the current index)
                line_display.set_data(np.arange(frame_idx), image_lists[i][:frame_idx])
        return img_displays + line_displays

    # Create the animation
    ani = animation.FuncAnimation(fig, update_frame, frames=num_frames, blit=False, repeat=False)

    # Save the animation as a movie file
    ani.save(save_path, fps=fps, writer='ffmpeg')

    plt.show()


# Delete this version
# def nice_heatmap_subplots( im_list , xlabel_list, ylabel_list, title_list,cbar_label_list, fontsize=15, cbar_orientation = 'bottom', axis_off=True, vlims=None, savefig=None):

#     n = len(im_list)
#     fs = fontsize
#     fig = plt.figure(figsize=(5*n, 5))

#     for a in range(n) :
#         ax1 = fig.add_subplot(int(f'1{n}{a+1}'))
#         ax1.set_title(title_list[a] ,fontsize=fs)

#         if vlims!=None:
#             im1 = ax1.imshow(  im_list[a] , vmin = vlims[a][0], vmax = vlims[a][1])
#         else:
#             im1 = ax1.imshow(  im_list[a] )
#         ax1.set_title( title_list[a] ,fontsize=fs)
#         ax1.set_xlabel( xlabel_list[a] ,fontsize=fs) 
#         ax1.set_ylabel( ylabel_list[a] ,fontsize=fs) 
#         ax1.tick_params( labelsize=fs ) 

#         if axis_off:
#             ax1.axis('off')
#         divider = make_axes_locatable(ax1)
#         if cbar_orientation == 'bottom':
#             cax = divider.append_axes('bottom', size='5%', pad=0.05)
#             cbar = fig.colorbar( im1, cax=cax, orientation='horizontal')
                
#         elif cbar_orientation == 'top':
#             cax = divider.append_axes('top', size='5%', pad=0.05)
#             cbar = fig.colorbar( im1, cax=cax, orientation='horizontal')
                
#         else: # we put it on the right 
#             cax = divider.append_axes('right', size='5%', pad=0.05)
#             cbar = fig.colorbar( im1, cax=cax, orientation='vertical')  
        
   
#         cbar.set_label( cbar_label_list[a], rotation=0,fontsize=fs)
#         cbar.ax.tick_params(labelsize=fs)
#     if savefig!=None:
#         plt.savefig( savefig , bbox_inches='tight', dpi=300) 

#     #plt.show() 

def nice_DM_plot( data, savefig=None , include_actuator_number = True): #for a 140 actuator BMC 3.5 DM
    fig,ax = plt.subplots(1,1)
    if len( np.array(data).shape ) == 1: 
        ax.imshow( get_DM_command_in_2D(data) )
        n_rows, n_cols = get_DM_command_in_2D(data).shape
    else: 
        ax.imshow( data )
        n_rows, n_cols = data.shape
    #ax.set_title('poorly registered actuators')
    ax.grid(True, which='minor',axis='both', linestyle='-', color='k', lw=2 )
    ax.set_xticks( np.arange(12) - 0.5 , minor=True)
    ax.set_yticks( np.arange(12) - 0.5 , minor=True)

    if include_actuator_number:
        
        for i in range(n_rows):
            for j in range(n_cols):
                # Calculate the flattened index
                flattened_index = i * n_cols + j
                # Add the flattened index as text in the plot
                ax.text(j, i, f'{flattened_index}', va='center', ha='center', color='white')

    if savefig!=None:
        plt.savefig( savefig , bbox_inches='tight', dpi=300) 




def truncated_pseudoinverse(U, s, Vt, k):
    """
    Compute the pseudoinverse of a matrix using a truncated SVD.

    Parameters:
        U (np.ndarray): Left singular vectors (m x m if full_matrices=True)
        s (np.ndarray): Singular values (vector of length min(m,n))
        Vt (np.ndarray): Right singular vectors (n x n if full_matrices=True)
        k (int): Number of singular values/modes to keep.

    Returns:
        np.ndarray: The truncated pseudoinverse of the original matrix.
    """
    # Keep only the first k modes
    U_k = U[:, :k]      # shape: (m, k)
    s_k = s[:k]         # shape: (k,)
    Vt_k = Vt[:k, :]    # shape: (k, n)

    # Build the inverse of the diagonal matrix with the truncated singular values
    S_inv_k = np.diag(1.0 / s_k)  # shape: (k, k)

    # Compute the truncated pseudoinverse
    IM_trunc_inv = Vt_k.T @ S_inv_k @ U_k.T
    return IM_trunc_inv





def project_matrix( CM , projection_vector_list ):
    """
    create two new matrices CM_TT, and CM_HO from CM, 
    where CM_TT projects any "signal" onto the column space of vectors in 
    projection_vector_list vectors, 
    and CM_HO which projects any "signal" to the null space of CM_TT that is within CM.
    
    Typical use is to seperate control matrix to tip/tilt reconstructor (CM_TT) and
    higher order reconstructor (CM_HO)

    Note vectors in projection_vector_list  must be 
    
    Parameters
    ----------
    CM : TYPE matrix
        DESCRIPTION. 
    projection_vector_list : list of vectors that are in col space of CM
        DESCRIPTION.

    Returns
    -------
    CM_TT , CM_HO

    """
    
    # Create the matrix T from projection_vector_list (e.g. tip and tilt vectors )
    projection_vector_list
    T = np.column_stack( projection_vector_list )  # T is Mx2
    
    # Calculate the projection matrix P
    #P = T @ np.linalg.inv(T.T @ T) @ T.T  # P is MxM <- also works like this 
    # Compute SVD of T (this way is more numerically stable)
    U, S, Vt = np.linalg.svd(T, full_matrices=False)
    
    #  Compute the projection matrix P using SVD
    P = U @ U.T  # U @ U.T gives the projection matrix onto the column space of T
    
    #  Compute CM_TT (projection onto tip and tilt space)
    CM_TT = P @ CM  # CM_TT is MxN
    
    #  Compute the null space projection matrix and CM_HO
    I = np.eye(T.shape[0])  # Identity matrix of size MxM
    CM_HO = (I - P) @ CM  # CM_HO is MxN

    return( CM_TT , CM_HO )

# ==========

def test_controller_in_cmd_space( zwfs, phase_controller, Vw =None , D=None , AO_lag=None  ):
    #outputs fits file with telemetry  
    # applies open loop for X iterations, Closed loop for Y iterations
    # ability to ramp number of controlled modes , how to deal with gains? gain = 1/number controlled modes for Zernike basis

    #open loop 
    for i in range(X) : 

        for i in range( 3 ) :
            raw_img_list.append( zwfs.get_image() ) # @D, remember for control_phase method this needs to be flattened and filtered for pupil region

        raw_img.append( np.median( raw_img_list, axis = 0 ) ) 

        # ALSO MANUALLY GET PSF IMAGE 
        
        err_img.append(  phase_controller.get_img_err( raw_img[-1].reshape(-1)[zwfs.pupil_pixels]  ) )

        reco_modes.append( phase_controller.control_phase( err_img[-1]  , controller_name = ctrl_method_label) )

        reco_dm_cmds.append( phase_controller.config['M2C'] @ mode_reco[-1] )

        delta_cmd.append( Ki * delta_cmd[-1] - Kp * reco_dm_cmds[-1] )

        # propagate_phase_screen one step 

        # don't yet apply the correction cmd while in open loop
        zwfs.dm.send_data( zwfs.dm_shapes['flat_dm'] + dist[-1] ) 

        time.sleep(0.1)


    #close loop 
    for i in range(X) : 

        for i in range( 3 ) :
            raw_img_list.append( zwfs.get_image() ) # @D, remember for control_phase method this needs to be flattened and filtered for pupil region

        raw_img.append( np.median( raw_img_list, axis = 0 ) ) 

        err_img.append(  phase_controller.get_img_err( raw_img[-1].reshape(-1)[zwfs.pupil_pixels]  ) )

        reco_modes.append( phase_controller.control_phase( err_img[-1]  , controller_name = ctrl_method_label) )

        reco_dm_cmds.append( phase_controller.config['M2C'] @ mode_reco[-1] )

        delta_cmd.append( Ki * delta_cmd[-1] - Kp * reco_dm_cmds[-1] )

        # propagate_phase_screen one step 

        # then apply the correction 
        zwfs.dm.send_data( zwfs.dm_shapes['flat_dm'] + delta_cmd[-1] + dist[-1] )

        time.sleep(0.1)





def apply_sequence_to_DM_and_record_images(zwfs, DM_command_sequence, number_images_recorded_per_cmd = 5, take_mean_of_images=False, save_dm_cmds = True, calibration_dict=None, additional_header_labels=None, sleeptime_between_commands=0.01, cropping_corners=None, save_fits = None):
    """
    

    Parameters
    ----------

    zwfs is a zwfs object from the ZWFS class that must have zwfs.dm and zwfs.camera attributes:
        dm : TYPE
            DESCRIPTION. DM Object from BMC SDK. Initialized  (this is zwfs.camera object) 
        camera : TYPE camera objection from FLI SDK. (this is zwfs.camera object)
            DESCRIPTION. Camera context from FLISDK. Initialized from context = FliSdk_V2.Init(), 

    DM_command_sequence : TYPE list 
        DESCRIPTION. Nc x Na matrix where Nc is number of commands to send in sequence (rows)
        Na is number actuators on DM (columns).   
    number_images_recorded_per_cmd : TYPE, optional
        DESCRIPTION. The default is 1. puting a value >= 0 means no images are recorded.
    take_mean_of_images: TYPE, optional
        DESCRIPTION. The default is False. if True we take the median image of number_images_recorded_per_cmd such that there is only one image per command (that is the aggregated image)
    calibration_dict: TYPE, optional
        DESCRIPTION. The default is None meaning saved images don't get flat fielded. 
        if flat fielding is required a dictionary must be supplied that contains 
        a bias, dark and flat frame under keys 'bias', 'dark', and 'flat' respectively
    additional_header_labels : TYPE, optional
        DESCRIPTION. The default is None which means no additional header is appended to fits file 
        otherwise a tuple (header, value) or list of tuples [(header_0, value_0)...] can be used. 
        If list, each item in list will be added as a header. 
    cropping_corners: TYPE, optional
        DESCRIPTION. list of length 4 holding [row min, row max, col min, col max] to crop raw data frames.
    save_fits : TYPE, optional
        DESCRIPTION. The default is None which means images are not saved, 
        if a string is provided images will be saved with that name in the current directory
    sleeptime_between_commands : TYPE, optional float
        DESCRIPTION. time to sleep between sending DM commands and recording images in seconds. default is 0.01s
    Returns
    -------
    fits file with images corresponding to each DM command in sequence
    first extension is images
    second extension is DM commands

    """

    
    should_we_record_images = True
    try: # foce to integer
        number_images_recorded_per_cmd = int(number_images_recorded_per_cmd)
        if number_images_recorded_per_cmd <= 0:
            should_we_record_images = False
    except:
        raise TypeError('cannot convert "number_images_recorded_per_cmd" to a integer. Check input type')
    
    image_list = [] #init list to hold images

    # NOTE THE CAMERA SHOULD ALREADY BE STARTED BEFORE BEGINNING - No checking here yet
    for cmd_indx, cmd in enumerate(DM_command_sequence):
        print(f'executing cmd_indx {cmd_indx} / {len(DM_command_sequence)}')
        # wait a sec        
        time.sleep(sleeptime_between_commands)
        # ok, now apply command
        zwfs.dm.send_data(cmd)
        # wait a sec        
        time.sleep(sleeptime_between_commands)

        if should_we_record_images: 
            if take_mean_of_images:
                ims_tmp = [ np.mean( zwfs.get_some_frames(number_of_frames = number_images_recorded_per_cmd, apply_manual_reduction = True ) ,axis=0) ] #[np.median([zwfs.get_image() for _ in range(number_images_recorded_per_cmd)] , axis=0)] #keep as list so it is the same type as when take_mean_of_images=False
            else:
                ims_tmp = [ zwfs.get_image() ] #get_raw_images(camera, number_images_recorded_per_cmd, cropping_corners) 
            image_list.append( ims_tmp )

    
    #FliSdk_V2.Stop(camera) # stop camera
    
    # init fits files if necessary
    if should_we_record_images: 
        #cmd2pix_registration
        data = fits.HDUList([]) #init main fits file to append things to
        
        # Camera data
        cam_fits = fits.PrimaryHDU( image_list )
        
        cam_fits.header.set('EXTNAME', 'SEQUENCE_IMGS' )
        #camera headers
        camera_info_dict = get_camera_info(zwfs.camera)
        for k,v in camera_info_dict.items():
            cam_fits.header.set(k,v)
        cam_fits.header.set('#images per DM command', number_images_recorded_per_cmd )
        cam_fits.header.set('take_mean_of_images', take_mean_of_images )
        
        cam_fits.header.set('cropping_corners_r1', zwfs.pupil_crop_region[0] )
        cam_fits.header.set('cropping_corners_r2', zwfs.pupil_crop_region[1] )
        cam_fits.header.set('cropping_corners_c1', zwfs.pupil_crop_region[2] )
        cam_fits.header.set('cropping_corners_c2', zwfs.pupil_crop_region[3] )

        #if user specifies additional headers using additional_header_labels
        if (additional_header_labels!=None): 
            if type(additional_header_labels)==list:
                for i,h in enumerate(additional_header_labels):
                    cam_fits.header.set(h[0],h[1])
            else:
                cam_fits.header.set(additional_header_labels[0],additional_header_labels[1])

        # add camera data to main fits
        data.append(cam_fits)
        
        if save_dm_cmds:
            # put commands in fits format
            dm_fits = fits.PrimaryHDU( DM_command_sequence )
            #DM headers 
            dm_fits.header.set('timestamp', str(datetime.datetime.now()) )
            dm_fits.header.set('EXTNAME', 'DM_CMD_SEQUENCE' )
            #dm_fits.header.set('DM', DM.... )
            #dm_fits.header.set('#actuators', DM.... )

            # append to the data
            data.append(dm_fits)
        
        if save_fits!=None:
            if type(save_fits)==str:
                data.writeto(save_fits)
            else:
                raise TypeError('save_images needs to be either None or a string indicating where to save file')
            
            
        return(data)
    
    else:
        return(None)



def get_camera_info(camera):
    if FliSdk_V2.IsCredOne(camera):
        cred = FliCredOne.FliCredOne() #cred1 object 

    elif FliSdk_V2.IsCredTwo(camera):
        cred = FliCredTwo.FliCredTwo() #cred3 object 
    
    elif FliSdk_V2.IsCredThree(camera):
        cred = FliCredThree.FliCredThree() #cred3 object 
    camera_info_dict = {} 
    
    # cropping rows 
    _, cropping_rows = FliSdk_V2.FliSerialCamera.SendCommand(camera, "cropping rows")
    _, cropping_columns =  FliSdk_V2.FliSerialCamera.SendCommand(camera, "cropping columns")

    # query camera settings 
    fps_res, fps_response = FliSdk_V2.FliSerialCamera.GetFps(camera)
    tint_res, tint_response = FliSdk_V2.FliSerialCamera.SendCommand(camera, "tint raw")
  
    # gain
    #gain = cred.GetConversionGain(camera)[1]
    _, gain = FliSdk_V2.FliSerialCamera.SendCommand(camera, "sensibility") #NOTE OLD FIRMWARE, NEWER USES sensitivity

    #camera headers
    camera_info_dict['timestamp'] = str(datetime.datetime.now()) 
    camera_info_dict['camera'] = FliSdk_V2.GetCurrentCameraName(camera) 
    camera_info_dict['camera_fps'] = fps_response
    camera_info_dict['camera_tint'] = tint_response
    camera_info_dict['camera_gain'] = gain.split(':')[-1]
    camera_info_dict['cropping_rows'] = cropping_rows
    camera_info_dict['cropping_columns'] = cropping_columns

    return(camera_info_dict)

    
def scan_detector_framerates(zwfs, frame_rates, number_images_recorded_per_cmd = 50, cropping_corners=None, save_fits = None): 
    """
    iterate through different camera frame rates and record a series of images for each
    this can be used for building darks or flats.

    Parameters
    ----------
    zwfs : TYPE zwfs object holding camera object from FLI SDK.
        DESCRIPTION. Camera context from FLISDK. Initialized from context = FliSdk_V2.Init() 
    frame_rates : TYPE list like 
        DESCRIPTION. array holding different frame rates to iterate through
    number_images_recorded_per_cmd : TYPE, optional
        DESCRIPTION. The default is 50. puting a value >= 0 means no images are recorded.
    save_fits : TYPE, optional
        DESCRIPTION. The default is None which means images are not saved, 
        if a string is provided images will be saved with that name in the current directory

    Raises
    ------
    TypeError
        DESCRIPTION.

    Returns
    -------
    fits file with each extension corresponding to a different camera frame rate 

    """
    
    #zwfs.set_camera_fps(600) 
    zwfs.start_camera() # start camera

    data = fits.HDUList([]) 
    for fps in frame_rates:
        
        zwfs.set_camera_fps(fps) # set max dit (tint=None) for given fps
	
        time.sleep(1) # wait 1 second
        #tmp_fits = fits.PrimaryHDU( [FliSdk_V2.GetProcessedImageGrayscale16bNumpyArray(camera,-1)  for i in range(number_images_recorded_per_cmd)] )
        img_list = []
        for _ in range(number_images_recorded_per_cmd):
            img_list.append( zwfs.get_image() )
            time.sleep(0.01)

        tmp_fits = fits.PrimaryHDU(  img_list )

        
        camera_info_dict = get_camera_info(zwfs.camera)
        for k,v in camera_info_dict.items():
            tmp_fits.header.set(k,v)     

        data.append( tmp_fits )

    zwfs.stop_camera()  # stop camera
    
    if save_fits!=None:
        if type(save_fits)==str:
            data.writeto(save_fits)
        else:
            raise TypeError('save_images needs to be either None or a string indicating where to save file')
        
    return(data)




def GET_BDR_RECON_DATA_INTERNAL(zwfs,  number_amp_samples = 18, amp_max = 0.2, number_images_recorded_per_cmd = 10, source_selector = None,save_fits = None) :
    """
    source_selector is motor to move light source for bias frame, if None we ask to manually move it
    """
    flat_dm_cmd = zwfs.dm_shapes['flat_dm']

    modal_basis = np.eye(len(flat_dm_cmd))
    cp_x1,cp_x2,cp_y1,cp_y2 = zwfs.pupil_crop_region

    ramp_values = np.linspace(-amp_max, amp_max, number_amp_samples)

    # ======== reference image with FPM OUT

    zwfs.dm.send_data(flat_dm_cmd) 
    #_ = input('MANUALLY MOVE PHASE MASK OUT OF BEAM, PRESS ENTER TO BEGIN' )
    #watch_camera(zwfs, frames_to_watch = 70, time_between_frames=0.05)

    # fourier tip to go off phase mask 
    fourier_basis = construct_command_basis( basis='fourier', number_of_modes = 40, Nx_act_DM = 12, Nx_act_basis = 12, act_offset=(0,0), without_piston=True)
    tip = fourier_basis[:,0]
    print( 'applying 2*tip cmd in Fourier basis to go off phase mask')
    zwfs.dm.send_data( 0.5 + 2*tip ) 
    time.sleep(0.1)
    N0_list = zwfs.get_some_frames(number_of_frames = number_images_recorded_per_cmd, apply_manual_reduction = True )
    #for _ in range(number_images_recorded_per_cmd):
    #    N0_list.append( zwfs.get_image( ) ) #REFERENCE INTENSITY WITH FPM IN
    N0 = np.mean( N0_list, axis = 0 ) 

    #make_fits

    # ======== reference image with FPM IN
    #_ = input('MANUALLY MOVE PHASE MASK BACK IN, PRESS ENTER TO BEGIN' )
    #watch_camera(zwfs, frames_to_watch = 70, time_between_frames=0.05)
    print( 'going back to DM flat to put beam ON phase mask')
    zwfs.dm.send_data(flat_dm_cmd) 
    time.sleep(0.1)
    I0_list = zwfs.get_some_frames(number_of_frames = number_images_recorded_per_cmd, apply_manual_reduction = True ) 
    #for _ in range(number_images_recorded_per_cmd):
    #    I0_list.append( zwfs.get_image(  ) ) #REFERENCE INTENSITY WITH FPM IN
    I0 = np.mean( I0_list, axis = 0 ) 

    # ======== BIAS FRAME
    if source_selector == None:
        _ = input('COVER THE DETECTOR FOR A BIAS FRAME OR TURN OFF SOURCE, PRESS ENTER ONCE READ' )
    else: # we assume you gave us the sourcemotor object for moving the source.
        # get the name of the source currently used:
        current_source_name = source_selector.current_position
        # move this source out to none
        source_selector.set_source(  'none' ) 
        print('Moving source out to get bias frame...')
    #watch_camera(zwfs, frames_to_watch = 50, time_between_frames=0.05)

    BIAS_list = []
    for _ in range(100):
        time.sleep(0.05)
        BIAS_list.append( np.mean(zwfs.get_some_frames(number_of_frames = number_images_recorded_per_cmd, apply_manual_reduction = True ),axis=0 ) ) #REFERENCE INTENSITY WITH FPM IN
    #I0 = np.median( I0_list, axis = 0 ) 

    if source_selector == None:    
        _ = input('PUT SOURCE BACK ON CAMERA, PRESS ENTER ONCE DONE' )
    else:
        source_selector.set_source( current_source_name )
        print('moving source back in')

    
    #====== make references fits files
    I0_fits = fits.PrimaryHDU( I0 )
    N0_fits = fits.PrimaryHDU( N0 )
    BIAS_fits = fits.PrimaryHDU( BIAS_list )
    I0_fits.header.set('EXTNAME','FPM_IN')
    N0_fits.header.set('EXTNAME','FPM_OUT')
    BIAS_fits.header.set('EXTNAME','BIAS')

    flat_DM_fits = fits.PrimaryHDU( flat_dm_cmd )
    flat_DM_fits.header.set('EXTNAME','FLAT_DM_CMD')

    _ = input('PRESS ENTER WHEN READY TO BUILD IM' )

    #make_fits

    # ======== RAMPING ACTUATORS  
    # --- creating sequence of dm commands
    _DM_command_sequence = [list(flat_dm_cmd + amp * modal_basis) for amp in ramp_values ]  
    # add in flat dm command at beginning of sequence and reshape so that cmd sequence is
    # [0, a0*b0,.. aN*b0, a0*b1,...,aN*b1, ..., a0*bM,...,aN*bM]
    DM_command_sequence = [flat_dm_cmd] + list( np.array(_DM_command_sequence).reshape(number_amp_samples*modal_basis.shape[0],modal_basis.shape[1] ) )

    # --- additional labels to append to fits file to keep information about the sequence applied 
    additional_labels = [('cp_x1',cp_x1),('cp_x2',cp_x2),('cp_y1',cp_y1),('cp_y2',cp_y2),('in-poke max amp', np.max(ramp_values)),('out-poke max amp', np.min(ramp_values)),('#ramp steps',number_amp_samples), ('seq0','flatdm'), ('reshape',f'{number_amp_samples}-{modal_basis.shape[0]}-{modal_basis.shape[1]}'),('Nmodes_poked',len(modal_basis)),('Nact',140)]

    # --- poke DM in and out and record data. Extension 0 corresponds to images, extension 1 corresponds to DM commands
    raw_recon_data = apply_sequence_to_DM_and_record_images(zwfs, DM_command_sequence, number_images_recorded_per_cmd = number_images_recorded_per_cmd, take_mean_of_images=True, save_dm_cmds = True, calibration_dict=None, additional_header_labels = additional_labels,sleeptime_between_commands=0.03, cropping_corners=None,  save_fits = None ) # None

    zwfs.dm.send_data(flat_dm_cmd) 

    # append FPM IN and OUT references (note FPM in reference is also first entry in recon_data so we can compare if we want!) 
    raw_recon_data.append( I0_fits ) 
    raw_recon_data.append( N0_fits ) 
    raw_recon_data.append( BIAS_fits )
    raw_recon_data.append( flat_DM_fits )

    if save_fits != None:  
        if type(save_fits)==str:
            raw_recon_data.writeto(save_fits)
        else:
            raise TypeError('save_images needs to be either None or a string indicating where to save file')


    return(raw_recon_data) 


def Ic_model_constrained(x, A, B, F, mu):
    penalty = 0
    if (F < 0) or (mu < 0): # F and mu can be correlated so constrain the quadrants 
        penalty = 1e3
    I = A + B * np.cos(F * x + mu) + penalty
    return I 

def Ic_model_constrained_3param(x, A,  F, mu): 
    #penalty = 0
    #if (F < 0) or (mu < 0): # F and mu can be correlated so constrain the quadrants 
    #    penalty = 1e3
    # 
    I = A**2 + 2 * A * np.cos(F * x + mu) + penalty
    return I 


# should this be free standing or a method? ZWFS? controller? - output a report / fits file
def PROCESS_BDR_RECON_DATA_INTERNAL(recon_data, bad_pixels = ([],[]), active_dm_actuator_filter=None, poke_amplitude_indx=3, debug=True, fig_path = 'tmp/', savefits=None) :
    """
    # calibration of our ZWFS: 
    # this will fit M0, b0, mu, F which can be appended to a phase_controller,
    # f and/or mu can be be provided (because may haave been previously characterise) in which case only b0/M0 is fitted.   
    # also P2C_1x1, P2C_3x3 matrix which corresponds to 1x1 and 3x3 pixels around where peak influence for an actuator was found. 
    # Also pupil regions to filter 
       - active pupil pixels 
       - secondary obstruction pixels
       - outside pupil pixels 

    # plot fits, histograms of values (corner plot?), also image highlighting the regions (coloring pixels) 
    # estimate center of DM in pixels and center of pupil in pixels (using previous method)   
    # note A^2 can be measured with FPM out, M can be sampled with FPM in where A^2 = 0. 
    
    # e.g. to generate bad pixel tuple in correct format
    # np.where( (np.std( poke_imgs ,axis = (0,1)) > 100) + (np.std( poke_imgs ,axis = (0,1)) == 0 ) )
    """    


    # ========================== !! 0 !! =====================
    # -- prelims of reading in and labelling data 
    tstamp = datetime.datetime.now().strftime("%d-%m-%YT%H.%M.%S")

    # poke values used in linear ramp
    No_ramps = int(recon_data['SEQUENCE_IMGS'].header['#ramp steps'])
    max_ramp = float( recon_data['SEQUENCE_IMGS'].header['in-poke max amp'] )
    min_ramp = float( recon_data['SEQUENCE_IMGS'].header['out-poke max amp'] ) 
    ramp_values = np.linspace( min_ramp, max_ramp, No_ramps)

    flat_dm_cmd = recon_data['FLAT_DM_CMD'].data

    Nmodes_poked = int(recon_data[0].header['HIERARCH Nmodes_poked']) # can also see recon_data[0].header['RESHAPE']

    Nact =  int(recon_data[0].header['HIERARCH Nact'])  

    N0 = recon_data['FPM_OUT'].data
    #P = np.sqrt( pupil ) # 
    I0 = recon_data['FPM_IN'].data

    # the first image is another reference I0 with FPM IN and flat DM
    poke_imgs = recon_data['SEQUENCE_IMGS'].data[1:].reshape(No_ramps, 140, I0.shape[0], I0.shape[1])
    #poke_imgs = poke_imgs[1:].reshape(No_ramps, 140, I0.shape[0], I0.shape[1])

    plt.figure( ) 
    plt.imshow( np.std( poke_imgs ,axis = (0,1)) )
    plt.colorbar(label = 'std pixels') 
    
    recomended_bad_pixels = np.where( (np.std( poke_imgs ,axis = (0,1)) > 100) + (np.std( poke_imgs ,axis = (0,1)) == 0 ))
    print('recommended bad pixels (high or zero std) at :',recomended_bad_pixels )
    
    if len(bad_pixels[0]) > 0:
        
        bad_pixel_mask = np.ones(I0.shape)
        for ibad,jbad in list(zip(bad_pixels[0], bad_pixels[1])):
            bad_pixel_mask[ibad,jbad] = 0
            
        I0 *= bad_pixel_mask
        N0 *= bad_pixel_mask
        poke_imgs  = poke_imgs * bad_pixel_mask

    a0 = len(ramp_values)//2 - poke_amplitude_indx # which poke value (index) do we want to consider for finding region of influence. Pick a value near the center of the ramp (ramp values are from negative to positive) where we are in a linear regime.
    
    if hasattr(active_dm_actuator_filter,'__len__'):
        #is it some form of boolean type?
        bool_test = ( (all(isinstance(x,np.bool_) for x in active_dm_actuator_filter)) or (all(isinstance(x,bool) for x in active_dm_actuator_filter) ) )
        #is it the right length (corresponds to number of DM actuators?) 
        len_test = (len( active_dm_actuator_filter ) == Nact)

        if len_test & bool_test:
            dm_pupil_filt = np.array(active_dm_actuator_filter ) # force to numpy array
        else:
            raise TypeError('active_dm_actuator_filter needs to be list like with boolean entries (numpy or naitive) with length = 140 (corresponding to number of actuators on DM')


    elif active_dm_actuator_filter==None:
        # ========================== !! 1 !! =====================
        #  == Then we let the user define the region of influence on DM where we will fit our models (by defining a threshold for I(epsilon)_max -I_0). This is important because the quality of the fits can go to shit around pupil/DM edges, we only need some good samples around the center to reconstruct what we need, setting this threshold is semi automated here  

        fig,ax= plt.subplots( 4, 4, figsize=(10,10))
        num_pixels = []
        candidate_thresholds = np.linspace(4 * np.std(abs(poke_imgs[a0,:,:,:] - I0)),np.max(abs(poke_imgs[a0,:,:,:] - I0)),16)
        for axx, thresh in zip(ax.reshape(-1),candidate_thresholds):
    
            dm_pupil_filt = thresh < np.array( [np.max( abs( poke_imgs[a0][act] - I0) ) for act in range(140)] ) 
            axx.imshow( get_DM_command_in_2D( dm_pupil_filt ) ) 
            axx.set_title('threshold = {}'.format(round( thresh )),fontsize=12) 
            axx.axis('off')
            num_pixels.append(sum(dm_pupil_filt)) 
            # we could use this to automate threshold decision.. look for where 
            # d num_pixels/ d threshold ~ 0.. np.argmin( abs( np.diff( num_pixels ) )[:10])
        plt.show()

        recommended_threshold = candidate_thresholds[np.argmin( abs( np.diff( num_pixels ) )[2:11]) + 1 ]
        print( f'\n\nrecommended threshold ~ {round(recommended_threshold)} \n(check this makes sense with the graph by checking the colored area is stable around changes in threshold about this value)\n\n')

        pupil_filt_threshold = float(input('input threshold of peak differences'))

        ### <---- THIS FILTER DETERMINES WHERE WE FIT THE MODELS (ONLY FIT WHERE DM HAS GOOD INFLUENCE!)
        dm_pupil_filt = pupil_filt_threshold < np.array( [np.max( abs( poke_imgs[a0][act] - I0) ) for act in range(Nact)] ) 

        if debug:
           plt.figure()
           plt.imshow( get_DM_command_in_2D( dm_pupil_filt ) )
           plt.title('influence region on DM where we will fit intensity models per actuator')
           plt.show()
           plt.savefig(  fig_path + f'process_fits_0_{tstamp}.png', bbox_inches='tight', dpi=300)


      
    # ========================== !! 2 !! =====================
    # ======== P2C 

    Sw_x, Sw_y = 3,3 #+- pixels taken around region of peak influence. PICK ODD NUMBERS SO WELL CENTERED!   
    act_img_mask_1x1 = {} #pixel with peak sensitivity to the actuator
    act_img_mask_3x3 = {} # 3x3 region around pixel with peak sensitivity to the actuator
    poor_registration_list = np.zeros(Nact).astype(bool) # list of actuators in control region that have poor registration 


    # I should probably include a threshold filter here - that no registration is made if delta < threshold
    # threshold set to 5 sigma above background (seems to work - but can be tweaked) 
    registration_threshold = 5 * np.mean(np.std(abs(poke_imgs- I0),axis=(0,1)) )
    # how to best deal with actuators that have poor registration ?
    for act_idx in range(Nact):
        delta =  poke_imgs[a0][act_idx] - I0

        mask_3x3 = np.zeros( I0.shape )
        mask_1x1 = np.zeros( I0.shape )
        if dm_pupil_filt[act_idx]: #  if we decided actuator has strong influence on ZWFS image, we 
            peak_delta = np.max( abs(delta) ) 
            if peak_delta > registration_threshold:

                i,j = np.unravel_index( np.argmax( abs(delta) ), I0.shape )
          
                mask_3x3[i-Sw_x-1: i+Sw_x, j-Sw_y-1:j+Sw_y] = 1 # keep centered, 
                mask_1x1[i,j] = 1 
                #mask *= 1/np.sum(mask[i-Sw_x-1: i+Sw_x, j-Sw_y-1:j+Sw_y]) #normalize by #pixels in window 
                act_img_mask_3x3[act_idx] = mask_3x3
                act_img_mask_1x1[act_idx] = mask_1x1
            else:
                poor_registration_list[act_idx] = True
                act_img_mask_3x3[act_idx] = mask_3x3 
                act_img_mask_1x1[act_idx] = mask_1x1 
        else :
            act_img_mask_3x3[act_idx] = mask_3x3 
            act_img_mask_1x1[act_idx] = mask_1x1 
            #act_flag[act_idx] = 0 
    if debug:
        plt.figure()
        plt.title('masked regions of influence per actuator')
        plt.imshow( np.sum( list(act_img_mask_3x3.values()), axis = 0 ) )
        #plt.show()
        plt.savefig(  fig_path + f'process_fits_1_{tstamp}.png', bbox_inches='tight', dpi=300)

    # turn our dictionary to a big pixel to command matrix 
    P2C_1x1 = np.array([list(act_img_mask_1x1[act_idx].reshape(-1)) for act_idx in range(Nact)])
    P2C_3x3 = np.array([list(act_img_mask_3x3[act_idx].reshape(-1)) for act_idx in range(Nact)])

    # we can look at filtering a particular actuator in image P2C_3x3[i].reshape(zwfs.get_image().shape)
    # we can also interpolate signals in 3x3 grid to if DM actuator not perfectly registered to center of pixel. 
    

    if debug: 
        im_list = [(I0 - N0) / N0 ]
        xlabel_list = ['x [pixels]']
        ylabel_list = ['y [pixels]']
        title_list = ['']
        cbar_label_list = [r'$\frac{|\psi_C|^2 - |\psi_A|^2}{|\psi_A|^2}$']
        savefig = None# fig_path + f'pupil_FPM_IN-OUT_readout_mode-FULL_t{tstamp}.png'

        nice_heatmap_subplots( im_list , xlabel_list, ylabel_list, title_list, cbar_label_list, fontsize=15, axis_off=True, cbar_orientation = 'bottom', savefig=savefig)

        # check the active region 
        fig,ax = plt.subplots(1,1)
        ax.imshow( get_DM_command_in_2D( dm_pupil_filt  ) )
        ax.set_title('active DM region')
        ax.grid(True, which='minor',axis='both', linestyle='-', color='k' ,lw=3)
        ax.set_xticks( np.arange(12) - 0.5 , minor=True)
        ax.set_yticks( np.arange(12) - 0.5 , minor=True)

        plt.savefig(  fig_path + f'process_fits_2_{tstamp}.png', bbox_inches='tight', dpi=300)
        #plt.savefig( fig_path + f'active_DM_region_{tstamp}.png' , bbox_inches='tight', dpi=300) 
        # check the well registered DM region : 
        fig,ax = plt.subplots(1,1)
        ax.imshow( get_DM_command_in_2D( np.sum( P2C_1x1, axis=1 )))
        ax.set_title('well registered actuators')
        ax.grid(True, which='minor',axis='both', linestyle='-', color='k',lw=2 )
        ax.set_xticks( np.arange(12) - 0.5 , minor=True)
        ax.set_yticks( np.arange(12) - 0.5 , minor=True)

        plt.savefig(  fig_path + f'process_fits_3_{tstamp}.png', bbox_inches='tight', dpi=300)
  
        #plt.savefig( fig_path + f'poorly_registered_actuators_{tstamp}.png' , bbox_inches='tight', dpi=300) 

        # check poorly registered actuators: 
        fig,ax = plt.subplots(1,1)
        ax.imshow( get_DM_command_in_2D(poor_registration_list) )
        ax.set_title('poorly registered actuators')
        ax.grid(True, which='minor',axis='both', linestyle='-', color='k', lw=2 )
        ax.set_xticks( np.arange(12) - 0.5 , minor=True)
        ax.set_yticks( np.arange(12) - 0.5 , minor=True)

        plt.savefig(  fig_path + f'process_fits_4_{tstamp}.png', bbox_inches='tight', dpi=300)

        plt.show() 


# ========================== !! 3 !! =====================
    # ======== FITTING 
    # what do we fit ( I - N0 ) / N0 


    param_dict = {}
    cov_dict = {}
    fit_residuals = []
    nofit_list = []
    if debug:
        Nrows = np.ceil( sum( dm_pupil_filt )**0.5).astype(int)
        fig,ax = plt.subplots(Nrows,Nrows,figsize=(20,20))
        axx = ax.reshape(-1)
        for aaa in axx:
            aaa.axis('off')
        j=0 #axx index

    #mean_filtered_pupil = 1/(3*3)  * np.mean( P2C_3x3 @ N0.reshape(-1) )

    for act_idx in range(len(flat_dm_cmd)): 

        #Note that if we use the P2C_3x3 we need to normalize it 1/(3*3) * P2C_3x3
        if dm_pupil_filt[act_idx] * ( ~poor_registration_list)[act_idx]:

            # -- we do this with matrix multiplication using  mask_matrix
            #P_i = np.sum( act_img_mask[act_idx] * pupil ) #Flat DM with FPM OUT 
            #P_i = mean_filtered_pupil.copy() # just consider mean pupil! 
        
            I_i = np.array( [P2C_1x1[act_idx] @ poke_imgs[i][act_idx].reshape(-1) for i in  range(len(ramp_values))] ) #np.array( [np.sum( act_img_mask[act_idx] * poke_imgs[i][act_idx] ) for i in range(len(ramp_values))] ) #spatially filtered sum of intensities per actuator cmds 
            I_0 = P2C_1x1[act_idx] @ I0.reshape(-1) #np.sum( act_img_mask[act_idx] * I0 ) # Flat DM with FPM IN  
            N_0 = P2C_1x1[act_idx] @ N0.reshape(-1) #np.sum( act_img_mask[act_idx] * N0 )
            # ================================
            #   THIS IS OUR MODEL!S=A+B*cos(F*x + mu)  
            #S = (I_i - I_0) / P_i # signal to fit!
            S = I_i  # A 
            #S = (I_i - N_0) / N_0 # signal to fit! <- should I take mean of total pupil?? 
            # THEORETICALLY THIS SIGNAL IS: S = |M0|^2/|A|^2 + M0/|A| * cos(F.c + mu)  
            # ================================

            #re-label and filter to capture best linear range 
            x_data = ramp_values[2:-2].copy()
            y_data = S[2:-2].copy()

            initial_guess = [np.mean(S), (np.max(S)-np.min(S))/2,  15, 2.4]
            #initial_guess = [7, 2, 15, 2.4] #[0.5, 0.5, 15, 2.4]  #A_opt, B_opt, F_opt, mu_opt  ( S = A+B*cos(F*x + mu) )

            try:
                # FIT 
                popt, pcov = curve_fit(Ic_model_constrained, x_data, y_data, p0=initial_guess)

                # Extract the optimized parameters explictly to measure residuals
                A_opt, B_opt, F_opt, mu_opt = popt

                # STORE FITS 
                param_dict[act_idx] = popt
                cov_dict[act_idx] = pcov 
                # also record fit residuals 
                fit_residuals.append( S - Ic_model_constrained(ramp_values, A_opt, B_opt, F_opt, mu_opt) )


                if debug: 

                    axx[j].plot( ramp_values, Ic_model_constrained(ramp_values, A_opt, B_opt, F_opt, mu_opt) ,label=f'fit (act{act_idx})') 
                    axx[j].plot( ramp_values, S ,label=f'measured (act{act_idx})' )
                    #axx[j].set_xlabel( 'normalized DM command')
                    #axx[j].set_ylabel( 'normalized Intensity')
                    axx[j].legend(fontsize=6)
                    #axx[j].set_title(act_idx,fontsize=5)
                    #ins = axx[j].inset_axes([0.15,0.15,0.25,0.25])
                    #ins.imshow(poke_imgs[3][act_idx] )
                    #axx[j].axis('off')
                    j+=1
            except:
                print(f'\n!!!!!!!!!!!!\nfit failed for actuator {act_idx}\n!!!!!!!!!!!!\nanalyse plot to try understand why')
                """nofit_list.append( act_idx ) 
                fig1, ax1 = plt.subplots(1,1)
                ax1.plot( ramp_values, S )
                ax1.set_title('could not fit this!') """
                 
       

    if debug:
        plt.savefig( fig_path + f'process_fits_5_{tstamp}.png' , bbox_inches='tight', dpi=300) 
        #plt.show() 

    if debug:
        """ used to buff things out (adding new 0 normal noise variance to samples) 
        Qlst,Wlst,Flst,mulst = [],[],[],[]
        Q_est =  np.array(list( param_dict.values() ))[:, 0]
        W_est = np.array(list( param_dict.values() ))[:, 1] 
        F_est = np.array(list( param_dict.values() ))[:, 2]
        mu_est = np.array(list( param_dict.values() ))[:, 3] 
        for q,w,f,u in param_dict.values():
            Qlst.append( list( q + 0.01*np.mean(Q_est)*np.random.randn(100 ) ) )
            Wlst.append( list( w + 0.01*np.mean(W_est)*np.random.randn(100 ) ) )
            Flst.append( list( f + 0.01*np.mean(F_est)*np.random.randn(100 ) ) )
            mulst.append( list( u + 0.01*np.mean(mu_est)*np.random.randn(100 ) ) )

        #buffcorners = np.array( [list( param_dict.values() ) for _ in range(10)]).reshape(-1,4)
        buffcorners = np.array([np.array(Qlst).ravel(),np.array(Wlst).ravel(), np.array(Flst).ravel(),np.array(mulst).ravel()]).T
        corner.corner( buffcorners , quantiles=[0.16,0.5,0.84], show_titles=True, labels = ['Q [adu]', 'W [adu/cos(rad)]', 'F [rad/cmd]', r'$\mu$ [rad]'] ) 
        """
        #labels = ['Q', 'W', 'F', r'$\mu$']
        corner.corner( np.array(list( param_dict.values() )), quantiles=[0.16,0.5,0.84], show_titles=True, labels = ['A', 'B', 'F', r'$\mu$'] , range = [(0,2*np.mean(y_data)),(0, 10*(np.max(y_data)-np.min(y_data)) ) , (5,20), (0,6) ] ) #, range = [(2*np.min(S), 102*np.max(S)), (0, 2*(np.max(S) - np.min(S)) ), (5, 20), (-3,3)] ) #['Q [adu]', 'W [adu/cos(rad)]', 'F [rad/cmd]', r'$\mu$ [rad]']
        plt.savefig( fig_path + f'process_fits_6_{tstamp}.png', bbox_inches='tight', dpi=300)
        plt.show()
        
    output_fits = fits.HDUList( [] )

    # reference images 
    N0_fits = fits.PrimaryHDU( N0 )
    N0_fits.header.set('EXTNAME','FPM OUT REF')
    N0_fits.header.set('WHAT IS','ref int. with FPM out')

    I0_fits = fits.PrimaryHDU( I0 )
    I0_fits.header.set('EXTNAME','FPM IN REF')
    I0_fits.header.set('WHAT IS','ref int. with FPM in')

    # output fits files 
    P2C_fits = fits.PrimaryHDU( np.array([P2C_1x1, P2C_3x3]) )
    P2C_fits.header.set('EXTNAME','P2C')
    P2C_fits.header.set('WHAT IS','pixel to DM actuator register')
    P2C_fits.header.set('index 0','P2C_1x1') 
    P2C_fits.header.set('index 1','P2C_3x3')    
    
    #fitted parameters
    param_fits = fits.PrimaryHDU( np.array(list( param_dict.values() )) )
    param_fits.header.set('EXTNAME','FITTED_PARAMS')
    """param_fits.header.set('COL0','Q [adu]')
    param_fits.header.set('COL1','W [adu/cos(rad)]')
    param_fits.header.set('COL2','F [rad/cmd]')
    param_fits.header.set('COL4','mu [rad]')"""
    param_fits.header.set('COL0','A [adu]')
    param_fits.header.set('COL1','B [adu]')
    param_fits.header.set('COL2','F [rad/cmd]')
    param_fits.header.set('COL4','mu [rad]')
    if len(nofit_list)!=0:
        for i, act_idx in enumerate(nofit_list):
            param_fits.header.set(f'{i}_fit_fail_act', act_idx)
        
    #covariances
    cov_fits = fits.PrimaryHDU( np.array(list(cov_dict.values())) )
    cov_fits.header.set('EXTNAME','FIT_COV')
    # residuals 
    res_fits = fits.PrimaryHDU( np.array(fit_residuals) )
    res_fits.header.set('EXTNAME','FIT_RESIDUALS')

    #DM regions 
    dm_fit_regions = fits.PrimaryHDU( np.array( [dm_pupil_filt, dm_pupil_filt*(~poor_registration_list), poor_registration_list] ).astype(int) )
    dm_fit_regions.header.set('EXTNAME','DM_REGISTRATION_REGIONS')
    dm_fit_regions.header.set('registration_threshold',registration_threshold)
    dm_fit_regions.header.set('index 0 ','active_DM_region')   
    dm_fit_regions.header.set('index 1 ','well registered actuators') 
    dm_fit_regions.header.set('index 2 ','poor registered actuators') 
 
    for f in [N0_fits, I0_fits, P2C_fits, param_fits, cov_fits,res_fits, dm_fit_regions ]:
        output_fits.append( f ) 

    if savefits!=None:
           
        output_fits.writeto( savefits, overwrite=True )  #data_path + 'ZWFS_internal_calibration.fits'

    return( output_fits ) 



def _compute_weighted_frequency(vector):
    # Step 1: Compute the Fourier Transform of the vector
    fft_values = np.fft.fft(vector)
    
    # Step 2: Compute the Power Spectral Density (PSD)
    psd = np.abs(fft_values)**2
    
    # Step 3: Compute the frequencies associated with the PSD
    freqs = np.fft.fftfreq(len(vector))
    
    # Only consider the positive frequencies
    positive_freq_indices = np.where(freqs >= 0)
    freqs = freqs[positive_freq_indices]
    psd = psd[positive_freq_indices]
    
    # Step 4: Compute the weighted frequency
    weighted_freq = np.sum(freqs * psd) / np.sum(psd)
    
    return weighted_freq

def sort_vectors_by_weighted_frequency(vectors):
    """
    sort list of vectors based on the power spectral density weighted 
    frequency of the vectors ordered from vectors having most power at 
    lower frequencies to vectors having most power at higher frequencies. 
    
    Useful for sorting DM basis cmds that may seem un-ordered
    """
    
    # Step 5: Compute the weighted frequency for each vector
    weighted_frequencies = [_compute_weighted_frequency(v) for v in vectors]
    
    # Step 6: Sort the vectors based on the weighted frequency
    sorted_vectors = [v for _, v in sorted(zip(weighted_frequencies, vectors))]
    
    return sorted_vectors



def twoD_Gaussian(xy, amplitude, xo, yo, sigma_x, sigma_y, theta, offset):
    x, y = xy
    xo = float(xo)
    yo = float(yo)    
    a = (np.cos(theta)**2)/(2*sigma_x**2) + (np.sin(theta)**2)/(2*sigma_y**2)
    b = -(np.sin(2*theta))/(4*sigma_x**2) + (np.sin(2*theta))/(4*sigma_y**2)
    c = (np.sin(theta)**2)/(2*sigma_x**2) + (np.cos(theta)**2)/(2*sigma_y**2)
    g = offset + amplitude*np.exp( - (a*((x-xo)**2) + 2*b*(x-xo)*(y-yo)
                            + c*((y-yo)**2)))
    return g.ravel()


def fit_b_pixel_space(I0, theta, image_filter , debug=True): 
    # fit b parameter from the reference fields I0 (FPM IN), N0 (FPM OUT) which should be 2D arrays, theta is scalar estimate of the FPM phase shift 
    # we can use N0 to remove bias/bkg by subtraction 

    x = np.linspace(-I0.shape[0]//2 , I0.shape[0]//2 ,I0.shape[0])
    y = np.linspace(-I0.shape[1]//2 , I0.shape[1]//2 ,I0.shape[1])
    X, Y = np.meshgrid( x, y)
    X_f=X.reshape(-1)[image_filter]
    Y_f=Y.reshape(-1)[image_filter]

    # we normalize by average of I0 over entire image 
    I0_mean = np.mean( I0 ) # we take the median FPM OUT signal inside the pupil (appart from center pixels) 
    data = (I0.reshape(-1)[image_filter]/I0_mean).reshape(-1) #((I0-N0)/N0).reshape(-1)[image_filter] #this is M^2/|A|^2
    initial_guess = (np.nanmax(data),np.mean(x),np.mean(y),np.std(x),np.std(y), 0, 0) 

    # fit it 
    popt, pcov = curve_fit(twoD_Gaussian, (X_f, Y_f), data, p0=initial_guess)

    data_fitted = twoD_Gaussian((X, Y), *popt)  #/ np.mean(I0.reshape(-1)[image_filter])

    # fitted b in pixel space
    bfit = data_fitted.reshape(X.shape)**0.5 / (2*(1-np.cos(theta)))**0.5

    if debug:
        im_list = [I0 * image_filter.reshape(I0.shape), bfit]
        xlabel_list = ['','']
        ylabel_list = ['','']
        title_list = ['','']
        cbar_label_list = ['filtered I0 [adu]', 'fitted b'] 
        savefig = None #fig_path + 'b_fit_internal_cal.png'

        nice_heatmap_subplots( im_list , xlabel_list, ylabel_list, title_list,cbar_label_list, fontsize=15, cbar_orientation = 'bottom', axis_off=True, savefig=savefig)

        plt.show()

    return(bfit) 


def put_b_in_cmd_space( b, zwfs , debug = True):
    # b is usually fitted in WFS pixel space - we need to map this back to the DM command space. 

    # create interpolation function zwfs.row_coords ,  zwfs.col_coords 
    b_interpolator = RegularGridInterpolator((zwfs.row_coords, zwfs.col_coords ), b)
    # interpolate onto zwfs.dm_row_coordinates_in_pixels, zwfs.dm_col_coordinates_in_pixels (this is 12x12 DM grid in pixel space). 
    X, Y = np.meshgrid(zwfs.dm_col_coordinates_in_pixels, zwfs.dm_row_coordinates_in_pixels ) 
    pts = np.vstack([X.ravel(), Y.ravel()]).T 
    b_on_dm_in_pixel_space = b_interpolator( pts ) 

    #plt.figure(); plt.imshow( b_on_dm_in_pixel_space.reshape(12,12) );plt.colorbar(); plt.show()
    
    # drop the corners and flatten - this is our b in command space 
    Nx_act_DM = 12 
    corner_indices = [0, Nx_act_DM-1, Nx_act_DM * (Nx_act_DM-1), -1]
    # put corners to nan on flattened array 
    b_on_dm_in_pixel_space.reshape(-1)[corner_indices] = np.nan
    #drop nan values so we go frp, 144 - 140 length array - OUR b in DM CMD SPACE :) 
    b_in_dm_cmd_space = b_on_dm_in_pixel_space[ np.isfinite(b_on_dm_in_pixel_space) ] 

    if debug:
        plt.figure(); 
        plt.title('fitted b in DM cmd space')
        plt.imshow( get_DM_command_in_2D( b_in_dm_cmd_space ));plt.colorbar(); 
        plt.show()
   
    return( b_in_dm_cmd_space ) 




def plot_ts( t_list, sig_list, savefig = None, **kwargs ):

    xlabel = kwargs.get("xlabel","Time [s]")
    ylabel = kwargs.get("ylabel", "Signal")
    title = kwargs.get("title", None)
    fontsize = kwargs.get("fontsize", 15)
    labelsize = kwargs.get("labelsize", 15)
    labels = kwargs.get("labels", [None for _ in range(len(t_list))])
    colors = kwargs.get("colors", ["k" for _ in range(len(t_list))])
    plt.figure( figsize=(8,5) )

    for i, (t, s) in enumerate( zip( t_list, sig_list) ) :
        
        plt.plot( t, s , color=colors[i], label = f"{labels[i]}") 
    plt.gca().tick_params(labelsize=labelsize)
    plt.xlabel(xlabel,fontsize=fontsize)
    plt.ylabel(ylabel,fontsize=fontsize)
    plt.title( title )

    #plt.title("Pixel-wise Power Spectral Density (Welch)")
    plt.legend(fontsize=12)
    #plt.grid(True, which="both", linestyle="--", alpha=0.5)
    #plt.tight_layout()
    if savefig is not None:
        plt.savefig( savefig, dpi=200, bbox_inches = 'tight')
    plt.show()

def plot_psd( f_list, psd_list, savefig = None, **kwargs ):

    xlabel = kwargs.get("xlabel","Frequency [Hz]")
    ylabel = kwargs.get("ylabel", "Power Spectral Density")
    title = kwargs.get("title", None)
    fontsize = kwargs.get("fontsize", 15)
    labelsize = kwargs.get("labelsize", 15)
    plot_cumulative = kwargs.get("plot_cumulative",True)
    labels = kwargs.get("labels", [None for _ in range(len(f_list))])
    colors = kwargs.get("colors", ["k" for _ in range(len(f_list))])
    plt.figure( figsize=(8,5) )

    for i, (f, psd) in enumerate( zip( f_list, psd_list) ) :
        df = np.mean( np.diff( f ) )
        plt.loglog( f, psd , color=colors[i] , label = f"{labels[i]}") 
        if plot_cumulative:
            plt.loglog(f, np.cumsum(psd[::-1] * df )[::-1], color=colors[i], alpha =0.5, ls=':', linewidth=2) #, label=f"{labels[i]} Reverse Cumulative")

    plt.gca().tick_params(labelsize=labelsize)
    plt.xlabel(xlabel,fontsize=fontsize)
    plt.ylabel(ylabel,fontsize=fontsize)
    plt.title( title )

    #plt.title("Pixel-wise Power Spectral Density (Welch)")
    plt.legend(fontsize=12)
    #plt.grid(True, which="both", linestyle="--", alpha=0.5)
    #plt.tight_layout()
    if savefig is not None:
        plt.savefig( savefig, dpi=200, bbox_inches = 'tight')
    plt.show()



import numpy as np
from scipy.signal import welch

def _auto_nperseg(N, target_segments=8, minlen=256):
    """Choose a power-of-two nperseg giving ~target_segments, bounded by data length."""
    if N <= 0:
        raise ValueError("Signal length must be > 0")
    n = max(minlen, int(N // max(1, target_segments)))
    n = min(n, N)
    # round down to power of two (Welch likes radix-2 FFT sizes)
    n_pow2 = 2 ** int(np.floor(np.log2(max(2, n))))
    return min(n_pow2, N)

def psd_welch_single(
    t, x, *, fs=None, nperseg=None, noverlap=None,
    window="hann", detrend="constant", scaling="density", average="median",
    jitter_tol=1e-2, fix_nonuniform="none"  # or "resample_linear"
):
    """
    Compute one-sided PSD using Welch's method.

    Parameters
    ----------
    t : array-like or None
        Time stamps (seconds). Must be (approximately) uniform if provided.
        If None, you must pass fs (sampling rate).
    x : array-like
        Signal values (1-D).
    fs : float, optional
        Sampling rate [Hz]. If not given, estimated from median dt.
    nperseg : int, optional
        Segment length. If None, auto-chosen (~8 segments, power-of-two).
    noverlap : int, optional
        Overlap samples. If None, defaults to 50% of nperseg.
    window, detrend, scaling, average : passed to scipy.signal.welch
    jitter_tol : float
        Max allowed relative timestamp jitter (e.g. 0.01 = 1%) before action.
    fix_nonuniform : {"none","resample_linear"}
        If non-uniform beyond tolerance, either raise ("none") or resample
        x onto a uniform grid using linear interpolation.

    Returns
    -------
    f : ndarray
        Frequency bins [Hz].
    Pxx : ndarray
        One-sided PSD [x^2/Hz] if scaling="density".
    """
    x = np.asarray(x, dtype=float)
    if t is None and fs is None:
        raise ValueError("Provide either t or fs.")
    if t is not None:
        t = np.asarray(t, dtype=float)
        if t.ndim != 1 or t.size != x.size:
            raise ValueError("t must be 1-D and the same length as x.")
        dt = np.diff(t)
        if not np.all(np.isfinite(dt)):
            raise ValueError("Non-finite timestamps in t.")
        dt_med = np.median(dt)
        fs_est = 1.0 / dt_med
        jitter = np.max(np.abs(dt - dt_med)) / dt_med if dt_med > 0 else np.inf
        if jitter > jitter_tol:
            if fix_nonuniform == "resample_linear":
                N = len(x)
                t_uniform = t[0] + np.arange(N) * dt_med
                x = np.interp(t_uniform, t, x)
                fs = fs_est
            else:
                raise ValueError(
                    f"Non-uniform sampling (jitter {jitter:.2%} > {100*jitter_tol:.2f}%). "
                    "Set fix_nonuniform='resample_linear' to auto-fix."
                )
        else:
            fs = fs_est if fs is None else fs
    # choose defaults
    if nperseg is None:
        nperseg = _auto_nperseg(len(x))
    if noverlap is None:
        noverlap = nperseg // 2

    f, Pxx = welch(
        x, fs=fs, window=window, nperseg=nperseg, noverlap=noverlap,
        detrend=detrend, scaling=scaling, average=average, return_onesided=True
    )
    return f, Pxx

def psd_welch_batch(
    t_list, x_list, **kwargs
):
    """
    Compute Welch PSD for a batch of time series.

    Parameters
    ----------
    t_list : list of 1-D arrays or list of None
        List of time vectors (or None for each if fs is supplied in kwargs).
    x_list : list of 1-D arrays
        List of signals (same lengths as corresponding t).
    **kwargs : passed to psd_welch_single (e.g., fs, nperseg, average, fix_nonuniform)

    Returns
    -------
    f_list, psd_list : lists of ndarrays
        Each entry aligns with input series.
    """
    if len(t_list) != len(x_list):
        raise ValueError("t_list and x_list must have the same length.")
    f_list, psd_list = [], []
    for t, x in zip(t_list, x_list):
        f, Pxx = psd_welch_single(t, x, **kwargs)
        f_list.append(f)
        psd_list.append(Pxx)
    return f_list, psd_list






def _dm_idx_map_12x12():
    """
    Build a 12x12 index map that enumerates the 140 valid actuators row-major,
    skipping the four corners (set to -1).
    """
    n = 12
    idx_map = -np.ones((n, n), dtype=int)
    k = 0
    for i in range(n):
        for j in range(n):
            if (i, j) in [(0,0), (0,n-1), (n-1,0), (n-1,n-1)]:
                continue
            idx_map[i, j] = k
            k += 1
    # sanity: should enumerate 0..139
    if k != 140:
        raise RuntimeError(f"Index map built {k} actuators, expected 140.")
    return idx_map

def _dm_grid_from_140(cmd140):
    """
    Map a 140-length vector to a 12x12 grid (corners = NaN).
    """
    cmd140 = np.asarray(cmd140, dtype=float).reshape(-1)
    if cmd140.size != 140:
        raise ValueError("cmd140 must be length 140.")
    idx_map = _dm_idx_map_12x12()
    grid = np.full(idx_map.shape, np.nan, dtype=float)
    valid = idx_map >= 0
    grid[valid] = cmd140[idx_map[valid]]
    return grid, idx_map

def plot_dm140(
    cmd140,
    *,
    flip_dm=True,
    annotate=True,
    index_one_based=True,
    cmap="viridis",
    vmin=None,
    vmax=None,
    title="DM command",
    label_color="white",
    fontsize=10,
    savefig=None,
    ax=None,
):
    """
    Plot a 140-length DM vector as a 12x12 image with actuator labels.

    Parameters
    ----------
    cmd140 : (140,) array-like
        DM command/values for the 140 actuators (0..1 or arbitrary units).
    flip_dm : bool
        If True, flip vertically for display (matches your previous convention).
    annotate : bool
        Overlay actuator indices on valid cells.
    index_one_based : bool
        If True, labels are 1..140; otherwise 0..139.
    cmap, vmin, vmax : matplotlib imshow controls.
    title : str
    label_color : str
    fontsize : int
    savefig : str or Path or None
    ax : matplotlib Axes or None

    Returns
    -------
    fig, ax : Matplotlib Figure and Axes.
    """
    grid, idx_map = _dm_grid_from_140(cmd140)

    # Optional flip for display (and matching labels)
    if flip_dm:
        grid = np.flipud(grid)
        idx_map = np.flipud(idx_map)

    # Create figure/axes
    created_ax = False
    if ax is None:
        fig = plt.figure(figsize=(5, 5))
        ax = fig.add_subplot(111)
        created_ax = True
    else:
        fig = ax.figure

    # Show image (mask NaNs for nicer rendering)
    im = ax.imshow(grid, cmap=cmap, vmin=vmin, vmax=vmax)
    ax.set_title(title)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.xaxis.tick_top()

    # Colorbar at the bottom
    divider = make_axes_locatable(ax)
    cax = divider.append_axes('bottom', size='5%', pad=0.05)
    cb = fig.colorbar(im, cax=cax, orientation='horizontal')
    cb.ax.tick_params(labelsize=fontsize)

    # Annotate actuator indices
    if annotate:
        n_rows, n_cols = grid.shape
        pe_stroke = [pe.withStroke(linewidth=2.0, foreground="black")]
        for i in range(n_rows):
            for j in range(n_cols):
                k = idx_map[i, j]
                if k >= 0:  # valid actuator
                    lab = k + 1 if index_one_based else k
                    ax.text(
                        j, i, f"{lab}",
                        ha="center", va="center",
                        color=label_color, fontsize=fontsize,
                        path_effects=pe_stroke,
                    )

    if savefig is not None:
        fig.savefig(savefig, dpi=300, bbox_inches="tight")

    if created_ax:
        plt.show()

    return fig, ax




def _format_frac(alpha, max_den=100):
    """Return LaTeX-ready fraction for alpha with bounded denominator."""
    frac = Fraction(alpha).limit_denominator(max_den)
    num, den = frac.numerator, frac.denominator
    if den == 1:
        # integer; avoid silly huge integers
        return f"{num}" if abs(num) <= 100 else f"{alpha:.2f}"
    return fr"\frac{{{num}}}{{{den}}}"

def fit_psd_powerlaw(
    f, psd, *,
    fmin=None, fmax=None,
    log_base=10.0,
    plot=False, ax=None, label=None, color=None, linewidth=2,
    return_fit_curve=False,
    max_fraction_den=100,
    show_B=True
):
    """
    Fit PSD(f) = B * f^alpha in log-log space. Optionally plot with alpha as a fraction.
    """
    f = np.asarray(f, float)
    psd = np.asarray(psd, float)

    m = np.isfinite(f) & np.isfinite(psd) & (f > 0) & (psd > 0)
    if fmin is not None: m &= (f >= fmin)
    if fmax is not None: m &= (f <= fmax)
    fx, Px = f[m], psd[m]
    if fx.size < 2:
        raise ValueError("Not enough valid points in the specified band to fit a power law.")

    # choose log base
    if np.isclose(log_base, 10.0):
        log = np.log10; exp = lambda x: 10**x; ln_base = np.log(10.0)
    elif np.isclose(log_base, np.e):
        log = np.log;   exp = np.exp;           ln_base = 1.0
    else:
        ln_base = np.log(log_base)
        log = lambda x: np.log(x) / ln_base
        exp = lambda x: np.exp(x * ln_base)

    x = log(fx); y = log(Px)
    p, cov = np.polyfit(x, y, 1, cov=True)
    alpha, intercept = p
    alpha_std = float(np.sqrt(cov[0, 0]))
    intercept_std = float(np.sqrt(cov[1, 1]))

    B = exp(intercept)
    B_std = float(abs(ln_base) * B * intercept_std)

    y_hat = alpha * x + intercept
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan

    out = {
        "alpha": float(alpha), "B": float(B),
        "alpha_std": float(alpha_std), "B_std": float(B_std),
        "intercept": float(intercept), "intercept_std": float(intercept_std),
        "r2": float(r2), "npts": int(fx.size),
    }

    if return_fit_curve or plot:
        f_fit = np.linspace(fx.min(), fx.max(), 200)
        psd_fit = B * (f_fit ** alpha)
        if return_fit_curve:
            out["f_fit"] = f_fit; out["psd_fit"] = psd_fit

    if plot:
        if ax is None:
            fig, ax = plt.subplots(figsize=(6, 4))
        ax.loglog(f, psd, alpha=0.6, label="PSD (data)" if label is None else None)

        frac_tex = _format_frac(alpha, max_den=max_fraction_den)
        if show_B:
            B_tex = f"{B:.2g}"
            fit_lbl = label or rf"$\widehat{{P}}(f)={B_tex}\,f^{{{frac_tex}}}$"
        else:
            fit_lbl = label or rf"fit: $f^{{{frac_tex}}}$"

        ax.loglog(f_fit, psd_fit, color=color, lw=linewidth, label=fit_lbl)
        ax.set_xlabel("Frequency [Hz]")
        ax.set_ylabel("Power Spectral Density")
        ax.set_title("PSD and power-law fit")
        ax.legend()
        ax.grid(True, which="both", ls=":", alpha=0.3)

    return out

# def fit_psd_powerlaw(
#     f, psd, *,
#     fmin=None, fmax=None,
#     log_base=10.0,
#     plot=False, ax=None, label=None, color=None, linewidth=2,
#     return_fit_curve=False
# ):
#     """
#     Fit PSD(f) = B * f^alpha by linear regression in log-log space.

#     Parameters
#     ----------
#     f, psd : 1-D arrays
#         Frequency [Hz] and PSD values (>0).
#     fmin, fmax : float or None
#         Optional frequency band for the fit (inclusive).
#     log_base : 10 or np.e
#         Use log10 (default) or natural log for the linearization.
#     plot : bool
#         If True, plot loglog PSD and fitted power law over the fit band.
#     ax : matplotlib Axes or None
#         Axes to plot on; created if None and plot=True.
#     label, color : str or None
#         Plot styling for the fit line.
#     linewidth : int/float
#         Fit line width.
#     return_fit_curve : bool
#         If True, also return (f_fit, psd_fit) arrays covering the fit band.

#     Returns
#     -------
#     result : dict
#         {
#           "alpha": slope exponent,
#           "B": prefactor,
#           "alpha_std": std error of alpha,
#           "B_std": std error of B (propagated),
#           "intercept": intercept in chosen log base,
#           "intercept_std": std error of intercept,
#           "r2": coefficient of determination on log data,
#           "npts": number of points used,
#           # optionally:
#           "f_fit": ..., "psd_fit": ...
#         }
#     """
#     f = np.asarray(f, float)
#     psd = np.asarray(psd, float)

#     # Valid data mask
#     m = np.isfinite(f) & np.isfinite(psd) & (f > 0) & (psd > 0)
#     if fmin is not None:
#         m &= (f >= fmin)
#     if fmax is not None:
#         m &= (f <= fmax)
#     fx, Px = f[m], psd[m]
#     if fx.size < 2:
#         raise ValueError("Not enough valid points in the specified band to fit a power law.")

#     # Choose log base
#     if log_base == 10 or np.isclose(log_base, 10.0):
#         log = np.log10
#         exp = lambda x: 10**x
#         ln_base = np.log(10.0)  # for error propagation
#     elif log_base in (np.e, np.exp(1)):
#         log = np.log
#         exp = np.exp
#         ln_base = 1.0
#     else:
#         # generic base: change-of-base to natural log
#         ln_base = np.log(log_base)
#         log = lambda x: np.log(x) / ln_base
#         exp = lambda x: np.exp(x * ln_base)

#     x = log(fx)
#     y = log(Px)

#     # Linear regression in log space: y = alpha * x + intercept
#     # Use numpy polyfit with covariance
#     p, cov = np.polyfit(x, y, 1, cov=True)
#     alpha = p[0]
#     intercept = p[1]
#     alpha_std = float(np.sqrt(cov[0, 0]))
#     intercept_std = float(np.sqrt(cov[1, 1]))

#     # Convert intercept to B in linear space
#     B = exp(intercept)
#     # Propagate uncertainty: B = base^(intercept)
#     # dB/d(intercept) = ln(base) * base^(intercept) = ln_base * B
#     B_std = float(abs(ln_base) * B * intercept_std)

#     # Goodness of fit on log data
#     y_hat = alpha * x + intercept
#     ss_res = float(np.sum((y - y_hat) ** 2))
#     ss_tot = float(np.sum((y - np.mean(y)) ** 2))
#     r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan

#     out = {
#         "alpha": float(alpha),
#         "B": float(B),
#         "alpha_std": float(alpha_std),
#         "B_std": float(B_std),
#         "intercept": float(intercept),
#         "intercept_std": float(intercept_std),
#         "r2": float(r2),
#         "npts": int(fx.size),
#     }

#     # Optional fitted curve (monotone over the fit band)
#     if return_fit_curve or plot:
#         f_fit = np.linspace(fx.min(), fx.max(), 200)
#         psd_fit = B * (f_fit ** alpha)
#         if return_fit_curve:
#             out["f_fit"] = f_fit
#             out["psd_fit"] = psd_fit

#     # Optional plotting
#     if plot:
#         if ax is None:
#             fig, ax = plt.subplots(figsize=(6, 4))
#         ax.loglog(f, psd, alpha=0.6, label="PSD (data)" if label is None else None)
#         ax.loglog(f_fit, psd_fit, color=color, lw=linewidth,
#                   label=(label or fr"fit: $B\,f^{{\alpha}}$, $\alpha={alpha:.2f}$"))
#         ax.set_xlabel("Frequency [Hz]")
#         ax.set_ylabel("Power Spectral Density")
#         ax.set_title("PSD and power-law fit")
#         ax.legend()
#         ax.grid(True, which="both", ls=":", alpha=0.3)

#     return out




def fit_psd_powerlaw_batch(
    f_list, psd_list, *,
    fmin=None, fmax=None,
    log_base=10.0,
    return_arrays=True
):
    """
    Batch fit of PSD(f) = B * f^alpha across multiple series (no plotting).

    Parameters
    ----------
    f_list, psd_list : list of 1-D arrays
        Lists of frequency and PSD arrays (same lengths as each other).
    fmin, fmax : float or list of float or None
        Scalar band for all series, or per-series list (len == len(f_list)).
    log_base : 10 or np.e
        Log base used by the linearization (must match fit_psd_powerlaw).
    return_arrays : bool
        If True, also return stacked NumPy arrays of key parameters.

    Returns
    -------
    results : list of dict
        Each entry is the dict returned by fit_psd_powerlaw(..., plot=False).
    arrays (optional) : dict of np.ndarray
        Stacked parameters: alpha, B, alpha_std, B_std, intercept, intercept_std, r2, npts.
    """
    n = len(f_list)
    if n != len(psd_list):
        raise ValueError("f_list and psd_list must have the same length.")

    def _as_seq(val):
        if val is None:
            return [None] * n
        if np.isscalar(val):
            return [val] * n
        if len(val) != n:
            raise ValueError("Per-series band list must match len(f_list).")
        return list(val)

    fmin_seq = _as_seq(fmin)
    fmax_seq = _as_seq(fmax)

    results = []
    for i, (f, psd, fmn, fmx) in enumerate(zip(f_list, psd_list, fmin_seq, fmax_seq)):
        res = fit_psd_powerlaw(
            f, psd,
            fmin=fmn, fmax=fmx,
            log_base=log_base,
            plot=False,
            return_fit_curve=False
        )
        results.append(res)

    if not return_arrays:
        return results

    # Stack key fields into arrays (use NaN if missing)
    def _stack(key, fill=np.nan):
        vals = []
        for r in results:
            vals.append(r.get(key, fill))
        return np.asarray(vals)

    arrays = dict(
        alpha=_stack("alpha"),
        B=_stack("B"),
        alpha_std=_stack("alpha_std"),
        B_std=_stack("B_std"),
        intercept=_stack("intercept"),
        intercept_std=_stack("intercept_std"),
        r2=_stack("r2"),
        npts=_stack("npts"),
    )
    return results, arrays


"""
    # get an estimate for things that should not ideally have spatial variability
    _, _, F_est, mu_est = np.median( list( param_dict.values() ) ,axis = 0)

    # from mu measure theta - our estimated phase shift  
    #   (using result: mu = tan^-1(sin(theta)/(cos(theta)-1) = 0.5*(theta - pi) ) 
    theta_est  = 2*mu_est + np.pi
        
    # THEORETICALLY THIS SIGNAL: S = (I - N0)/N0 = |M0|^2/|A|^2 + 2*M0/|A| * cos(F.c + mu) 
    # Q =  |M0|^2/|A|^2 , W = |M0|/|A|. 
    # therefore M0 = np.sqrt( Q * |A|^2 ) => W = Q
        

    # things that can have spatial variability
    Q_est =  np.array(list( param_dict.values() ))[:, 0]
    W_est = np.array(list( param_dict.values() ))[:, 1] 


    # ========== GET A and B from these ... check against measured values within pupil obstruction and fit gaussian for b within pupil.   

    A_est = np.sqrt( Q_est + np.sqrt( Q_est**2 + W_est**2 ) ) / np.sqrt(2) 

    M_est = W_est /  (np.sqrt(2) * np.sqrt( Q_est + np.sqrt( Q_est**2 - W_est**2 ) )  ) 
    
    b_est = M_est / (2*(1-np.cos(theta_est)))**0.5

    bsamples = np.nan * np.zeros( dm_pupil_filt.shape)
    bsamples[dm_pupil_filt] = b_est

    plt.figure()
    plt.imshow( get_DM_command_in_2D( bsamples )); plt.colorbar()
    plt.show()
"""






