import os
import sys
##################################################################
# deal with the folders
url1=os.path.join(os.getcwd(),"model/")
url2=os.path.join(os.getcwd(),"traj_handles_ro47001/")
url3=os.path.join(os.getcwd(),"map/")
sys.path.append(url1)
sys.path.append(url2)
sys.path.append(url3)
#################################################################
import numpy as np

def tj_from_line(start_pos, end_pos, time_ttl, t_c):
    v_max = (end_pos-start_pos)*2/time_ttl
    if t_c >= 0 and t_c < time_ttl/2:
        vel = v_max*t_c/(time_ttl/2)
        pos = start_pos + t_c*vel/2
        acc = np.array([0, 0, 0])
    else:
        vel = v_max*(time_ttl-t_c)/(time_ttl/2)
        pos = end_pos - (time_ttl-t_c)*vel/2
        acc = np.array([0, 0, 0])
    
    return pos, vel, acc