"""Pure functions for the Lab 9 observation adapter, testable without ROS."""
import math
import numpy as np
from arc_rl.nav_core import OBS, build_observation


def sample_laser(ranges, angle_min, angle_increment, range_min, range_max):
    values=np.asarray(ranges,dtype=float)
    if len(values)<OBS.n_beams or angle_increment<=0 or (len(values)-1)*angle_increment < 2*math.pi-.08:
        raise ValueError('A positive-angle full-circle scan is required')
    if not np.all((np.isfinite(values)&(values>=range_min)&(values<=range_max)) | np.isposinf(values)):
        raise ValueError('Invalid laser sample. Stop before inference.')
    if range_max < OBS.lidar_max_range-.01:
        raise ValueError('Laser range does not match the trained observation contract')
    targets=np.linspace(-math.pi,math.pi,OBS.n_beams,endpoint=False)
    angles=angle_min+np.arange(len(values))*angle_increment
    differences=np.abs(np.arctan2(np.sin(angles[:,None]-targets),np.cos(angles[:,None]-targets)))
    indices=differences.argmin(axis=0)
    if np.any(differences[indices,np.arange(OBS.n_beams)]>angle_increment*1.01):
        raise ValueError('Scan does not cover the required directions')
    return np.minimum(values[indices],OBS.lidar_max_range).astype(np.float32)


def from_pose(beams, x,y,yaw,gx,gy,v,w):
    angle=math.atan2(gy-y,gx-x)-yaw
    return build_observation(beams,math.hypot(gx-x,gy-y),math.atan2(math.sin(angle),math.cos(angle)),v,w)
