import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller
global alpha,kp

class ControllerStanleyBicycle(Controller):
    def __init__(self, model, 
                 # TODO 4.3.1: Tune Stanley Gain
                 kp=1,alpha=1):
        self.path = None
        self.kp = kp
        self.l = model.l
        self.kp2 = 1.5
        self.Lfc=0
        self.current_idx = 0
        #NewFeature
        self.last_delta=0
        self.alpha=alpha
    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]

        # Check if reached end of track
        #if self.current_idx >= len(self.path) - 5:
        #    return 0.0

        # Search Front Wheel Target Locally
        front_x = x + self.l*np.cos(np.deg2rad(yaw))
        front_y = y + self.l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta)) if np.cos(np.deg2rad(delta)) != 0 else v
        
        min_idx, min_dist = utils.search_nearest_local(self.path, (front_x,front_y), self.current_idx, lookahead=50)
        if self.current_idx >= len(self.path) - 5:
            min_idx=0
        self.current_idx = min_idx
        
        #NewFeature pure pursuit
        dist=min_dist
        i=min_idx
        Ld = self.kp2*v + self.Lfc
        
        while dist<Ld**2 and i<len(self.path)-1:
            i+=1
            dist = (x - self.path[i,0])**2 + (y - self.path[i,1])**2
        target = self.path[min_idx]
            
        # TODO 4.3.1: Stanley Control for Bicycle Kinematic Model
        next_delta = 0
        theta_e=(target[2]-yaw+180)%360-180
        dx = target[0] - front_x
        dy = target[1] - front_y
        #angle=np.atan2(dy,dx)-np.deg2rad(yaw)
        angle=np.deg2rad(target[2])
        error1=np.sin(angle)*dx-np.cos(angle)*dy
        #error2=-np.sign((np.rad2deg(np.atan2(dy,dx))-yaw+180)%360-180)*np.sqrt(min_dist)
        #print(error1,error2)
        error=error1
        next_delta=(np.rad2deg(np.arctan2(-self.kp*error,vf+0.1))+theta_e)
        alpha=0.8
        #next_delta=next_delta*alpha+self.last_delta*(1-alpha)
        
        # [end] TODO 4.3.1

        
        #pure pursuit
        target = self.path[i]
        theta_target = np.arctan2(target[1] - y, target[0]-x)
        theta_err = (theta_target - np.deg2rad(yaw))
        pp = np.rad2deg(np.arctan2(2*self.l*np.sin(theta_err),Ld))
        self.last_delta=next_delta*alpha+self.last_delta*(1-alpha)
        next_delta=(pp*(1-self.alpha)+self.last_delta*self.alpha)
        
        #next_delta=next_delta*self.alpha+pp*(1-self.alpha)
        return next_delta
