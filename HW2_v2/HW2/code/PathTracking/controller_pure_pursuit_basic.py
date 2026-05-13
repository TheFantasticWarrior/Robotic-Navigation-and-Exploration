import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPurePursuitBasic(Controller):
    def __init__(self, model, 
                 # Optional TODO: Tune Pure Pursuit Gain
                 kp=0.6, Lfc=1):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.current_idx = 0

    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 3:
            return 0.0

        min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
        self.current_idx = min_idx
        
        Ld = self.kp*v + self.Lfc

        # Optional TODO: Pure Pursuit Control for Basic Kinematic Model
        # You can implement this if you want to use Pure Pursuit for basic kinematic model in F1 Challenge
        next_w = 0
        dist=min_dist
        i=min_idx
        while dist<Ld**2 and i<len(self.path)-1:
            i+=1
            dist = (x - self.path[i,0])**2 + (y - self.path[i,1])**2
        target=self.path[i]
        theta_target = np.arctan2(target[1] - y, target[0]-x)
        theta_err = (theta_target - np.deg2rad(yaw))
        next_delta = np.arctan2(2*v*np.sin(theta_err),Ld)
        #r=Ld/2/np.sin(theta_err)
        #print(theta_target,yaw,theta_err,next_delta,r,self.l)
        # [end] TODO 4.3.1
        return np.rad2deg(next_delta)

        return next_w
