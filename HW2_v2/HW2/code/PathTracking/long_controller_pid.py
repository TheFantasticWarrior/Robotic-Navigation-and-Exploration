import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class PIDLongController(Controller):
    def __init__(self, model, a_range, kp=1.5, ki=5, kd=0):
        self.path = None
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.acc_ep = 0
        self.last_ep = 0
        self.dt = model.dt
        self.a_range = a_range
        self.current_idx = 0
    
    def set_path(self, path):
        super().set_path(path)
        self.acc_ep = 0
        self.last_ep = 0
        self.current_idx = 0
    
    def feedback(self, info):
        # Check Path
        #print(3,self.current_idx,len(self.path))
        
        if self.path is None:
            print("No path !!")
            return None, None
        
        # Extract State
        x, y, yaw, v = info["x"], info["y"], info["yaw"], info["v"]

        # Check if reached end of track
        #NewFeature remove goal check or else can't reach goal
        if False and self.current_idx >= len(self.path)-1 :
            # Brake to 0 speed using PID when finishing the track
            v_ref = 0.0
            target = self.path[-1]
            #print(2,self.current_idx,len(self.path))
            return np.clip(-v, self.a_range[0], self.a_range[1]), target
        else:
            # Search Nearest Target Locally
            min_idx, min_dist = utils.search_nearest_local(self.path, (x,y), self.current_idx, lookahead=50)
            self.current_idx = min_idx
            target = self.path[min_idx]
            v_ref = target[4]
            #print(4,self.current_idx,len(self.path),v_ref)
        
        # TODO 3.2: PID Control for Longitudinal Motion
        #NewFeature monza specifically needs this to reach goal
        v_ref=max(v_ref,10)
        next_a = v_ref - v
        # [end] TODO 3.2
        u=self.kp*next_a+self.ki*self.acc_ep+self.kd*(next_a-self.last_ep)/ self.dt
        self.acc_ep+=next_a*self.dt
        self.last_ep=next_a
        next_a=u
        return next_a, target