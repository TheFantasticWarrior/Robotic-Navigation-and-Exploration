import sys
import numpy as np 
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBicycle(Controller):
    def __init__(self, model, Q=None, R=None, control_state='steering_angle'):
        self.path = None
        if control_state == 'steering_angle':
            self.Q = np.eye(2)
            self.R = np.eye(1)
            # TODO 4.4.1: Tune LQR Gains
            self.Q[0,0] = 1
            self.Q[1,1] = 50
            self.Q[0,1] =self.Q[1,0] = -5
            self.R[0,0] = 2000
        elif control_state == 'steering_angular_velocity':
            self.Q = np.eye(3)
            self.R = np.eye(1)
            # TODO 4.4.4: Tune LQR Gains
            self.Q[0,0] = 100
            self.Q[1,1] = 50
            self.Q[2,2] = 1
            self.R[0,0] = 20
        else:
            self.Q = np.eye(5)
            self.R = np.eye(1)
            # TODO 4.4.4: Tune LQR Gains
            self.Q[0,0] = 100
            self.Q[1,1] = 5
            self.Q[2,2] = 1
            self.Q[3,3] = 100
            self.Q[4,4] = 1000
            self.R[0,0] = 2000
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
        self.dt = model.dt
        self.l = model.l
        self.control_state = control_state
        self.current_idx = 0

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0
        self.pdelta = 0
        self.current_idx = 0

    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01): # Discrete-time Algebra Riccati Equation (DARE)
        P = Q.copy()
        for i in range(max_iter):
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        #print(i)
        return Pn

    # State: [x, y, yaw, delta, v]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None
        
        # Extract State 
        x, y, yaw, delta, v = info["x"], info["y"], info["yaw"], info["delta"], info["v"]
        yaw = utils.angle_norm(yaw)

        # Check if reached end of track
        if self.current_idx >= len(self.path) - 3:
            return 0.0
        
        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])
        
        if self.control_state == 'steering_angle':
            # TODO 4.4.1: LQR Control for Bicycle Kinematic Model with steering angle as control input
            a=np.eye(2)
            a[0,1]=v*self.dt
            b=np.zeros((2,1))
            b[1]=v*self.dt/self.l
            
            p=self._solve_DARE(a,b,self.Q,self.R)
            #print(v,float(yaw),target[2])
            #breakpoint()
            dx = target[0] - x
            dy = target[1] - y
            angle=np.deg2rad(target[2])
            error1=np.sin(angle)*dx-np.cos(angle)*dy
            u=-np.linalg.inv(self.R + b.T @ p @ b)@b.T@p@a@np.array([error1,np.deg2rad(utils.angle_norm(yaw-target[2]))])
            next_delta = float(u)
            # [end] TODO 4.4.1
        elif self.control_state == 'steering_angular_velocity':
            # TODO 4.4.4: LQR Control for Bicycle Kinematic Model with steering angular velocity as control input
            a=np.eye(3)
            a[0,1]=v*self.dt
            a[1,2]=v*self.dt/self.l
            
            b=np.zeros((3,1))
            b[2]=self.dt
            p=self._solve_DARE(a,b,self.Q,self.R)
            #print(v,float(yaw),target[2])
            #breakpoint()
            dx = target[0] - x
            dy = target[1] - y
            angle=np.deg2rad(target[2])
            error1=np.sin(angle)*dx-np.cos(angle)*dy
            u=-np.linalg.inv(self.R + b.T @ p @ b)@b.T@p@a@np.array([error1,np.deg2rad(utils.angle_norm(yaw-target[2])),self.pdelta])
            next_delta = float(u)*self.dt+self.pdelta
            #next_delta = 0
            self.pdelta=next_delta
            # [end] TODO 4.4.4
        else:
            target2 = self.path[min(len(self.path)-1,min_idx+3)]
            target2[2] = utils.angle_norm(target2[2])
        
            a=np.eye(5)
            a[0,1]=v*self.dt
            a[1,2]=v*self.dt/self.l
            a[3,4]=v*1*self.dt
            a[4,2] = v *1* self.dt / self.l
            a[3,3]*=0.99
            a[4,4]*=0.99
            b=np.zeros((5,1))
            b[2]=self.dt
            
            p=self._solve_DARE(a,b,self.Q,self.R)
            #print(v,float(yaw),target[2])
            #breakpoint()
            dx = target[0] - x
            dy = target[1] - y
            angle=np.deg2rad(target[2])
            error1=np.sin(angle)*dx-np.cos(angle)*dy
            
            dx2 = target2[0] - x
            dy2 = target2[1] - y
            angle2=np.deg2rad(target2[2])
            error2=np.sin(angle2)*dx2-np.cos(angle2)*dy2

            u=-np.linalg.inv(self.R + b.T @ p @ b)@b.T@p@a@np.array([error1,np.deg2rad(utils.angle_norm(yaw-target[2])),self.pdelta,error2,np.deg2rad(utils.angle_norm(yaw-target2[2]))])
            next_delta = float(u)*self.dt+self.pdelta
            #next_delta = 0
            self.pdelta=next_delta
        
        return np.rad2deg(next_delta)
