
import cv2
import numpy as np

from path_planning import *
from path_planning.rrt_star_planner import RRTStarPlanner


class RRTStarImplementation(RRTStarPlanner):
    # TODO: implement your own version of preloop, step and postloop
    def preloop(self):
        # This is for illustrative purposes only, feel free to modify
        self.visited_nodes:set[PathNode] = set()
        self.visited_nodes.add(self.start_node)

    def step(self):
        # ===== some given data/parameters =====
        self.start_node
        self.goal_node
        self.world_map # bgr
        self.occupancy_map # bool
        self.goal_threshold
        self.step_size # for steer
        self.search_radius # for reparent/rewire
        # ==========
        random_node = self.sample_random_node() # must use this to 
                                            # sample new node
        
        v=list(self.visited_nodes)
        dist=[calculate_node_distance(random_node,x) for x in v]
        idx=np.argmin(dist)
        nearest=v[idx]
        if dist[idx]<self.step_size:
            new=random_node
        else:
            new=PathNode(PixelCoordinates(*(
            np.array((random_node.coordinates-nearest.coordinates).to_tuple())*self.step_size/dist[idx]+nearest.coordinates.to_tuple())))
        print(calculate_node_distance(new,nearest),self.search_radius)
        if not check_collision_free(self.occupancy_map,nearest,new):
            return
        near_nodes={x:calculate_node_distance(new,x) for x in v if calculate_node_distance(new,x)<=self.search_radius and check_collision_free(self.occupancy_map,x,new)}
        if not len(near_nodes):
            return
        parent_cost=[x.cost+y for x,y in near_nodes.items()]
        idx=np.argmin(parent_cost)
        new.parent=list(near_nodes.keys())[idx]
        new.cost=parent_cost[idx]
        
        for x,d in near_nodes.items():
            if d+new.cost<x.cost:
                x.parent=new
                x.cost=d+new.cost
        self.visited_nodes.add(new)
        if calculate_node_distance(new,self.goal_node)<self.goal_threshold:
            self.goal_node.parent=new
            self.goal_node.cost=new.cost+calculate_node_distance(new,self.goal_node)
            self.visited_nodes.add(self.goal_node)
            self.is_done.set() # only set this on termination
            return
        #self.is_done.set() # only set this on termination
    
    def postloop(self):
        return (
            collect_path(self.goal_node), 
            self.visited_nodes # replace with set of visited nodes
        )