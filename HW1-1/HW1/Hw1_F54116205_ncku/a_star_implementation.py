
import cv2
import numpy as np
from sklearn import neighbors

from path_planning import *
from path_planning.a_star_planner import AStarPlanner


class AStarImplementation(AStarPlanner):
    # TODO: implement your own version of preloop, step and postloop
    def preloop(self):
        # This is for illustrative purposes only, feel free to modify
        self.queue:set[PathNode] = set()
        self.g:dict[PathNode, float] = {}
        self.h:dict[PathNode, float] = {}
        self.visited_nodes:set[PathNode] = set()
        self.visited_nodes.add(self.start_node)
        self.queue.add(self.start_node)
        self.g[self.start_node] = 0
        self.h[self.start_node] = calculate_node_distance(
            self.start_node, 
            self.goal_node
        )
        print(f"{self.goal_node.coordinates=}")

    def step(self):
        # ===== some given data/parameters =====
        self.start_node
        self.goal_node
        self.world_map # bgr
        self.occupancy_map # bool
        self.goal_threshold
        self.grid_size # for sampling neighbor nodes
        # ==========
        # to sample neighbor nodes, use 
        # self.get_neighbor_nodes(current_node)
        if len(self.queue)==0:
            self.is_done.set() # only set this on termination
            return
        q=list(self.queue)
        gs=[self.g[x]+self.h[x] for x in q]
        idx=np.argmin(gs)
        current_node=q[idx]
        self.queue.remove(current_node)
        self.visited_nodes.add(current_node)
        #print(current_node.coordinates)
        if calculate_node_distance(current_node,self.goal_node)<self.goal_threshold:
            self.goal_node.parent=current_node
            self.goal_node.cost=current_node.cost+calculate_node_distance(current_node,self.goal_node)
            self.visited_nodes.add(self.goal_node)
            self.is_done.set() # only set this on termination
            return
        neighbors=self.get_neighbor_nodes(current_node)
        #print(len(neighbors))
        for neighbor in neighbors:
            if neighbor in self.visited_nodes:
                continue
            dist=self.g[current_node]+calculate_node_distance(current_node,neighbor)
            if neighbor in self.queue:
                if dist>=self.g[neighbor]:
                    continue
            else:
                self.queue.add(neighbor)
            neighbor.parent=current_node
            self.g[neighbor]=dist
            neighbor.cost=dist
            self.h[neighbor]=calculate_node_distance(neighbor,self.goal_node)


    def postloop(self):
        return (
            collect_path(self.goal_node), 
            self.visited_nodes# replace with set of visited nodes
        )