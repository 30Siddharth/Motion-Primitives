import sympy as sy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle 
import networkx as nx
import pdb
import scipy.integrate as integrate
import logging

import inspect
import json
import matplotlib as mpl
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt


import pdb
import numpy as np
from flightsim.simulate import Quadrotor, simulate, ExitStatus
from flightsim.world import World
from flightsim.crazyflie_params import quad_params
from proj1_3.code.occupancy_map import OccupancyMap
from proj1_3.code.se3_control import SE3Control
from pathlib import Path
from getSuccessors import successors
from itertools import product
import time
import pdb
import queue as Q
import pandas as pd  
  


class motion_primitive:
    
    def __init__(self,gs,dt,end_state,v_max,a_max):
        ### Constraints on Quadrotor dynamics ####
        self.v_max=v_max
        self.a_max=a_max
        
        self.dt=dt
        self.gs=gs
        self.discretize=9;
        self.start_state=  []
        self.ux = np.linspace(-10, 10, self.discretize)
        self.uy = np.linspace(-10, 10, self.discretize)  
        self.uz = np.linspace(-10, 10, self.discretize)
        self.motion_primitive=[]
        self.q = Q.PriorityQueue()
        self.end_state=end_state
    
    def robot_radius(self, rad):
        self.radius=rad
    
    def vel_max(self, vel):
        self.v_max=vel
    
    def a_max(self, acc):
        self.a_max=acc
    
    def u_max(self,u):
        self.u_max=u
    
    def generate_heuristic(self,start_state,end_state):
  
        cost=(start_state[0]-end_state[0])**2+(start_state[1]-end_state[1])**2+(start_state[2]-end_state[2])**2 
        
        if cost<20:
            cost=(np.sum(np.abs(start_state-end_state))**2)/3
        return cost
 
    def update_start_state(self,start_state):
        self.start_state=start_state

    def generate_motion_primitive(self):
        """
        generate motion primitive with forward simulation
        """
        N=self.discretize**3
        n=self.discretize
        for i in range(n):
            for j in range(n):
                for k in range(n):
                    self.motion_primitive.append(np.array([self.ux[i],self.uy[j],self.uz[k]]).reshape(1,3))
        pdb.set_trace()
        self.motion_primitive=np.array(self.motion_primitive)

      
    
    def check_dynamics(self,state):
        if all(abs(i) <=self.v_max for i in state[3:6]) and all(abs(j) <=self.a_max for j in state[6:9]):
            return True
        else:
            return False
        
    
    def get_successors(self):
        
        rr, cc = gs.reachable(self.motion_primitive,self.start_state,self.dt)
        N=self.discretize**3
        rr=np.array(rr).reshape(N,16)
        cc=np.array(cc)
       
        #pdb.set_trace()

        for i in range(N):
            try:
                if self.check_dynamics(rr[i,:]):
                    cc[i]=cc[i]/1000+ 10*self.generate_heuristic(rr[i,0:9],self.end_state[0:9])
                    self.q.put((cc[i],tuple(rr[i])))
                 
            except:
                i=0
        return rr,cc
        

class graph_construct:
    def __init__(self,gs):

        #variable time choice 
        self.del_t=0.1
        self.node_q=[]

        #starting and end position
        self.start_pos = np.array([0,0,0],dtype=np.float16).reshape(3,1)
        self.start_vel=np.array([0,0,0],dtype=np.float32)
        self.start_acc=np.array([0,0,0],dtype=np.float32)
        
        self.end_pos = np.array([10,10,10],dtype=np.float16).reshape(3,1)
        self.end_vel=np.array([0,0,0],dtype=np.float32)
        self.end_acc=np.array([0,0,0],dtype=np.float32)
        
        
        self.node_q.append(self.start_pos)
        self.time=0  



        # Initialize graph and queue of unexpanded nodes
        self.G = nx.DiGraph()

       
        self.gs=gs
    
    def discretize_3D(self):
            # Discretize the 3D sparce . 
        self.x_ = np.linspace(-10, 10,101)
        self.y_ = np.linspace(-10, 10, 101)
        self.z_ = np.linspace(-10, 10, 101)
        


    
    

    def generate_nodes(self,start_state,edge_cost,next_node,motion_primitive):
        # Construct motion primitive graph #
        # closest_pt_start=self.find_closest_3D(start_state[0:2])
        # closest_pt_next_arr=self.find_closest_3D(motion_primitve.forward_simulation(closest_pt_start))
        
        for i in range(edge_cost.size):
            if not self.G.has_node(tuple(next_node[i])):
                self.node_q.append(next_node[i])
                self.G.add_edge(tuple(start_state), tuple(next_node[i]), weight=edge_cost[i])
        

            # check if the new node added is goal node if yes stop the graph. Non optimal solution
               
        



    


 

if __name__ == "__main__":


    # Returns a motion primitive array"
    filename = '../util/test_maze.json'
    # Videoname = 'MyMap.mp4'

    # Load the test example.
    file = Path(inspect.getsourcefile(lambda:0)).parent.resolve() / '..' / 'util' / filename
    world = World.from_file(file)          # World boundary and obstacles.
    start_node  = world.world['start']          # Start point, shape=(3,)
    goal_node   = world.world['goal']           # Goal point, shape=(3,)
    # This object defines the quadrotor dynamical model and should not be changed.
    robot_radius = 0.25

    initial_state = np.array([start_node[0],start_node[1],start_node[2],0,0, 0, 0,0,0,0, 0, 0, 1, 0, 0, 0])
    goal_state = np.array([goal_node[0],goal_node[1],goal_node[2],0, 0,0, 0,0,0,0, 0, 0, 1, 0, 0, 0])
    #pdb.set_trace()
    gs = successors(world)
    x0 = np.array([1,2,3,0.1,0.1,0.1,0.001,0.001,0.001,0,0,0,1,0,0,0])
    um = np.array([[10,10,10],[20,20,20]]).reshape((2,1,3))
    tau =0.3
    max_v=1
    max_a=1

   
    
    start=time.perf_counter()
    graph_search=graph_construct(gs)
    graph_search.G.add_node(tuple(start_node))
    graph_search.G.add_node(tuple(goal_node))
    motion_primitive=motion_primitive(gs,tau,goal_state,max_v,max_a)
    motion_primitive.generate_motion_primitive()
    motion_primitive.update_start_state(initial_state)
    rr,cc=motion_primitive.get_successors()
    finish=time.perf_counter()
    print('time completed ',round(finish-start,2))
    graph_search.generate_nodes(start_node,cc,rr,motion_primitive)
   
 
 
    

    i=0
    #while motion_primitive.q.qsize() > 0:
    node=[]
    while True:
        node=motion_primitive.q.get()
        node=np.array(node)
        motion_primitive.update_start_state(node[1])
        rr,cc=motion_primitive.get_successors()
        if node[0]<7:
            break
      
        if graph_search.generate_nodes(node[1],cc,rr,motion_primitive):
            break
        if i%100==0:
            print(node[1])
            print("Iterator {} Length {}".format(i,motion_primitive.q.qsize()))
        i=i+1
    pdb.set_trace()
    path =nx.shortest_path(graph_search.G,tuple(start_node),tuple(node[1]),weight='cost')

    waypoints=[]
    for i in range(len(path)):
        waypoints.append(np.asarray(path[i][0:3]))
    print(waypoints)
    csvfile = open('waypoints.csv', 'wb')
    df = pd.DataFrame(waypoints)
    df.to_csv('waypoints.csv', index=False)