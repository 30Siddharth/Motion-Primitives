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
  

class path_search:
    def __init__(self):
        pass
    
    def optimal_path(self):
        
        """
        1. Optimality: how close the cost of an opti- mal (e.g., shortest) path in the lattice is to the truly optimal path in the continuum
        2. Completeness: the degree to which a given search space approaches the capacity to ex- press all available motions
        3. Complexity: how much computation is re- quired to solve a particular planning query
        """



        pass

class motion_primitive:
    
    def __init__(self,gs,dt):
        ### Constraints on Quadrotor dynamics ####
        self.v_max=0
        self.a_max=0
        
        self.dt=dt
        self.gs=gs
        self.discretize=11;
        self.start_state=  np.array([1,2,3,0.1,0.1,0.1,0.001,0.001,0.001,0,0,0,1,0,0,0])
        self.ux = np.linspace(-30, 30, self.discretize)
        self.uy = np.linspace(-30, 30, self.discretize)  
        self.uz = np.linspace(-30, 30, self.discretize)
        self.motion_primitive=[]
    
    def robot_radius(self, rad):
        self.radius=rad
    
    def vel_max(self, vel):
        self.v_max=vel
    
    def a_max(self, acc):
        self.a_max=acc
    
    def u_max(self,u):
        self.u_max=u
    
    def generate_heuristic(start_state,end_states):
        np.linang.norm((start_state-end_states))

    def update_start_state(self,start_state):
        self.start_state=start_state

    def generate_motion_primitive(self):
        """
        generate motion primitive with forward simulation
        """
        N=self.discretize**3
        self.motion_primitive=np.array(list(product(self.ux,self.uy,self.uz))).reshape((N,1,3))

    def get_successors(self):
        rr, cc = gs.reachable(self.motion_primitive,self.start_state,self.dt)
        rr=np.array(rr)
        cc=np.array(cc)
        print(rr.shape)
        print(cc.shape)
        

class graph_construct:
    def __init__(gs):

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

        # Discretize the states and actions
        self.discretize()

        # Initialize graph and queue of unexpanded nodes
        self.G = nx.DiGraph()

        self.G.add_node(tuple(self.start_pos.reshape(4)))
        self.G.add_node(tuple(self.end_pos.reshape(4)))
        self.gs=gs
    

    def generate_nodes(self,start_state,state_space,motion_primitive):
        # Construct motion primitive graph #
        closest_pt_start=state_space.find_closest_3D(start_state[0:2])
        closest_pt_next_arr=state_space.find_closest_3D(motion_primitve.forward_simulation(closest_pt_start))
        
        for closest_pt_next in closest_pt_next_arr:
            if not self.G.has_node(tuple(closest_pt_next)):
                self.node_q.append(closest_pt_next)
                self.G.add_edge(tuple(closest_pt_start), tuple(closest_pt_next), weight=cost,control=control)

            # check if the new node added is goal node if yes stop the graph. Non optimal solution
               
        

class state_space:
    """
    pos - 3D position, x,y,z, theta,phi, psi
    vel - 3D velocity
    acc - 3D jerk
    Reference code doesnt seem to have 6D. Thats strance
    Jerk control so doesnt come in state space i guess rather on x_dot
    """
    def __init__(self, pos,vel,acc):
        self.pos=pos
        self.vel=vel
        self.jerk=jerk
        self.x_
        self.y_
        self.z_
        self.u1
        self.u2
        self.u3
        self.u4
    


    def discretize_3D(self):
        # Discretize the 3D sparce . 
        self.x_ = np.linspace(-10, 10,101)
        self.y_ = np.linspace(-10, 10, 101)
        self.z_ = np.linspace(-10, 10, 101)
        


        
    

    def find_closest_3D(self, pos):
        # Find the closest point 
        x = pos[0,:]
        y= pos[1,:]
        z = pos[2,:]
       

        closest_pt = np.zeros(pos.shape)  
        idx1=np.searchsorted(self.x_,x)
        prev_idx_is_less1 = ((idx1 == len(self.x_))|(np.fabs(x - self.x_[np.maximum(idx1-1, 0)]) < np.fabs(x - self.x_[np.minimum(idx1, len(self.x_)-1)])))
        idx1[prev_idx_is_less1] -= 1
        pos[0,:]= self.x_[idx1]

        idx2=np.searchsorted(self.y_,y)
        prev_idx_is_less2 = ((idx2 == len(self.y_))|(np.fabs(y - self.y_[np.maximum(idx2-1, 0)]) < np.fabs(y - self.y_[np.minimum(idx2, len(self.y_)-1)])))
        idx2[prev_idx_is_less2] -= 1
        pos[1,:]= self.x_[idx2]


        idx3=np.searchsorted(self.z_,z)
        prev_idx_is_less3 = ((idx3 == len(self.z_))|(np.fabs(z - self.y_[np.maximum(idx3-1, 0)]) < np.fabs(z - self.z_[np.minimum(idx3, len(self.z_)-1)])))
        idx3[prev_idx_is_less3] -= 1
        pos[2,:]= self.x_[idx3]

        return pos

if __name__ == "__main__":


    # Returns a motion primitive array"
    filename = '../util/test_maze.json'
    # Videoname = 'MyMap.mp4'

    # Load the test example.
    file = Path(inspect.getsourcefile(lambda:0)).parent.resolve() / '..' / 'util' / filename
    world = World.from_file(file)          # World boundary and obstacles.
    start  = world.world['start']          # Start point, shape=(3,)
    goal   = world.world['goal']           # Goal point, shape=(3,)
    # This object defines the quadrotor dynamical model and should not be changed.
    robot_radius = 0.25

    #TODO: Check this if reqd
    initial_state = {'x': start,
				 'v': (0, 0, 0),
 				 'q': (0, 0, 0, 1), # [i,j,k,w]
				 'w': (0, 0, 0)}

    gs = successors(world)
    x0 = np.array([1,2,3,0.1,0.1,0.1,0.001,0.001,0.001,0,0,0,1,0,0,0])
    um = np.array([[10,10,10],[20,20,20]]).reshape((2,1,3))
    tau = 0.1
    rr, cc = gs.reachable(um,x0,tau)
    

    motion_primitive=motion_primitive(gs,tau)
    motion_primitive.generate_motion_primitive()
    motion_primitive.get_successors()
   # graph_search=graph_construct(gs)
   # state_space=state_space()
    

#     i=0
#     while len(graph_search.node_q)>0:
#     """ chose node based on A* not simply pop stuff 
#         use priority queue """
#       node=graph_search.node_q.pop(0)
#       if graph_search.generate_nodes(node,tate_space,smotion_primitive):
#           break
      
#       if i%5000==0:
#           print("Iterator {} Length {}".format(i,len(graph_search.node_q)))
#       i=i+1

#   path =nx.shortest_path(graph_search.G,tuple(graph_search.start_state,tuple(node),weight='cost')