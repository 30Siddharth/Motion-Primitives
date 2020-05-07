import numpy as np
from flightsim.simulate import Quadrotor, simulate, ExitStatus
from flightsim.world import World

from proj1_3.code.occupancy_map import OccupancyMap
from proj1_3.code.se3_control import SE3Control



class successors(object):

    def __init__(self,world):
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.25 
        self.occ_map = OccupancyMap(world, self.resolution, self.margin)

    def reachable(self, Um, x0, tau):
        # TODO
        '''
        Given an initial state and set of motion primitives return the 
        set of coeffecients which give all the reachable states
        * Also check for collsions

        Input
        Um  = N X 3 X 1 np array of motion primitives
        tau = duration of motion primitive
        x0  = 3 X 3 array of initial state [x0, v0, a0]
            p0 = 1 X 3 -- postion in m
            v0 = 1 X 3 -- velocity in m/s
            a0 = 1 X 3 -- acceleration in m/s^2

        Output
        Rs = N X 3 X 3 np array  of the reachable set from the initial point given the Um
        Cs = N X 1 np array of the cost of all the reachable points
        '''

        N = Um.shape[0]
        Rs = np.zeros((N,3,3))
        Cs = np.zeros((N,1))
        J0 = Um
        for i in range(N):
            collision_flag = False
            D = np.append(x0, J0[i])
            # TODO check for collision using occupancy map
            xf = D[0] + D[1]*tau + 0.5*D[2]*tau**2 + (1/6)*D[3]*tau**3
            collision_flag = self.collision_check(xf)
            if collision_flag is True:
                Cs[i] = np.inf
            else:
                Rs[i,:,:], Cs_err = self.forward_simulate(x0, D, tau)
                # The total cost of the 
                Cs[i] = J0[i] + tau + Cs_err 

    def forward_simulate(self, x0, D, tau):
        # TODO
        '''
        Given the desired trajectory run a forward simulation to identify
        if the desired trajectory can be followed and identify the dynamic 
        feasibilty based on the ability of the controller to follow the trajectory 
        

        Input:
        D = M X 4 X 3 np array of coeffecients [jerk, acceleration, velocity, position]
            of the collision free trajectories
        x0 = 3 X 3 np array of initial state []
        tau = Duration of planner

        Output:
        Rs = M X 3 X 3 np array of state at the end of the period after the simulation
        err_cs = M X 1 np array of the cost of the trajectory based on the controller's
                 ability to follow the trajectory
        '''
        

        return Rs, err_cs

    def collision_check(self, x):
        '''
        Given a point in metric coordinates, identify if it is collision free
        
        Input:
        x = np array 3 X 1 of the (x,y,z) positions
        
        Ouput:
        Flag: True or False based on collision
        '''
        idx = self.occ_map.metric_to_index(x)
        
        if idx in self.occ_map.map[:]:
            return True
        else:
            return False

    

