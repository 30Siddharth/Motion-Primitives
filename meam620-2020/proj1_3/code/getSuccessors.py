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



class successors(object):

    def __init__(self,world):
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.25 
        self.occ_map = OccupancyMap(world, self.resolution, self.margin)
        self.D = np.zeros((4,3))
        self.traj = trajectory(self.D)

    def reachable(self, Um, x_init, tau):
        # TODO
        '''
        Given an initial state and set of motion primitives return the 
        set of coeffecients which give all the reachable states
        * Also check for collsions

        Input
        Um  = N X 1 X 3 np array of motion primitives
        tau = duration of motion primitive
        x_init  = 1 X 17 array of initial state [x0, v0, a0, q0, w0]
            p0 = 1 X 3 -- postion in m
            v0 = 1 X 3 -- velocity in m/s
            a0 = 1 X 3 -- acceleration in m/s^2
            q0 = 1 X 4 -- quaternion 
            w0 = 1 X 3 -- angular velocity in 1/s^2

        Output
        Rs = N X 3 X 3 np array  of the reachable set from the initial point given the Um
        Cs = N X 1 np array of the cost of all the reachable points
        '''

        N = Um.shape[0]
        Rs = np.zeros((N,1,16))
        Cs = np.zeros((N,1))
        J0 = Um
        x0 = x_init
        
        for i in range(N):
            collision_flag = False
            # TODO check for collision using occupancy map
            self.D = np.append(x0[0:9],J0[i,:,:]).reshape((4,3))
            xf = self.D[0,:] + self.D[1,:]*tau + 0.5*self.D[2,:]*tau**2 + (1/6)*self.D[3,:]*tau**3
            collision_flag = self.collision_check(xf)
            if collision_flag is True:
                Cs[i] = np.inf
            else:
                Rs[i,:,:], Cs_err = self.forward_simulate(x0, J0[i,:,:], tau)
                # The total cost of the 
                Cs[i] = np.sum(np.abs(J0[i,:,:])) + tau + Cs_err 
        
        return Rs, Cs

    def forward_simulate(self, x0, J0, tau):
        # TODO
        '''
        Given the desired trajectory run a forward simulation to identify
        if the desired trajectory can be followed and identify the dynamic 
        feasibilty based on the ability of the controller to follow the trajectory 
        

        Input:
        x0 = 1 X 17 array of initial state [x0, v0, a0, q0, w0]
            p0 = 1 X 3 -- postion in m
            v0 = 1 X 3 -- velocity in m/s
            a0 = 1 X 3 -- acceleration in m/s^2
            q0 = 1 X 4 -- quaternion 
            w0 = 1 X 3 -- angular velocity in 1/s^2
        tau = Duration of planner

        Output:
        Rs =  3 X 3 np array of state at the end of the period after the simulation
        err_cs = M X 1 np array of the cost of the trajectory based on the controller's
                 ability to follow the trajectory
        '''
        #TODO update self.D
        t_final = tau
        initial_state  = {'x': tuple(x0[0:3]),
                          'v': tuple(x0[3:6]),
                          'q': tuple(x0[9:13]),
                          'w': tuple(x0[13:16])}
        
        quadrotor = Quadrotor(quad_params)
        my_se3_controller = SE3Control(quad_params)

        # Traj object has been created to main tain consistency with the simulator which
        # needs the trajectory  to be an ibject with an update function

        self.traj.update_D(self.D)

        (sim_time, state, control, flat, exit) = simulate(initial_state,
                                                          quadrotor,  
                                                          my_se3_controller, 
                                                          self.traj,         
                                                          t_final)
        
        if exit.value == 'Timeout: Simulation end time reached.':
            err = state['x'] - flat['x']  # TODO check if order is stored in the same order in both dictionary
            err_cs = np.sum(np.absolute(err))
            Rsx = state['x'][-1]
            Rsv = state['v'][-1]
            Rsa = J0*tau + x0[6:9]
            Rsq = state['q'][-1]
            Rsw = state['w'][-1]
            # pdb.set_trace()
            Rsp = np.array([Rsx, Rsv, Rsa.reshape((3,))]).reshape((1,9))
            Rst = np.concatenate([Rsq, Rsw]).reshape((1,7))
            # pdb.set_trace()
            Rs = np.append(Rsp,Rst)
            # pdb.set_trace()


        else:
            err_cs = np.inf
            Rs = None

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

class trajectory(object):
    def __init__(self, D):
        
        # D = 3 X 4 np array of the coeffecients of the polynomial
        '''
        D = [[x0,y0,z0],
             [vx0,vy0,vz0],
             [ax0,ay0,az0],
             [jx0,jy0,jz0]]
        '''
        self.D = D
        
    def update_D(self,D):
        self.D=D


    def update(self,t):
        
        '''
        Takes in the time t and returns the flat outputs based on the coefficient of the 
        polynomial for the trajetory
        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        '''
        
        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        D=self.D
        if t!=np.inf:
            x       = D[0,:] + D[1,:]*t + 0.5*D[2,:]*t**2 + (1/6)*D[3,:]*t**3
            x_dot   = D[1,:] + D[2,:]*t + 0.5*D[3,:]*t**2
            x_ddot  = D[2,:] + D[3,:]*t
            x_dddot = D[3,:]
        else:
            x=D[0,:]

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output


if __name__ == "__main__":
    # Choose a test example file. You should write your own example files too!
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
    tau = 1
    rr, cc = gs.reachable(um,x0,tau)
    print(rr)
    print(cc)


