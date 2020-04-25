import numpy as np
import math as m
from proj1_3.code.graph_search import graph_search
import matplotlib.pyplot as plt
import pdb


class WorldTraj(object):
    """

    """
    # def __init__(self, points):
    #     """
    #     This is the constructor for the Trajectory object. A fresh trajectory
    #     object will be constructed before each mission. For a waypoint
    #     trajectory, the input argument is an array of 3D destination
    #     coordinates. You are free to choose the times of arrival and the path
    #     taken between the points in any way you like.

    #     You should initialize parameters and pre-compute values such as
    #     polynomial coefficients here.

    #     Inputs:
    #         points, (N, 3) array of N waypoint coordinates in 3D
    #     """

    #     # STUDENT CODE HERE
    #     self.points = points
    def __init__(self, world, start, goal):
        """
        This is the constructor for the trajectory object. A fresh trajectory
        object will be constructed before each mission. For a world trajectory,
        the input arguments are start and end positions and a world object. You
        are free to choose the path taken in any way you like.

        You should initialize parameters and pre-compute values such as
        polynomial coefficients here.

        Parameters:
            world, World object representing the environment obstacles
            start, xyz position in meters, shape=(3,)
            goal,  xyz position in meters, shape=(3,)

        """

        # You must choose resolution and margin parameters to use for path
        # planning. In the previous project these were provided to you; now you
        # must chose them for yourself. Your may try these default values, but
        # you should experiment with them!
        self.resolution = np.array([0.25, 0.25, 0.25])
        self.margin = 0.5

        # You must store the dense path returned from your Dijkstra or AStar
        # graph search algorithm as an object member. You will need it for
        # debugging, it will be used when plotting results.
        self.path = graph_search(world, self.resolution, self.margin, start, goal, astar=False)

        # You must generate a sparse set of waypoints to fly between. Your
        # original Dijkstra or AStar path probably has too many points that are
        # too close together. Store these waypoints as a class member; you will
        # need it for debugging and it will be used when plotting results.
        # self.points = np.zeros((1,3)) # shape=(n_pts,3)

        self.points = self.path
        self.Goal = goal


        

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

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
        """


        points = self.points
        path = points[0].reshape(1,3)
        goal_flag = False 
        j = 0   
        while goal_flag is False:
            for i in range(len(points)-2):
                if all(points[i-j+2] == self.Goal):
                    goal_flag = True
                else:
                    m1 = points[i-j+1] - points[i-j]
                    m2 = points[i-j+2] - points[i+1-j]
                    if all(m1==m2) is False:
                        path = np.append(path,points[i-j+1,:].reshape(1,3),axis=0)
                        points = points[i-j+1:,:]
                        j = i+1
                

        path = np.append(path[:-1,:],self.Goal.reshape(1,3),axis=0)


        x        = np.zeros((3,))
        x_dot    = np.zeros((3,))
        x_ddot   = np.zeros((3,))
        x_dddot  = np.zeros((3,))
        x_ddddot = np.zeros((3,))
        yaw = 0
        yaw_dot = 0

        # STUDENT CODE HERE
        print('t',t)

        '''
        I first store all the points in an array. 
        I make another array of time stamps, each spaced at some predefined $\delta t$. 
        Then I slice this new vector of time stamps and choose only the points corresponding to the points. This way each trajectory between any two adjacent point is fixed.
        Then we compute the velocity vector between any two adjacent point.
        We use linear interpolation to travel between these points.

        Advantages: 
        -- The quadrotor will always be facing in the right direction. 
        -- The farther the points, the higher the speed

        Disadvantages:
        -- If the points are close enough then this might be slow
        '''
        

        l = len(self.points)


        Speed = 2.5
        Dis_diff = np.diff(path, axis=0)
        normx = np.array([np.linalg.norm(Dis_diff[i]) for i,_ in enumerate(Dis_diff)])
        l = 1/normx
        dir_cos = (Dis_diff.T*l).T
        Velocity = dir_cos*Speed
        normv = np.linalg.norm(Velocity, axis = 1)
        delTime = normx/normv
        Time = np.array([np.sum(delTime[:i]) for i in range(len(delTime)+1)])

        time = np.linspace(0,20,len(path)+1)

        td = np.diff(time)
        td = td.reshape(len(td),1)
        
        i = np.array([0,1]).reshape(1,2)
        seg_time = td*i
        seg_time = seg_time.reshape(2*len(seg_time),1)

    # Position Constraints
        
        blks_pos = np.hstack((seg_time**3,seg_time**2,seg_time**1,seg_time**0))
        pos_blks = np.zeros((3,blks_pos.shape[0],blks_pos.shape[1]))
        pos_blks[0,:,:] = blks_pos
        pos_blks[1,:,:] = blks_pos
        pos_blks[2,:,:] = blks_pos

        Apos = np.zeros((3,2*len(path),4*len(path)))
        for idx in range(len(path)):
            i = 2*idx
            if idx ==0:
                Apos[:,i:i+2,-4*(idx+1):] = pos_blks[:,i:i+2,:]
            elif idx == len(path)-1:
                Apos[:,i:,-4*(idx+1):-4*(idx+1)+4] = pos_blks[:,i:,:]
            else:
                Apos[:,i:i+2,-4*(idx+1):-4*(idx+1)+4] = pos_blks[:,i:i+2,:]
   
        waypoint = np.zeros((3,len(path),1))
        waypoint[0,:,:] = path[:,0].reshape(len(path),1)
        waypoint[1,:,:] = path[:,1].reshape(len(path),1)
        waypoint[2,:,:] = path[:,2].reshape(len(path),1)

        # Avel Velocity Constraints
        
        blks_vel = np.hstack((3*seg_time[:-1]**2,2*seg_time[:-1]**1,seg_time[:-1]**0,np.zeros(seg_time[:-1].shape)))
        # blks_vel = np.hstack((3*seg_time[1:]**2,2*seg_time[1:]**1,seg_time[1:]**0,np.zeros(seg_time[1:].shape)))
        vel_stack = np.zeros((3,blks_vel.shape[0],blks_vel.shape[1]))
        vel_stack[0,:,:] = blks_vel
        vel_stack[1,:,:] = blks_vel
        vel_stack[2,:,:] = blks_vel

        Avel = np.zeros((3,2*len(path)-2,4*len(path)))
        print(Avel.shape)
        for idx in range(len(path)-2):
            i = 2*idx
            if idx ==0:
                Avel[:,i:i+2,-4*(idx+1):] = vel_stack[:,i:i+2,:]
            elif idx == len(path)-1:
                Avel[:,i:,-4*(idx+1):-4*(idx+1)+4] = vel_stack[:,i:,:]
            else:
                Avel[:,i:i+2,-4*(idx+1):-4*(idx+1)+4] = vel_stack[:,i:i+2,:] 

        Avel = Avel[:,1:,:]
        Avel = Avel[:,1::2,:] - Avel[:,0:-1:2,:]
        Avel[:,-1,6] = 1 
        Avel_end = np.zeros((3,1,Avel.shape[2]))
        Avel_end[:,0,0] = 3*seg_time[-1]**2
        Avel_end[:,0,1] = 2*seg_time[-1]
        Avel_end[:,0,2] = 1
        Avel_start = np.zeros((3,1,Avel.shape[2]))
        Avel_start[:,0,-2] = 1



        # Accleration Constraints
        blks_acc = np.hstack((6*seg_time[:-1]**1,2*seg_time[:-1]**0,np.zeros(seg_time[:-1].shape),np.zeros(seg_time[:-1].shape)))
        # blks_acc = np.hstack((6*seg_time[1:]**1,2*seg_time[1:]**0,np.zeros(seg_time[1:].shape),np.zeros(seg_time[1:].shape)))
        acc_stack = np.zeros((3,blks_acc.shape[0],blks_acc.shape[1]))
        acc_stack[0,:,:] = blks_acc
        acc_stack[1,:,:] = blks_acc
        acc_stack[2,:,:] = blks_acc

        Aacc = np.zeros((3,2*len(path)-2,4*len(path)))
        for idx in range(len(path)-2):
            i = 2*idx
            if idx ==0:
                Aacc[:,i:i+2,-4*(idx+1):] = acc_stack[:,i:i+2,:]
            elif idx == len(path)-1:
                Aacc[:,i:,-4*(idx+1):-4*(idx+1)+4] = acc_stack[:,i:,:]
            else:
                Aacc[:,i:i+2,-4*(idx+1):-4*(idx+1)+4] = acc_stack[:,i:i+2,:] 

        Aacc = Aacc[:,1:,:]
        Aacc = Aacc[:,1::2,:] - Aacc[:,0:-1:2,:]
        Aacc[:,-1,5] = 2

        Aacc_end = np.zeros((3,1,Aacc.shape[2]))
        Aacc_end[:,0,0] = 6*seg_time[-1]
        Aacc_end[:,0,1] = 2
        Aacc_start = np.zeros((3,1,Aacc.shape[2]))
        Aacc_start[:,0,-3] = 2

        # A = np.zeros((3,4*len(path),4*len(path)))
        A = np.concatenate((Apos,Avel,Aacc,Aacc_end,Aacc_start,Avel_end,Avel_start),axis=1)
        B = np.zeros((3,4*len(path),1))
        K = np.matmul(np.linalg.inv(A),B)

        p = plt.figure(0)
        plt.imshow(A[0,:,:])
        p1 = plt.figure(1)
        plt.imshow(A[1,:,:])
        p2 = plt.figure(2)
        plt.imshow(A[2,:,:])
        plt.show()
        

        pos = 0

        for i in  range(len(Time)-1):
            if t == np.inf:
                x = path[-1]
            elif Time[i] < t <= Time[i+1]:
                pos = i
                x_dot = Velocity[pos]
                x = x_dot*(t-Time[i]) + path[pos]        
            elif Time[-1] <= t:
                x = path[-1]
            else:
                pass
        # print('X--> ',x)
        # print('V--> ',x_dot)

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output





    # def line(self,t1,t2,z1,z2):
    #     a = (z2-z1)/(t2-t1)
    #     b = z1 - t1*a

    #     return a,b
                

        # t0 = 0.1
        # if t < t0:
        #     x = self.points[0]
        #     if l > 1:
        #         if self.points[-1]==self.points[0] and self.points[0]==self.points[1]:
        #             x = self.points[0]
        #         else:
        #             x = self.points[0]

        # elif t0 <= t < t0 + 3:
        #     if self.points[0]==self.points[1] and self.points[1]==self.points[2]:
        #             x = self.points[1]
        #     else:
        #         a,b = self.line(t0,t0+3,self.points[0], self.points[1])
        #         x_dot = a
        #         x = a*t + b
        #         print('X', x)
        #         print('vel', x_dot)

        # elif t0 + 3 <= t < t0 + 6:
        #     if self.points[1]==self.points[2] and self.points[2]==self.points[3]:
        #             x = self.points[1]
        #     else:
        #         a,b = self.line(t0,t0+3,self.points[0], self.points[1])
        #         x_dot = a
        #         x = a*t + b
        #         print('X', x)
        #         print('vel', x_dot)


        # elif t0 + 6 <= t < t0 + 9:
        #     if self.points[2]==self.points[3] and self.points[3]==self.points[4]:
        #             x = self.points[1]
        #     else:
        #         a,b = self.line(t0,t0+3,self.points[0], self.points[1])
        #         x_dot = a
        #         x = a*t + b
        #         print('X', x)
        #         print('vel', x_dot)













        # elif t0 + 3 <= t < t0 + 6:
        #     a,b = self.line(t0+3,t0+6,self.points[1], self.points[2])
        #     x_dot = a
        #     x = a*t + b
        #     print('X', x)
        #     print('vel', x_dot)
        # elif t0+6 <= t < t0+9:
        #     a,b = self.line(t0+6,t0+9,self.points[2], self.points[3])
        #     x_dot = a
        #     x = a*t + b
        #     print('X', x)
        #     print('vel', x_dot)
        # else:
        #     x = self.points[-1]
        #     print('X', x)
        #     print('vel', x_dot)

        # points = self.points
        # l = len(points)
        # i = -l
        
        # if i < -1:
        #     print(points[i])
        #     i = i+1  


