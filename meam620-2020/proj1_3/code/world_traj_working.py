import numpy as np
import math as m
from proj1_3.code.graph_search import graph_search

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
        # print(points.shape)
        # print(points)
        path = points[0].reshape(1,3)
        goal_flag = False 
        j = 0   
        while goal_flag is False:
            # print(len(points))
            for i in range(len(points)-2):
                # print(len(points))
                # print(i)
                # print(path[j])
                # print(points)
                if all(points[i-j+2] == self.Goal):
                    goal_flag = True
                else:
                    m1 = points[i-j+1] - points[i-j]
                    m2 = points[i-j+2] - points[i+1-j]
                    # print(all(m1==m2))
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
        # del_t = 2
        # Time = np.arange(0,60,del_t)
        # Time = Time[:l]
        

        Speed = 2.5
        Dis_diff = np.diff(path, axis=0)
        # Sign = np.sign(Dis_diff)
        # # Velocity = np.array([np.multiply(Speed,Sign[i]) for i in range(len(Sign))])
        normx = np.array([np.linalg.norm(Dis_diff[i]) for i,_ in enumerate(Dis_diff)])
        l = 1/normx
        dir_cos = (Dis_diff.T*l).T
        Velocity = dir_cos*Speed
        normv = np.linalg.norm(Velocity, axis = 1)
        delTime = normx/normv
        Time = np.array([np.sum(delTime[:i]) for i in range(len(delTime)+1)])
        # print('TIme Differences',delTime)
        # print('Time', Time)
        # print('Velocity', Velocity)
        # print('Points', self.points)
        # print('Displacemnt', Dis_diff)

        pos = 0

        for i in  range(len(Time)-1):
            if t == np.inf:
                x = path[-1]
            elif Time[i] < t <= Time[i+1]:
                pos = i
                x_dot = Velocity[pos]
                x = x_dot*(t-Time[i]) + path[pos]
                # print('X--> ',x)
                # print('V--> ',x_dot)           
            elif Time[-1] <= t:
                x = path[-1]
            else:
                pass
        # print('X--> ',x)
        # print('V--> ',x_dot)

        flat_output = { 'x':x, 'x_dot':x_dot, 'x_ddot':x_ddot, 'x_dddot':x_dddot, 'x_ddddot':x_ddddot,
                        'yaw':yaw, 'yaw_dot':yaw_dot}
        return flat_output

        #  l = np.array([np.sum(time[:i]) for i in range(len(time)+1)])
        # sp = np.array([np.multiply(speed,sign[i]) for i in range(len(sign))])




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


