import sympy as sy


class linearize:
    def __init__(self):
        self.g=9.8

    
    def linearize(self):
        A=[[0,0,0,1,0,0,0,0,0,0,0,0],
           [0,0,0,0,1,0,0,0,0,0,0,0],
           [0,0,0,0,0,1,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0],
           [0,0,0,0,0,0,0,0,0,0,0,0], 
           [0,0,0,0,0,0,0,0,0,0,0,0],]

class optimality:
    def __init__(self):
        pass
    
    def optimal_path_cost(self)
        
        """
        1. Optimality: how close the cost of an opti- mal (e.g., shortest) path in the lattice is to the truly optimal path in the continuum
        2. Completeness: the degree to which a given search space approaches the capacity to ex- press all available motions
        3. Complexity: how much computation is re- quired to solve a particular planning query
        """
        pass

class motion_primitive:
    
    def __init__(self,start_pos,goal_pos ):
        ### Constraints on Quadrotor dynamics ####
        self.v_max
        self.a_max
        self.u_max
        self.Dt

        ###start parameters end parameters###
        ### currently start and goal velocity, acceleration and jerk have a target to be 0 ###
        self.start_pos=start_pos
        self.start_vel=np.array([0,0,0],dtype=np.float32)
        self.start_acc=np.array([0,0,0],dtype=np.float32)
        self.start_jrk=np.array([0,0,0],dtype=np.float32)

        ###start parameters end parameters###
        self.goal_pos=goal_pos
        self.goal_vel=np.array([0,0,0],dtype=np.float32)
        self.goal_acc=np.array([0,0,0],dtype=np.float32)
        self.goal_jrk=np.array([0,0,0],dtype=np.float32)

        
        self.mu=mu
        self.radius=0.0
        pass
    
    def discretize_U(self):
        return np.linspace(self.u_max, -self.u_max,self.mu)

    def robot_radius(self, rad):
        self.radius=rad
    
    def vel_max(self, vel):
        self.v_max=vel
    
    def a_max(self, acc):
        self.a_max=acc
    
    def u_max(self,u):
        self.u_max=u


class jerk_controller:

    def __init__(self):
        
        

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



if __name__ == "__main__":
    print('hello')
    ### set map ####

    ### set limits ###

    ### set start and end goals ###


    ### Get trajectory ###

    ### Trajectory tracking ###