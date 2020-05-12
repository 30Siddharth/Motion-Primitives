import numpy as np
from scipy.spatial.transform import Rotation
import math as m
import pdb


class SE3Control(object):
    """

    """
    def __init__(self, quad_params):
        """
        This is the constructor for the SE3Control object. You may instead
        initialize any parameters, control gain values, or private state here.

        For grading purposes the controller is always initialized with one input
        argument: the quadrotor's physical parameters. If you add any additional
        input arguments for testing purposes, you must provide good default
        values!

        Parameters:
            quad_params, dict with keys specified by crazyflie_params.py

        """

        # Quadrotor physical parameters.
        self.mass            = quad_params['mass'] # kg
        self.Ixx             = quad_params['Ixx']  # kg*m^2
        self.Iyy             = quad_params['Iyy']  # kg*m^2
        self.Izz             = quad_params['Izz']  # kg*m^2
        self.arm_length      = quad_params['arm_length'] # meters
        self.rotor_speed_min = quad_params['rotor_speed_min'] # rad/s
        self.rotor_speed_max = quad_params['rotor_speed_max'] # rad/s
        self.k_thrust        = quad_params['k_thrust'] # N/(rad/s)**2
        self.k_drag          = quad_params['k_drag']   # Nm/(rad/s)**2

        # You may define any additional constants you like including control gains.
        self.inertia = np.diag(np.array([self.Ixx, self.Iyy, self.Izz])) # kg*m^2
        self.g = 9.81 # m/s^2
        
   
        # Declaring constants:
        Arml  = self.arm_length
        gamma = self.k_drag/self.k_thrust
        Gamma_inv  = np.array([[1,1,1,1],
                               [0,Arml,0,-Arml],
                               [-Arml,0,Arml,0],
                               [gamma,-gamma,gamma,-gamma]])
        self.Gamma_inv = np.linalg.inv(Gamma_inv)


        # Tuning from ideal response for linear motion
        zeta = 1.0*np.array([1, 1, 1]) # desired damping ratio
        percent = 0.01 * np.array([2, 2, 2]) # desired error margin
        ts = 0.99*np.array([1.2, 1.2, 1.0]) # desired settling time
        wn = np.divide(np.divide(-np.log(percent), zeta), ts) # natural frequency

        self.K_p = np.multiply(np.identity(3), np.multiply(wn, wn))
        self.K_d = np.multiply(np.identity(3), 2*wn*zeta)

        # Tuning from ideal response for angular motion
        zeta = 1.0*np.array([1, 1, 1]) # desired damping ratio
        percent = 0.01 * np.array([2, 2, 2]) # desired error margin
        ts = np.array([0.05, 0.05, 0.1]) # desired settling time
        wn = np.divide(np.divide(-np.log(percent), zeta), ts) # natural frequency

        self.K_rot = np.multiply(np.identity(3), np.multiply(wn, wn))
        self.K_omega = np.multiply(np.identity(3), 2*wn*zeta) * 2.5


    def update(self, t, state, flat_output):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys
                cmd_motor_speeds, rad/s
                cmd_thrust, N (for debugging and laboratory; not used by simulator)
                cmd_moment, N*m (for debugging; not used by simulator)
                cmd_q, quaternion [i,j,k,w] (for laboratory; not used by simulator)
        """
      
        cmd_motor_speeds = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.zeros((4,))
    

        # Rotation
        # pdb.set_trace()
        state['x']=state['x'].reshape((729,3))
        state['v']=state['v'].reshape((729,3))
        state['q']=state['q'].reshape((729,4))
        state['w']=state['w'].reshape((729,3))
        #pdb.set_trace()
        q = state['q']
        R = Rotation.from_quat(q).as_matrix()  # <---------------------<< Identifying Rotation
        b3 = np.matmul(R,np.array([0,0,1]))
        
   
        # Desired Acceleration
        err_v = state['v'] - flat_output['x_dot']
        err_x = state['x'] - flat_output['x']
        r_des = flat_output['x_ddot'] - np.matmul(np.array(state['v'] - flat_output['x_dot']),np.array(self.K_p)) - np.matmul(np.array(state['x'] - flat_output['x']),np.array(self.K_d))
        #r_des = np.diagonal(r_des)
    
        # Required Force
        F_des = self.mass* (r_des + np.array([0,0,self.g]))
        # for i,_ in enumerate(F_des):
        #     if F_des[i] < 0:
        #         F_des[i] = 0
                # F_des = self.mass*(np.array([0,0,self.g]))
        
        # Required sum of Forces
        u1 = np.matmul(F_des,b3.T).diagonal()
        

        ## **** Computing U2 *****

        # Required Yaw Rotation
        o = np.zeros(F_des.shape)
        if (F_des == o).all():
            b3_des=np.zeros(F_des.shape)
            b3_des[:,2] = 1
        else:
            b3_des = F_des/np.linalg.norm(F_des)        
        
        a_psi = np.array([np.cos(np.array(flat_output['yaw'])), np.sin(np.array(flat_output['yaw'])), np.zeros((729,))]).T
        b2_des = np.cross(b3_des,a_psi,axisa=1,axisb=1)
        b2_des = b2_des/np.linalg.norm(b2_des)
        b1_des = np.cross(b2_des,b3_des)
        # b1_des = b1_des/np.linalg.norm(b1_des)
        R_des = np.array([b1_des.T,b2_des.T, b3_des.T]).T

      
        # Error in rotation
        err_rot = 0.5*(np.matmul(R_des.transpose(0,2,1),R) - np.matmul(R.transpose(0,2,1),R_des))
        err_rot = np.array([-err_rot[:,1,2], err_rot[:,0,2], -err_rot[:,0,1]]).T
     
        

        # Error in angular velocity
        err_omega = -(flat_output['yaw_dot'] - state['w'])

        U2 = np.matmul(-np.matmul(err_rot,self.K_rot) - np.matmul(err_omega,self.K_omega),self.inertia)  #<---------------------<<<
        U2 = np.array([U2[:,0], U2[:,1], U2[:,2]]).T

        
        # Total Input 
        U = np.array([u1,U2[:,0],U2[:,1],U2[:,2]])
        forces = np.matmul(self.Gamma_inv,U)
        forces= np.where(forces <0,0, forces)
       

        cmd_thrust = u1
        cmd_moment = U2
        cmd_motor_speeds = forces/self.k_thrust
        cmd_motor_speeds = np.sqrt(cmd_motor_speeds)
        cmd_motor_speeds= np.where(cmd_motor_speeds >self.rotor_speed_max,self.rotor_speed_max, cmd_motor_speeds)

        
        cmd_q = Rotation.from_matrix(R_des).as_quat()

       

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q}
       
        return control_input
