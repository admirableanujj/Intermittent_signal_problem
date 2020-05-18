import numpy as np
from random import gauss
import params
from numpy import cos, sin, tan, pi
import logging
import sys

class Sensor:

    def __init__(self):
        self.acc = 0.0
        self.pos = 0.0
        self.uvw = 0.0
        self.phi_theta_psi = 0.0
        self.pqr = 0.0
        self.acc_array = []
        self.pos_array = []
        self.uvw_array = []
        self.phi_theta_psi_array = []
        self.pqr_array = []
  
    def imu_model_dynamics(self, w1, w2, w3, w4, phi_theta_psi, t, pos, uvw, pqr, noise_flag, glb):
            m = params.m
            g = params.g
            l = params.l
            dt = params.dt
            Ixx = 2*m*l**2                 #% moment of inertia of x
            Iyy = 2*m*l**2                 #% moment of inertia of y
            Izz = 4*m*l**2                 #% moment of inertia of z
            Kf = 2.2*10**-4                #% constant to relate force and angular speed
            Km = 5.4*10**-6                #% constant to relate moment and angular speed
            phi = phi_theta_psi[0]
            theta = phi_theta_psi[1]
            psi  = phi_theta_psi[2]
            p = pqr[0]   
            q = pqr[1]    
            r = pqr[2]
            u = uvw[0]   
            v = uvw[1]    
            w = uvw[2]

            F1 = params.Kf*w1**2      
            F2 = params.Kf*w2**2       
            F3 = params.Kf*w3**2       
            F4 = params.Kf*w4**2
            F_total = F1 + F2+ F3 + F4
            # glb.state[15] = F_total
            # print(F_total)
            R_b_v = np.array([
                    [       cos(theta)*cos(psi),                         cos(theta)*sin(psi),                                   -sin(theta)],     
                    [sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),   sin(phi)*cos(theta)],
                    [cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),   cos(phi)*cos(theta)]])
                                    
            R_pqr = np.array([
                    [1,     sin(phi)*tan(theta),                   cos(phi)*tan(theta)],
                    [0,                cos(phi),                             -sin(phi)],
                    [0,     sin(phi)*(cos(theta)**-1),       cos(phi)*(cos(theta)**-1)]])

            
            disturbance  = [[0],[0],[0]]

            #   % now finally getting the actual accelerations the quad has
            # forces = np.array([[0],[0],[F_total]])
            # glb.state[15] = F_total 
            uvw_dot = np.array([[0], [0], [(1/m)*F_total]]) - np.array([[-g*sin(theta)],[g*cos(theta)*sin(phi)],[g*cos(theta)*cos(phi)]]) + np.array([[r*v - q*w],[p*w - r*u],[q*u - p*v]])
            
            # uvw_dot + disturbance
            # uvw_dot = foo2
            # print(uvw_dot)
            # print(np.shape(uvw_dot))
            
            # logging.info(f'IMU:pos :, {pos}, uvw :, {uvw}')
            # logging.info(f'IMU:w2: {w2}, w4: {w4}, w3: {w3}, w1: {w1}')

            ######################
            # Add Noise in the System.
            ###################### 
            if noise_flag:
                uvw_dot[0][0] = uvw_dot[0][0] + gauss(0,0.1)
                uvw_dot[1][0] = uvw_dot[1][0] + gauss(0,0.1)
                uvw_dot[2][0] = uvw_dot[2][0] + gauss(0,0.1)
                # uvw_dot[2][0] = uvw_dot[2][0] 
            ######END of Noise#########    
            uvw = uvw + np.transpose(uvw_dot*dt) + disturbance
            pos_dot = np.dot(uvw ,np.transpose(R_b_v) )
            # uvw = uvw + np.dot(np.transpose(uvw_dot*dt) ,np.transpose(R_b_v) )
            # # print(np.shape(uvw))
            # pos_dot = uvw
            pos = pos + pos_dot*dt
            # 
            #     
            ###########################        

            p_dot = (l*Kf*(w2**2 - w4**2) - q*r*(Izz-Iyy))/Ixx  #% Rotation along +ve Xb axis
            q_dot = (l*Kf*(w3**2 - w1**2) - p*r*(Ixx - Izz))/Iyy #% Rotation along +ve Yb axis
            r_dot = Km*(w1**2 - w2**2 + w3**2 - w4**2)/Izz  -p*q*(Iyy - Ixx)/Izz

            # ------------------------
            # logging.info(f'IMU:p_dot :, {p_dot}, q_dot :, {q_dot}, r_dot:, {r_dot}')
            # logging.info(f'IMU:w2: {w2}, w4: {w4}, w3: {w3}, w1: {w1}')
            
            pqr_dot = [p_dot, q_dot, r_dot]
            pqr = pqr + np.multiply(pqr_dot,params.dt)      #% approximate linearization;
            p = pqr[0]   
            q = pqr[1]    
            r = pqr[2]


            phi_theta_psi_dot = np.dot(R_pqr,pqr)       #% Rotation matrix for body to inertial frame
            phi_dot = phi_theta_psi_dot[0]   
            theta_dot = phi_theta_psi_dot[1]
            psi_dot = phi_theta_psi_dot[2]


            phi_theta_psi = phi_theta_psi + (phi_theta_psi_dot)*dt   #% approximate linearization
            phi   = phi_theta_psi[0]
            theta = phi_theta_psi[1]
            psi   = phi_theta_psi[2]

            if uvw.ndim > 1 :
                uvw = uvw[0]
            if pos.ndim > 1 :
                pos = pos[0]
            
            # logging.info(f'IMU:pos :, {pos}, uvw :, {uvw}, angles: {phi_theta_psi}')
            # print(uvw)
            
            return uvw_dot, pos, uvw, phi_theta_psi, pqr

    def gps_module(self, first_time_file_read, step_number, glb):
        global_vars = glb
        if first_time_file_read :
            self.pos_data = global_vars.file_read('pos_array')
            self.vel_array = global_vars.file_read('vel_array')
            self.phi_theta_psi_array = global_vars.file_read('phi_theta_psi_array')
            glb.first_time_file_read = False
        temp_pos = self.pos_data[step_number]
        temp_pos[0] = temp_pos[0] + gauss(0,2.5)
        temp_pos[1] = temp_pos[1] + gauss(0,2.5)
        temp_pos[2] = temp_pos[2] + gauss(0,2.5)
        # print(temp_pos)
        temp_vel = self.vel_array[step_number]
        temp_phi_theta_psi = self.phi_theta_psi_array[step_number]
        return temp_pos, temp_vel, temp_phi_theta_psi


            
        