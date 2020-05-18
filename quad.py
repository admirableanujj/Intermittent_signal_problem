import numpy as np
from random import gauss
import params
from numpy import cos, sin, tan, pi
import logging

class Quadcopter:

    def __init__(self):
        self.acc = []
        self.vel = []
        self.pos = []
    
    def add_noise_to_accelaration(self, acc):
        acc[0][0] = acc[0][0] + gauss(0,1)
        acc[1][0] = acc[1][0] + gauss(0,1)
        acc[2][0] = acc[2][0] + gauss(0,1)

        return [acc[0][0],acc[1][0],acc[2][0]]
    
    def forces(self, w1, w2, w3, w4):
        F1 = params.Kf*w1**2      
        F2 = params.Kf*w2**2       
        F3 = params.Kf*w3**2       
        F4 = params.Kf*w4**2
        F_total = F1 + F2+ F3 + F4
        return F_total

    def calculate_accelaration(self, R_b_v, F_total):
        forces = np.array([[0],[0],[F_total]])    
        acc = (1/params.m)*([[0],[0],[-params.m*params.g]] + np.dot(np.transpose(R_b_v),forces))
        return acc
    
    def calculate_velocity(self, acc, dt):
        vel = self.vel
        vel = vel + np.transpose(acc*dt)
        self.vel = vel  
    
    def quad_dynamics(self, w1, w2, w3, w4, phi_theta_psi, t, pos, vel, pqr, noise_flag, glb):
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

        F1 = params.Kf*w1**2      
        F2 = params.Kf*w2**2       
        F3 = params.Kf*w3**2       
        F4 = params.Kf*w4**2
        F_total = F1 + F2+ F3 + F4
        glb.state[15] = F_total
        R_b_v = np.array([
                [       cos(theta)*cos(psi),                         cos(theta)*sin(psi),                                   -sin(theta)],     
                [sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi),  sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),   sin(phi)*cos(theta)],
                [cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),  cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),   cos(phi)*cos(theta)]])
                                
        R_pqr = np.array([
                [1,     sin(phi)*tan(theta),                   cos(phi)*tan(theta)],
                [0,                cos(phi),                             -sin(phi)],
                [0,     sin(phi)*(cos(theta)**-1),       cos(phi)*(cos(theta)**-1)]])

        
        disturbance  = [0,0,0]

        #   % now finally getting the actual accelerations the quad has
        forces = np.array([[0],[0],[F_total]])
        # print('1: ', R_b_v)
        # print('2: ',np.transpose(R_b_v))
        # glb.state[15] = F_total 
        acc = (1/m)*([[0],[0],[-m*g]] + np.dot(np.transpose(R_b_v), forces))
        # acc = (1/m)*([[0],[0],[-m*g]] + np.dot(R_b_v, forces))
        # print(acc)
        # print(np.shape(acc))
        # print([acc[0][0], acc[1][0], acc[2][0]])

        # acc_array.insert(index, [acc[0][0], acc[1][0], acc[2][0]]) 
        # print('acc: ', np.transpose(acc), 'no_Noise_acc: ',  np.transpose(no_Noise_acc))
        
        ######################
        # Add Noise in the System.
        ###################### 
        # if noise_flag:
        #     acc[0][0] = acc[0][0] + gauss(0,1)
        #     acc[1][0] = acc[1][0] + gauss(0,1)
        #     acc[2][0] = acc[2][0] + gauss(0,1)
        ######END of Noise#########        
        ###########################        

        # glb.state[12] = acc[0][0]
        # glb.state[13] = acc[1][0]
        # glb.state[14] = acc[2][0]        
        
        
        # print(acc_array)
        # vel = vel + np.transpose(acc*dt) + disturbance + [gauss(-1,1), gauss(-1,1), gauss(-1,1)]
        vel = vel + np.transpose(acc*dt) + disturbance
        # no_noise_vel = no_noise_vel + np.transpose(no_Noise_acc*dt)

          
        pos = pos + vel*dt
        # no_noise_pos = no_noise_pos + no_noise_vel*dt
        # print('pos: ', pos, 'no_noise_pos: ', no_noise_pos)

        # p_dot = l*Kf*(w2**2 - w4**2) - q*r*(Izz-Iyy))/Ixx  #% Rotation along +ve Xb axis
        # q_dot = l*Kf*(w3**2 - w1**2) - p*r*(Ixx - Izz))/Iyy #% Rotation along +ve Yb axis
        # r_dot = np.nan_to_num(Km*(w1**2 - w2**2 + w3**2 - w4**2)/Izz  -p*q*(Iyy - Ixx))/Izz
        # ------------------------

        # w1 = self.motor_saturation(w1)
        # w2 = self.motor_saturation(w2)
        # w3 = self.motor_saturation(w3)
        # w4 = self.motor_saturation(w4)

        p_dot = (l*Kf*(w2**2 - w4**2) - q*r*(Izz-Iyy))/Ixx  #% Rotation along +ve Xb axis
        q_dot = (l*Kf*(w3**2 - w1**2) - p*r*(Ixx - Izz))/Iyy #% Rotation along +ve Yb axis
        r_dot = Km*(w1**2 - w2**2 + w3**2 - w4**2)/Izz  -p*q*(Iyy - Ixx)/Izz

        # ------------------------
        # print(f'p_dot :, {p_dot}, q_dot :, {q_dot}, r_dot:, {r_dot}')
        # logging.info(f'p_dot :, {p_dot}, q_dot :, {q_dot}, r_dot:, {r_dot}')
        # logging.info(f'w2: {w2}, w4: {w4}, w3: {w3}, w1: {w1}')
        
        pqr_dot = [p_dot, q_dot, r_dot]
        # print(pqr_dot)
        # print(dt)
        pqr = pqr + np.multiply(pqr_dot,params.dt)      #% approximate linearization;
        p = pqr[0]   
        q = pqr[1]    
        r = pqr[2]
        # glb.state[6] = p
        # glb.state[7] = q
        # glb.state[8] = r

        phi_theta_psi_dot = np.dot(R_pqr,pqr)       #% Rotation matrix for body to inertial frame
        phi_dot = phi_theta_psi_dot[0]   
        theta_dot = phi_theta_psi_dot[1]
        psi_dot = phi_theta_psi_dot[2]


        phi_theta_psi = phi_theta_psi + (phi_theta_psi_dot)*dt   #% approximate linearization
        phi   = phi_theta_psi[0]
        theta = phi_theta_psi[1]
        psi   = phi_theta_psi[2]
        # glb.state[3] = phi
        # glb.state[4] = theta
        # glb.state[5] = psi

        if vel.ndim > 1 :
            vel = vel[0]
        if pos.ndim > 1 :
            pos = pos[0]
        # if no_noise_pos.ndim > 1 :
        #     no_noise_pos = no_noise_pos[0]
        ######################
        # Implementing Kalman filter.
        ######################
        
        
        # pn = pos[1]    
        # pe = pos[0]     
        # pd = pos[2]  
        # glb.state[0] = pos[0]    
        # glb.state[1] = pos[1]     
        # glb.state[2] = pos[2] 
        # glb.state[9] =  vel[0]    
        # glb.state[10] = vel[1]     
        # glb.state[11] = vel[2]  
        # print('vel: ', vel)
        # print('pos: ', pos)
        # logging.info(f'Quad:pos :, {pos}, vel :, {vel}, angles: {phi_theta_psi}')
        return acc, pos, vel, phi_theta_psi, pqr
        
    def motor_saturation(self, w):
        if w >= 500:
            w = 500
        elif w <= 30:
             w = 30
        return w
 




