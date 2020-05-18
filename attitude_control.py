import numpy as np
import params
from global_vars import global_vars

class Attitude_control:

    def __init__(self):
        self.phi_theta_psi_array = []
        self.p_q_r_array = []
        self.angles_desired_array = []
        self.angular_vel_required_array = []
    
    def attitude_control(self, phi_theta_psi, p_q_r, angles_desired, angular_vel_required, delta_wf,k, global_vars):
        glb = global_vars
        phi   = phi_theta_psi[0]
        theta = phi_theta_psi[1]
        psi   = phi_theta_psi[2]

        p = p_q_r[0]   
        q = p_q_r[1]
        r = p_q_r[2]

        phi_des   = angles_desired[0] 
        theta_des = angles_desired[1] 
        psi_des   = angles_desired[2]

        ##########################################
        glb.gps[6] = angles_desired[0]
        glb.gps[7] = angles_desired[1]
        glb.gps[8] = angles_desired[2]
        ##########################################

        p_des     = angular_vel_required[0]
        q_des     = angular_vel_required[1]
        r_des     = angular_vel_required[2]

        # % % Good values
        # % kp_phi   = 0.135;           kd_phi   = 6.5;
        # % kp_theta = 0.135;           kd_theta = 6.5;
        # % kp_psi   = 0.33;            kd_psi  =  1.2;


        # kp_phi   = 29           
        # kd_phi   = 19.5
        # glb.kp_theta = 29           
        # glb.kd_theta = 19.5
        # kp_psi   = 1               
        # kd_psi  =  3.6
        kp_phi   = 40           
        kd_phi   = 15
        glb.kp_theta = 40           
        glb.kd_theta = 15
        kp_psi   = 1               
        kd_psi  =  3.6


        # %%%%%%  Good gains   %%%%%%%
        # % kp_phi   = 0.135;           kd_phi   = 6.5;
        # % kp_theta = 0.135;           kd_theta = 6.5;
        # % kp_psi   = 0.33;            kd_psi  =  1.2;


        # % % Ideal gains - base gains
        # % kp_phi   = 0.013;           kd_phi   = 0.26;
        # % kp_theta = 0.013;           kd_theta = 0.26;
        # % kp_psi   = 0.03;            kd_psi   = 0.01;
        # % Actual attitude control. Deciding rotor speeds based on error in angle
        delta_w_phi =   kp_phi  * (phi_des-phi)    + kd_phi  *(p_des-p)
        delta_w_theta = glb.kp_theta*(theta_des-theta) + glb.kd_theta*(q_des-q)
        delta_w_psi   = kp_psi  *(psi_des-psi)    + kd_psi  *(r_des-r)
        
        # % now finding desired rotor speeds (w1_des, w2_des, w3_des, w4_des)
        w1_des = (glb.wh + delta_wf) - delta_w_theta + delta_w_psi 
        w2_des = (glb.wh + delta_wf) + delta_w_phi   - delta_w_psi 
        w3_des = (glb.wh + delta_wf) + delta_w_theta + delta_w_psi 
        w4_des = (glb.wh + delta_wf) - delta_w_phi   - delta_w_psi 
        
        w_des = [w1_des, w2_des, w3_des, w4_des]

        # print('phi_theta_psi: ', phi_theta_psi)
        # print(self.phi_theta_psi_array)
        #####Declared for class
        self.phi_theta_psi_array.append(phi_theta_psi)
        self.p_q_r_array.append(p_q_r)
        self.angles_desired_array.append(angles_desired)
        self.angular_vel_required_array.append(angular_vel_required)
        #####Declared for Global
        glb.phi_theta_psi_array.append(phi_theta_psi)
        glb.p_q_r_array.append(p_q_r)
        glb.angles_desired_array.append(angles_desired)
        glb.angular_vel_required_array.append(angular_vel_required)

        return w_des