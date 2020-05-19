import numpy as np 

class Position_Control:

    


    def __init__(self):
        self.position_final_required_array =[]
        self.vel_final_required_array = []
        self.pos_array = []
        self.vel_array = []
        
    def position_control(self, waypoint_desired, pos, vel, k, global_vars, *args):
        
        # global position_final_required_array
        # global vel_final_required_array
        # global pos_array
        # global vel_array
        # global error_x_virtual,  error_y_virtual,  error_z_virtual
        # global error_x,          error_y,          error_z
        # global error_vel_x,      error_vel_y,      error_vel_z
        # global Kp_x,             Kd_x    
        glb = global_vars

        pos_des = (waypoint_desired)
        vel_des = np.array([0, 0, 0])
        acc_final_desired = np.array([0, 0, 0])
        # glb.Kp_x = 0.195
        # glb.Kd_x = 1.50
        # Kp_y = 0.195
        # Kd_y = 1.50
        # Kp_z = 0.155
        # Kd_z = 1.8
        glb.Kp_x = 0.195
        glb.Kd_x = 1.5
        Kp_y = 0.195
        Kd_y = 1.50
        Kp_z = 0.155
        Kd_z = 1.8

        

        # % % Good gains
        # % Kp_x = 0.01;            Kd_x = 0.2;
        # % Kp_y = 0.01;            Kd_y = 0.2;
        # % Kp_z = 3.5;             Kd_z = 30;


        # % % Ideal Base gains
        # % Kp_x = 0.001;        Kd_x = 0.02;
        # % Kp_y = 0.001;        Kd_y = 0.02;
        # % Kp_z = 0.5;          Kd_z = 8;
        Kp = np.array([glb.Kp_x, Kp_y, Kp_z])
        Kd = np.array([glb.Kd_x, Kd_y, Kd_z])
        # print('pos: ', pos, 'pos_des: ', pos_des)
        glb.error_x = pos_des[0] - pos[0]
        glb.error_y = pos_des[1] - pos[1]
        glb.error_z = pos_des[2] - pos[2]
        error = [glb.error_x, glb.error_y, glb.error_z]
        # % Saturation cap for position control of 11 units
        # print(glb.error_x)
        for no in range(0,3):
            if abs(error[no]) > 11:
                error[no] = 11 * np.sign(error[no])
        glb.error_x_virtual = error[0]
        glb.error_y_virtual = error[1]
        glb.error_z_virtual = error[2]

        ##########################################
        glb.gps[0] = error[0]
        glb.gps[1] = error[1]
        glb.gps[2] = error[2]
        ##########################################

        glb.error_vel_x = vel_des[0] - vel[0]
        glb.error_vel_y = vel_des[1] - vel[1]
        glb.error_vel_z = vel_des[2] - vel[2]
        error_vel = [glb.error_vel_x, glb.error_vel_y, glb.error_vel_z]

        ##########################################
        glb.gps[3] = error_vel[0]
        glb.gps[4] = error_vel[1]
        glb.gps[5] = error_vel[2]
        ##########################################
        # % Notice here that acceleration/Force is the control input for finally 
        # %conmtrolling position. So there is a good chance that a PD Controller 
        # % will work.
        # % Position Control. Deciding acceleration to be given position based on position error 
        acc_dec = acc_final_desired +  Kp*error + Kd*error_vel
        ######################Class Variable
        self.position_final_required_array.insert(k,np.transpose(pos_des))
        self.vel_final_required_array.insert(k, np.transpose(vel_des))
        self.pos_array.insert(k,np.transpose(pos))
        self.vel_array.insert(k, np.transpose(vel))
        #####################GLobal Variable
        glb.position_final_required_array.append(np.transpose(pos_des))
        glb.vel_final_required_array.append(np.transpose(vel_des))
        glb.pos_array.append(np.transpose(pos))
        glb.vel_array.append(np.transpose(vel))

        return acc_dec


