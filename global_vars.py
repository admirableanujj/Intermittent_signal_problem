import numpy as np
class global_vars:

    def __init__(self):
        self.wh = 0
        self.error_x =  0           
        self.error_y =  0        
        self.error_z =  0
        self.error_x_virtual  =  0  
        self.error_y_virtual  =  0
        self.error_z_virtual  =  0
        self.error_vel_x   =  0  
        self.error_vel_y   =  0 
        self.error_vel_z   =  0
        self.error_x_array = []
        self.error_y_array = []
        self.error_z_array = []
        self.error_x_virtual_array = []
        self.error_y_virtual_array = []
        self.error_z_virtual_array = []
        self.error_xdes_array = []
        self.error_ydes_array = []
        self.error_zdes_array = []
        self.acc = []

        # % Defining global array variables for attitude control
        self.phi_theta_psi_array =  []
        self.p_q_r_array =  []
        self.angles_desired_array =  []
        self.angular_vel_required_array =  []
        self.kp_theta = []
        self.kd_theta = []

        # % Defining global array variables for position control
        self.position_final_required_array =  []
        self.vel_final_required_array =  []
        self.pos_array =  []
        self.vel_array =  []
        self.Kp_x = 0
        self.Kd_x = 0
        
        # % Defining global array variables for drone dynamics without noise.
        self.state = np.zeros(16)
        self.state_ekf = np.zeros(16)
        
        # % Defining global array variables for plotting state after EKF.
        self.pos_array_ekf =  []
        self.vel_array_ekf =  []
        self.phi_theta_psi_array_ekf =  []
        self.no_noise_pose = []
        self.w_des_array = []
        self.first_time_file_read = True
        # x, y, z, phi, theta, psi, 
        self.gps = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        # % NeuralNets
        self.rows= []
        self.loss_val = []
        self.p_matrix_array = []
        self.kalman_gain_array = []
        self.waypoints = []

    @staticmethod
    def file_write(file_name, data_set):
        f = open(f'.\\temp\\glb_temp_{file_name}', 'w')
        for data in data_set:
            # print(list(data))
            try:
                f.write(str(list(data))+'\n')
            except:
                f.write(str(data)+ '\n')
        f.close()

    @staticmethod
    def file_read(file_name):
        f = open(f'.\\temp\\glb_temp_{file_name}', 'r')
        lines = f.readlines()
        f.close()
        temp = []
        for line in lines:
            foo = line.replace('[','').replace(']','').replace('\n','').replace('\'','')
            foo = foo.split(',')
            temp.append([float(foo[0]), float(foo[1]), float(foo[2])])
            # print(float(foo[0]))    
        # print(temp)
        return temp
    
    @staticmethod
    def file_read2(file_name):
        f = open(f'.\\temp\\glb_temp_{file_name}', 'r')
        lines = f.readlines()
        f.close()
        temp = []
        for line in lines:
            foo = line.replace('[','').replace(']','').replace('\n','').replace('\'','')
            foo = foo.split(',')
            # print(foo)
            temp.append([float(x) for x in foo])
            # temp.append([float(foo[0]), float(foo[1]), float(foo[2])])
            # print(float(foo[0]))    
        # print(temp)
        return temp