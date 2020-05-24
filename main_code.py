import math
import numpy as np 
from  position_control import Position_Control
from attitude_control import Attitude_control
# from mpmath import sec 
from numpy import cos, sin, tan, pi
import matplotlib.pyplot as plt
import params
from global_vars import global_vars
from mpl_toolkits.mplot3d import Axes3D
import logging
from random import seed
from random import gauss
from EKF import extended_Kalman_Filter
from quad import Quadcopter
from plot_graphs import Plot_graph
from plot_graphs2 import Plot_graph as ptg2
from sensor import Sensor
import csv
from neural_net import Neuralnet as nn
from nn_EKF import neural_net_based_EKF as nn_EKF
# from nn_EKF2 import neural_net_based_EKF as nn_EKF2
import torch


def motor_saturation(w):
        if w >= 500:
            w = 500
        elif w <= 30:
             w = 30
        return w

def angle_saturation(angle):
    if angle >= 0.1:
            angle = 0.1
    elif angle <= -0.1:
            angle = -0.1
    return angle



def file_write_csv(file_name, rows, fields):
    with open(file_name, 'w') as csvfile:  
        # creating a csv writer object  
        csvwriter = csv.writer(csvfile)  
        # writing the fields  
        csvwriter.writerow(fields)  
        # writing the data rows  
        csvwriter.writerows(rows) 

def file_read_csv(file_name):
    # reading csv file 
    rows = []
    with open(file_name, 'r') as csvfile: 
        # creating a csv reader object 
        csvreader = csv.reader(csvfile) 
        # extracting field names through first row 
        fields = next(csvreader) 
        # extracting each data row one by one 
        for row in csvreader: 
            rows.append(row) 
    return fields, rows


def main():
    # try:
        logging.basicConfig(filename='logger.log', level= logging.INFO)
        logging.info('Setting Parmerters')
        count = 0

        dt= params.dt
        m = params.m                      #% mass of the quadrotor
        l = params.l                      #% length of arm of the quadrotor
        g = params.g                      #% gravity
        i = params.i
        k = params.k
        index = params.index   

        Ixx = 2*m*l**2                 #% moment of inertia of x
        Iyy = 2*m*l**2                 #% moment of inertia of y
        Izz = 4*m*l**2                 #% moment of inertia of z

        Kf = 2.2*10**-4                #% constant to relate force and angular speed
        Km = 5.4*10**-6                #% constant to relate moment and angular speed

        # % Desired States
        psi_des = 0

        # % Initializing values
        pos = np.array([0,0,0])              
        vel = np.array([0,0,0])
        phi_theta_psi = np.array([0,0,0])
        pqr = np.array([0,0,0])
        phi = 0       
        theta = 0  
        psi = 0
        p   = 0
        q   = 0
        r   = 0

        ###############################
        # waypoints = np.array([[3, 6,  -1],
        #                       [17, 25, -34], 
        #                       [-60, 20, 60]])
        # waypoints = np.array([[3, 6,  -1],
        #                       [17*8, 25*6, 34*4], 
        #                       [-60, 20, 60]])
        # waypoints = np.array([
        #                       [17*4, 25*3, 34*1], 
        #                       [-60, 20, 60]])         #waypoint in cm
        waypoints = np.array([
                              [30, 30, 30], 
                              [30, 30, 30]])

        # % waypoints = [-12  45 -19];
        # t_sim  = 80
        factor = 100
        t_sim  = 20
        target_waypoint = 0

        logging.info('Setting Parmerters')

        sim_data = {}
        # ['no_noise', 'noise', 'ekf', 'neural_nets']
        for sim in ['no_noise','noise', 'ekf','neural_nets']:
            ekf = extended_Kalman_Filter()
            nn_ekf = nn_EKF()
            logging.info(f'{sim}')
            loss_val = 0
            glb = global_vars()
            # % Initializing values
            pos = np.array([0,0,0])              
            vel = np.array([0,0,0])
            phi_theta_psi = np.array([0,0,0])
            pqr = np.array([0,0,0])
            phi = 0       
            theta = 0  
            psi = 0
            p   = 0
            q   = 0
            r   = 0
            glb.error_x = 0
            glb.error_y = 0
            glb.error_z = 0
            glb.wh = math.sqrt(m*g/(4*Kf))   #Motor Speed
            acc_array = []
            quad = Quadcopter() 
            imu = Sensor()
            step_number = 0         
            s_intv = 30
            e_intv = 50
            glb.waypoints = waypoints
            glb.factor = factor
            ###############################
            for t in range(0, t_sim*factor, int(params.dt*factor)):
                # print(t)
                waypoint_desired = waypoints[i][0:3]
                # print(waypoint_desired)
                # if sim == 'ekf':
                #     print(abs(glb.error_x) + abs(glb.error_y) + abs(glb.error_z))
                # print(waypoint_desired)
                if(abs(glb.error_x) + abs(glb.error_y) + abs(glb.error_z)) < 0.3:
                    count = count + 1
                    # print(abs(glb.error_x) + abs(glb.error_y) + abs(glb.error_z))
                    # print(waypoint_desired)
                    if count > 400*params.dt*factor:
                        i = i + 1
                        # print(sim)
                        if i > target_waypoint:
                            i = target_waypoint
                        count = 0
                # % 1 - x axis
                # % 2 - y axis
                # % 3 - z axis
                # % =========== Position Control Loop  ===================%
                # print(pos, ' : ',vel)
                pc = Position_Control()
                acc_des   = pc.position_control(waypoint_desired, pos, vel, k, glb)
                phi_des   = (1/g)*(acc_des[0] * sin(psi_des) - acc_des[1] * cos(psi_des))  #% Inversion
                theta_des = (1/g)*(acc_des[0] * cos(psi_des) + acc_des[1] * sin(psi_des))
                delta_wf  = (m/(8*Kf*glb.wh)) * acc_des[2]  #% desired rotational speed to produce the net force in z direction 
                p_des = ((phi_des - phi)*0)/dt              #% assuming linear approximation 
                q_des = ((theta_des - theta)*0)/dt
                r_des = 0       
                phi_theta_psi  = [phi, theta, psi]
                p_q_r          = [p, q, r]
                angles_desired = [angle_saturation(phi_des), angle_saturation(theta_des), angle_saturation(psi_des)]
                angular_vel_required = [p_des, q_des, r_des]
                # print('phi_theta_psi: ', phi_theta_psi, 'p_q_r: ', p_q_r)
                # %  ==============   Attitude Control Loop   ============ %
                ac = Attitude_control()
                w_des = ac.attitude_control(phi_theta_psi, p_q_r, angles_desired, angular_vel_required, delta_wf, k, glb)
                w1 = motor_saturation(w_des[0])
                w2 = motor_saturation(w_des[1])
                w3 = motor_saturation(w_des[2])   
                w4 = motor_saturation(w_des[3])    
                k =  k + 1
                # %=======================END OF ATTITUDE CONTROL LOOP===========

                # %=======================END OF POSITION CONTROL LOOP==============
                # %%%%%% ==========  DYNAMICS OF THE QUADROTOR ==========  %%%%%%%%%

                if sim == 'noise' or sim == 'ekf' or sim =='neural_nets':
                # if sim == 'noise':                
                    _, qpos, qvel, qphi_theta_psi, qpqr = quad.quad_dynamics( w1, w2, w3, w4, phi_theta_psi, t, pos, vel, pqr, False, glb)
                    acc, pos, vel, phi_theta_psi, pqr = imu.imu_model_dynamics( w1, w2, w3, w4, phi_theta_psi, t, pos, vel, pqr, False, glb)  #This line used for measurement of GPS ##IMP##
                    imu.acc, imu.pos, imu.uvw, imu.phi_theta_psi, imu.pqr = imu.imu_model_dynamics( w1, w2, w3, w4, phi_theta_psi, t, pos, vel, pqr, True, glb)
                    # print(f'qpos: {pos} ,imu.pos:{imu.pos}')
                else:
                    acc, pos, vel, phi_theta_psi, pqr = quad.quad_dynamics( w1, w2, w3, w4, phi_theta_psi, t, pos, vel, pqr, False, glb)
                    imu.acc, imu.pos, imu.uvw, imu.phi_theta_psi, imu.pqr = imu.imu_model_dynamics( w1, w2, w3, w4, phi_theta_psi, t, pos, vel, pqr, False, glb)
                    if True:
                        acc = imu.acc
                        pos = imu.pos
                        vel = imu.uvw
                        phi_theta_psi = imu.phi_theta_psi
                        pqr = imu.pqr 

                # print(f'{qpos}')
                # logging.info(f'IMU: imu.acc: {imu.acc}, imu.pos:{imu.pos}, imu.uvw:{imu.uvw}, imu.phi_theta_psi:{imu.phi_theta_psi}, imu.pqr: {imu.pqr}')
                # logging.info(f'QUAD: acc: {acc}, pos:{pos}, vel:{vel}, phi_theta_psi:{phi_theta_psi}, pqr: {pqr}')
                # logging.info('---------------------------------')

                acc_array.insert(index, [acc[0][0], acc[1][0], acc[2][0]]) 
                p = pqr[0]   
                q = pqr[1]    
                r = pqr[2]
                phi   = phi_theta_psi[0]
                theta = phi_theta_psi[1]
                psi   = phi_theta_psi[2]
                
                #%%%%%%%%%%%%%%% END OF ANGLE UPDATE LOOP %%%%%%%

                ######################
                # Implementing IMU model.
                ######################
                if sim == 'noise':
                # if sim == 'noise' or 'ekf':
                    pos[0] =   imu.pos[0]
                    pos[1] =   imu.pos[1]
                    pos[2] =   imu.pos[2]
                    phi    =   imu.phi_theta_psi[0]
                    theta  =   imu.phi_theta_psi[1]
                    psi    =   imu.phi_theta_psi[2]
                    vel[0] =   imu.uvw[0]
                    vel[1] =   imu.uvw[1]
                    vel[2] =   imu.uvw[2]
                    p      =   imu.pqr[0]
                    q      =   imu.pqr[1]
                    r      =   imu.pqr[2]

                #%%%%%%%%%%%%%%% END OF IMU IMPLEMENTATION %%%%%%%
                glb.state[6] = p
                glb.state[7] = q
                glb.state[8] = r
                ######################
                # Implementing Kalman filter.
                ######################
                if sim == 'ekf' or sim == 'neural_nets':
                    time_intv = range(int(s_intv), int(e_intv))
                    temp_pos, temp_vel, temp_phi_theta_psi = imu.gps_module(qpos, qvel, qphi_theta_psi)

                    ekf_flag = True
                    glb.state[0] = imu.uvw[0]  
                    glb.state[1] = imu.uvw[1]
                    glb.state[2] = imu.uvw[2]
                    glb.state[3] = imu.phi_theta_psi[0]
                    glb.state[4] = imu.phi_theta_psi[1]
                    glb.state[5] = imu.phi_theta_psi[2]
                    glb.state[6] = imu.pqr[0]
                    glb.state[7] = imu.pqr[1]
                    glb.state[8] = imu.pqr[2]
                    gps = [temp_pos[0], temp_pos[1], temp_pos[2] , temp_vel[0], temp_vel[1], temp_vel[2], temp_phi_theta_psi[0], temp_phi_theta_psi[1], temp_phi_theta_psi[2]]

                    if t in time_intv:
                        # print(time_intv)
                        # print(f's_intv:{s_intv},e_intv:{e_intv},t:{t}')
                        # gps = [glb.gps[0], glb.gps[1], glb.gps[2], glb.gps[3], glb.gps[4], glb.gps[5], glb.gps[6], glb.gps[7], glb.gps[8]]
                        # gps = [temp_pos[0], temp_pos[1], temp_pos[2], temp_vel[0], temp_vel[1], temp_vel[2], temp_phi_theta_psi[0], temp_phi_theta_psi[1], temp_phi_theta_psi[2]]
                        # gps = [0, 0, 0, temp_vel[0], temp_vel[1], temp_vel[2], temp_phi_theta_psi[0], temp_phi_theta_psi[1], temp_phi_theta_psi[2]]
                        gps_nn = [imu.pos[0], imu.pos[1], imu.pos[2], imu.uvw[0], imu.uvw[1], imu.uvw[2], imu.phi_theta_psi[0], imu.phi_theta_psi[1], imu.phi_theta_psi[2]]
                        ekf_states, _ = ekf.cal_kalman_gain(dt, [], t, glb)
                        # Signal after every 3 sec
                        if t > e_intv-2:
                            s_intv = e_intv + 1
                            e_intv = s_intv + factor
                        if sim == 'neural_nets':
                            nn_ekf_states, _ , loss_val = nn_ekf.cal_kalman_gain(dt, nn_obj, ann_input, gps_nn, np.array(gps), False, glb)

                    else:
                        # print(time_intv)
                        gps = [temp_pos[0], temp_pos[1], temp_pos[2] , temp_vel[0], temp_vel[1], temp_vel[2], temp_phi_theta_psi[0], temp_phi_theta_psi[1], temp_phi_theta_psi[2]]
                        # gps = [0, 0, 0, 0, 0, 0, 0, 0, 0]
                        gps_nn = [imu.pos[0], imu.pos[1], imu.pos[2], imu.uvw[0], imu.uvw[1], imu.uvw[2], imu.phi_theta_psi[0], imu.phi_theta_psi[1], imu.phi_theta_psi[2]]
                        # fields = ['px', 'py', 'pz', 'dpx', 'dpy', 'dpz', 'phi', 'theta', 'psi', 'ax', 'ay', 'az', 'w1', 'w2', 'w3','w4']
                        ekf_states, _ = ekf.cal_kalman_gain(dt, gps, t, glb)
                        if sim == 'neural_nets':
                            nn_ekf_states, _ , loss_val = nn_ekf.cal_kalman_gain(dt, nn_obj, ann_input, gps_nn, np.array(gps), True, glb)
                        # print(f'pos: {pos}, ekf_states: {ekf_states[0][0:3]}, t:{t}')

                    if ekf_states.ndim > 1 :
                        ekf_states = ekf_states[0]
                    glb.state_ekf[0]  =  ekf_states[0]
                    glb.state_ekf[1]  =  ekf_states[1]
                    glb.state_ekf[2]  =  ekf_states[2]
                    glb.state_ekf[3]  =  ekf_states[3]
                    glb.state_ekf[4]  =  ekf_states[4]
                    glb.state_ekf[5]  =  ekf_states[5]
                    glb.state_ekf[6]  =  ekf_states[6]
                    glb.state_ekf[7]  =  ekf_states[7]
                    glb.state_ekf[8]  =  ekf_states[8]
                        # --------------------------
                    # pos[0] =   glb.state_ekf[0]
                    # pos[1] =   glb.state_ekf[1]
                    # pos[2] =   glb.state_ekf[2]
                    # phi    =   glb.state_ekf[6]
                    # theta  =   glb.state_ekf[7]
                    # psi    =   glb.state_ekf[8]
                    # vel[0] =   glb.state_ekf[3]
                    # vel[1] =   glb.state_ekf[4]
                    # vel[2] =   glb.state_ekf[5]
                    # print(glb.state_ekf)
                    # --------------------------
                else:
                    ekf_flag = False

                #%%%%%%%%%%%%%%% END OF KALMAN IMPLEMENTATION %%%%%%%
                #%%%%%%%%%%%%%%% Nueral net IMPLEMENTATION %%%%%%%
                if sim == 'neural_nets':
                    ekf_flag = True
                    if nn_ekf_states.ndim > 1 :
                        nn_ekf_states = nn_ekf_states[0]
                    glb.state_ekf[0]  =  nn_ekf_states[0]
                    glb.state_ekf[1]  =  nn_ekf_states[1]
                    glb.state_ekf[2]  =  nn_ekf_states[2]
                    glb.state_ekf[3]  =  nn_ekf_states[3]
                    glb.state_ekf[4]  =  nn_ekf_states[4]
                    glb.state_ekf[5]  =  nn_ekf_states[5]
                    glb.state_ekf[6]  =  nn_ekf_states[6]
                    glb.state_ekf[7]  =  nn_ekf_states[7]
                    glb.state_ekf[8]  =  nn_ekf_states[8]
                        


                #%%%%%%%%%%%%%%% END OF Nueral net test IMPLEMENTATION %%%%%%%
                if sim == 'ekf' or sim == 'neural':
                    # print(qpos)
                    glb.qpos_array.insert(index, qpos)
                glb.w_des_array.insert(index, [w1, w2, w3, w4])
                glb.error_x_array.insert(index, glb.error_x)
                glb.error_y_array.insert(index,  glb.error_y)
                glb.error_z_array.insert(index, glb.error_z)
                glb.error_x_virtual_array.insert(index, glb.error_x_virtual)
                glb.error_y_virtual_array.insert(index, glb.error_y_virtual)
                glb.error_z_virtual_array.insert(index, glb.error_z_virtual)
                
                glb.error_xdes_array.insert(index, 0)
                glb.error_ydes_array.insert(index, 0)
                glb.error_zdes_array.insert(index, 0)
                
                imu.acc_array.insert(index, imu.acc)
                imu.pos_array.insert(index, imu.pos)
                imu.uvw_array.insert(index, imu.uvw)
                imu.phi_theta_psi_array.insert(index, imu.phi_theta_psi)
                imu.pqr_array.insert(index, imu.pqr)

                index = index + 1
                step_number = step_number + 1
                # print(glb.state_ekf)
                # ---------------for Neural net
                row = [pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], phi, theta, psi, acc[0][0], acc[1][0], acc[2][0], w1, w2, w3, w4,p, q, r, t]
                # qacc, qpos, qvel, qphi_theta_psi, qpqr = quad.quad_dynamics( w1, w2, w3, w4, phi_theta_psi, t, pos, vel, pqr, False, glb)
                # row = [pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], phi, theta, psi, w1, w2, w3, w4]
                ann_input = np.array(row)
                # ---------------for Neural net
                # fields = ['px', 'py', 'pz', 'dpx', 'dpy', 'dpz', 'phi', 'theta', 'psi', 'ax', 'ay', 'az', 'w1', 'w2', 'w3','w4']
                glb.rows.insert(t, row)
                glb.loss_val.insert(index, loss_val)
                
                if sim == "ekf" or sim == 'neural':
                    if ekf_states.ndim > 1 :
                        ekf_states = ekf_states[0]
                    # print(f'qpos: {qpos},state_ekf{glb.state_ekf[0:3]}')
                    pos[0] =   glb.state_ekf[0]
                    pos[1] =   glb.state_ekf[1]
                    pos[2] =   glb.state_ekf[2]
                    # phi    =   glb.state_ekf[6]
                    # theta  =   glb.state_ekf[7]
                    # psi    =   glb.state_ekf[8]
                    # vel[0] =   glb.state_ekf[3]
                    # vel[1] =   glb.state_ekf[4]
                    # vel[2] =   glb.state_ekf[5]



            ################ Compiling data from all runs ###########
            # error_xyzdes_array = [glb.error_xdes_array, glb.error_ydes_array, glb.error_zdes_array]
            temp_dict = {sim: { 'glb.error_xdes_array' : glb.error_xdes_array,
                                'glb.error_ydes_array' : glb.error_ydes_array,
                                'glb.error_zdes_array' : glb.error_zdes_array,
                                'glb.error_x_array' : glb.error_x_array,
                                'glb.error_y_array' : glb.error_y_array,
                                'glb.error_z_array' : glb.error_z_array,
                                'glb.error_x_virtual_array' : glb.error_x_virtual_array,
                                'glb.error_y_virtual_array' : glb.error_y_virtual_array,
                                'glb.error_z_virtual_array' : glb.error_z_virtual_array,
                                'glb.phi_theta_psi_array' : glb.phi_theta_psi_array,
                                'glb.angles_desired_array' : glb.angles_desired_array,
                                'glb.angular_vel_required_array' : glb.angular_vel_required_array,
                                'glb.p_q_r_array' : glb.p_q_r_array,
                                'glb.position_final_required_array' : glb.position_final_required_array,
                                'glb.pos_array' : glb.pos_array,
                                'glb.vel_final_required_array' : glb.vel_final_required_array,
                                'glb.vel_array' : glb.vel_array,
                                'acc_array' : acc_array,
                                'glb.w_des_array': glb.w_des_array,
                                'imu.acc_array': imu.acc_array,
                                'imu.pos_array': imu.pos_array,
                                'imu.uvw_array': imu.uvw_array,
                                'imu.phi_theta_psi_array': imu.phi_theta_psi_array,
                                'imu.pqr_array': imu.pqr_array,
                                'glb.state_ekf': glb.state_ekf,
                                'glb.qpos_array': glb.qpos_array}}
            sim_data.update(temp_dict)
            glb.write_flag = False
            # print(valset)
            # print(glb.rows)
            if glb.kalman_gain_array:
                global_vars.file_write('kalman_gain_array', glb.kalman_gain_array)
            if glb.p_matrix_array:
                global_vars.file_write('p_matrix_array', glb.p_matrix_array)

            if sim == 'ekf':
                # global_vars.file_write( 'pos_array', glb.pos_array)
                # global_vars.file_write( 'vel_array', glb.vel_array)
                # global_vars.file_write( 'phi_theta_psi_array', glb.phi_theta_psi_array)
                # ---------------------------
                fields = ['px', 'py', 'pz', 'dpx', 'dpy', 'dpz', 'phi', 'theta', 'psi', 'ax', 'ay', 'az', 'w1', 'w2', 'w3','w4','p', 'q', 'r', 't']
                rows = glb.rows
                file_name = 'train_data.csv'
                file_write_csv(file_name, rows, fields)
                fields, neu_rows = file_read_csv(file_name)
                x_in, y_in = nn.data_in_prrocessor(neu_rows)
                nn_obj = nn(x_in, y_in, False)  #Creates nueral net object with Training
                count = 0
                y_in_array = []
                y_pred_array = []
                loss_array = []
                for x in x_in:
                    y_pred = nn_obj.neural_net_predict(x)
                    loss = nn_obj.neural_net_update(torch.from_numpy(y_in[count]), y_pred, count)
                    y_in_array.append(y_in)
                    y_pred_array.append(y_pred)
                    loss_array.append(loss.detach().numpy().tolist())
                    count += 1
                    # logging.info(f'y_pred: {y_pred} \t y_in: {y_in[count]}')
                global_vars.file_write( 'test_loss_array', loss_array)
                # logging.info('y_in')
                # for i1 in y_in_array:
                #     logging.info(i1)
                # logging.info('y_pred_array')
                # for i1 in y_pred_array:
                #     logging.info(i1)
            logging.info('---------------------------------')
                
                # print(obj)
            # print(list(sim_data.keys()))
            #######################FEEDBACK################################
            # pos[0] =   pos[0]
            # pos[1] =   pos[1]
            # pos[2] =   pos[2]
            # phi    =   phi_theta_psi[0]
            # theta  =   phi_theta_psi[1] 
            # psi    =   phi_theta_psi[2]
            # vel[0] =   vel[0]
            # vel[1] =   vel[1]
            # vel[2] =   vel[2]
            ###############################################################
            
        ################ Compiling data from all runs ###########
        # print(sim_data)
        py = Plot_graph()
        py2 =  ptg2()
        py2.plot_graphs(acc_array, sim_data, ekf_flag, glb)
        py.plot_graphs(acc_array, sim_data, ekf_flag, glb)          
        logging.info('All is well that ends well.')
    # except Exception as e:
    #     print(f'Error:{e}')


main()
