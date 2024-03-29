# -*- coding: utf-8 -*-
"""
For the one who always believed in me Ankiit Saxena
author: Anujj Saxena
email: saxenaaj@mail.uc.edu
Please feel free to use and modify this, but keep the above information. Thanks!
"""
################################################################
# KALMAN FILTER:
# It is an iterative mathematical process that uses a set of equations and consecutive data inputs to quickly 
# estimate the true value, position, velocity of the object being measured. When the measured values contain
# unpredicted or random error, uncertainity or variation. 
################################################################
import numpy as np
from numpy import sin, cos, tan, pi, sign
from scipy.integrate import ode
from global_vars import global_vars
import params
import logging
import torch

class neural_net_based_EKF:
    
    def  __init__(self):
        self.Q = np.identity(9)*0.001
        self.x_hat = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0]])
        self.P = np.identity(9)      
    
    # ################################################################
    # Predict ######################################################
    # ###############################################################

    # ----------------------------------------------------------------
    # Predict state estimate
    # ---------------------------------------------------------------
    def state_estimate(self, dt, glb):
        # print('t: ',t, 'Ts :' , Ts)
        state = glb.state
        u = state[0]
        v = state[1]
        w = state[2]
        phi = state[3]
        theta = state[4]
        psi = state[5]
        p = state[6]
        q = state[7]
        r = state[8]
        # x_hat = px_hat, py_hat, pz_hat, phi_hat, theta_hat, psi_hat
        x_hat = self.x_hat
        az = -1*state[15]/params.m
        # print(az)        
        
        DynamicsDot = np.array([
                [                                                        u],
                [                                                        v],
                [                                                        w],
                [                                   cos(phi)*sin(theta)*az],
                [                                             -sin(phi)*az],
                [                        params.g + cos(theta)*cos(psi)*az],
                [       p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta) ],
                [                                  q*cos(phi) - r*sin(phi)],
                [        (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta)]])


        x_hat = x_hat + DynamicsDot*dt
        # print('self.x_hat: ',np.shape(self.x_hat))
        # print('x_hat: ',np.shape(x_hat))
        return x_hat

    #----------------------------------------------------------------
    # Predict covariance estimate
    # ---------------------------------------------------------------
    def covariance_estimate(self, dt, glb, states):
        states = states.detach().numpy()
        state = glb.state
        u = states[3]
        v = states[4]
        w = states[5]
        phi = states[6]
        theta = states[7]
        psi = states[8]
        p = state[6]
        q = state[7]
        r = state[8]
        az = -1*state[15]/params.m
        P = self.P
        # P = np.identity(6)
        Q = self.Q
        

        F_matrix = np.array([
            [0, 0, 0, 1, 0, 0,                 	 	 0,	 	                  0,  0],
            [0, 0, 0, 0, 1, 0,                  	 0, 	                  0,  0],
            [0, 0, 0, 0, 0, 1,                	     0,     	              0,  0],
            [0, 0, 0, 0, 0, 0, 	sin(phi)*sin(theta)*az,  cos(phi)*cos(theta)*az,  0],
			[0, 0, 0, 0, 0, 0, 			  -1*cos(phi)*az,					  0,  0],
			[0, 0, 0, 0, 0, 0, -1*sin(phi)*cos(theta)*az, cos(phi)*sin(theta)*az, 0],
			[0, 0, 0, 0, 0, 0, q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta), (r*cos(phi)+q*sin(phi))/cos(theta)**2,   0],
            [0, 0, 0, 0, 0, 0,        						   -q*sin(phi),            -r*cos(phi), 			     0],
            [0, 0, 0, 0, 0, 0, (q*cos(phi) - r*sin(phi))/cos(theta), -(q*sin(phi)+r*cos(phi))*tan(theta)/cos(theta), 0]])
        
        # print(np.dot(F_matrix,P*Ts))
        P_k = P + (np.dot(F_matrix,P) + np.dot(P,F_matrix.transpose()) + Q)*dt
        return P_k

    # ###############################################################
    # Update ######################################################
    # ###############################################################

    #----------------------------------------------------------------
    # Kalman Gain
    # ---------------------------------------------------------------
    def cal_kalman_gain(self, dt, nn_model, x_new, measure, glb):
        # print('pos: ', quad.pos)
            x_hat = self.state_estimate(dt, glb)
            states = nn_model.neural_net_predict(x_new)
        # print('XXHat-shape:  ', x_hat)
            Pk = self.covariance_estimate(dt,glb, states)
        # print('Pk: ', Pk)

        # try:
            C = np.identity(9)
            R = np.identity(9)*0.01
            I = np.identity(9)
            # Y = np.array([[measure[0]], [measure[1]], [measure[2]], [measure[3]], [measure[4]], [measure[5]], [measure[6]], [measure[7]], [measure[8]]])
            Y = np.array([measure[0], measure[1], measure[2], measure[3], measure[4], measure[5], measure[6], measure[7], measure[8]])
            # print(np.dot(np.dot(C,Pk),C.transpose()))
            L = np.nan_to_num(np.dot(np.dot(Pk,C.transpose()),np.linalg.inv(R + np.dot(np.dot(C,Pk),C.transpose()))))  #kalman_gain
            # print(L)
            # logging.info(f'L:{L}')
            self.Pk = np.dot((I - np.dot(L,C)),Pk)
            # print(self.Pk)
            # print(x_hat.detach().numpy())
            # print(np.dot(L, np.transpose(Y - np.transpose(np.dot(C,x_hat)))))
            # foo = (np.transpose(x_hat.detach().numpy()) + np.dot(L,(Y - np.dot(C, x_hat.detach().numpy()))))
            foo = (x_hat + np.dot(L, np.transpose(Y - np.transpose(np.dot(C,x_hat)))))
            # .detach().numpy()
            # print(np.shape(foo))
            # print(np.shape(x_hat))
            nn_model.neural_net_update(torch.from_numpy(foo), torch.from_numpy(x_hat), dt)
            self.x_hat = foo
            glb.kalman_gain_array.append(L)
            
            return np.transpose(foo), self.Pk
    # ---------------------------------------------------------------