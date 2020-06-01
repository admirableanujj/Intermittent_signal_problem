import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
from matplotlib import animation
# from plot_animation import Animation as ani
import animation
import params
class Plot_graph:

    def __init__(self):
        pass
    
    def plot_graphs(self, acc_array, sim_data, ekf_flag, global_vars):
        glb = global_vars
        try:
            no_noise = sim_data['no_noise']
            # self.graph_for_position_error( no_noise, glb)        
            # self.graph_for_attitude_related_error(no_noise, glb)
            # self.graph_for_position_values(no_noise, glb)
            # self.graph_for_angular_speed_motor(no_noise, glb)
            # self.plot_loss_train('no_noise')
            # self.plot_loss_test('no_noise')
        except:
            e = sys.exc_info()
            print(f'no_noise_Error: {e}')
        
        try:
            noise = sim_data['noise']
            # self.graph_for_position_error( noise, glb)
            # self.graph_for_attitude_related_error(noise, glb)
            # self.graph_for_position_values(noise, glb)
            # self.graph_for_angular_speed_motor(noise, glb)
        except:
            e = sys.exc_info()
            print(f'noise_Error: {e}')
        try:
            if ekf_flag:
                ekf = sim_data['ekf']
        except:
            e = sys.exc_info()
            print(f"ekf_Error:{e}")
        try:
            neural = sim_data['neural']
        except:
            e = sys.exc_info()
            print(f"Nueral_nets: {e}")
        try:
            pass


        except:
            e = sys.exc_info()
            # self.plot_acc_array(no_noise, noise, no_noise,ekf_flag)

            print(f'No z axis graph for you.: {e}')
        self.graph_for_position_values2(no_noise, {'ekf':ekf})
        self.graph_for_position_values2(no_noise, {'neural':neural})
        self.graph_for_position_values({'ekf':ekf})
        # self.graph_for_pos_difference({'ekf':ekf})
        self.graph_for_position_values({'neural':neural})
        # ani =  animation.sameAxisAnimation(noise, glb)        
        # plt.show()   
    
    def graph_for_position_values2(self, true, data_in):
        for key, value in data_in.items():
            data_name = key
            data = value
            # print(f'data_name:{data_name}, data: {np.shape(value)}')
        fig3 = plt.figure()
        n = ['x','y','z']
        for i in range(0,3):
            ax = fig3.add_subplot(131+i)
            X_scale = np.linspace(0,(len(data.get('glb.position_final_required_array'))+1)*params.dt,len(data.get('glb.position_final_required_array'))+1)
            X_scale = X_scale[0:-1]
            line31a=ax.plot(X_scale, [x[i] for x in data.get('glb.position_final_required_array')],'r')
            line31b=ax.plot(X_scale, [x[i] for x in data.get('glb.qpos_array')], color='C1', linestyle='-.')
            line31c=ax.plot(X_scale, [x[i] for x in data.get('imu.pos_array')], 'b')
            line31a[0].set_label(f'pos {n[i]} Desired')
            line31b[0].set_label(f'pos {n[i]} true')
            line31c[0].set_label(f'pos {n[i]} {data_name}')
        # line31d[0].set_label('pos x ekf')
            ax.set_xlabel('Time (in sec)')
            ax.set_ylabel('Distance')
            ax.legend(loc = 0)
            ax.set_title(f'{n[i]}_des vs {n[i]}')
            ax.grid(color='k', linestyle='-', linewidth=0.5)

    def graph_for_position_values(self, data_in):
        for key, value in data_in.items():
            data_name = key
            data = value
        fig3 = plt.figure()
        # print(data.get('glb.qpos_array'))
        n = ['x','y','z']
        for i in range(0,1):
            # ax = fig3.add_subplot(131+i)
            ax = fig3.add_subplot(111)
            X_scale = np.linspace(0,(len(data.get('glb.position_final_required_array'))+1)*params.dt,len(data.get('glb.position_final_required_array'))+1)
            X_scale = X_scale[0:-1]
            line31a=ax.plot(X_scale, [x[i] for x in data.get('glb.position_final_required_array')],'r')
            line31b=ax.plot(X_scale, [x[i] for x in data.get('glb.qpos_array')], color='C1', linestyle='-.')
            line31c=ax.plot(X_scale, [x[i] for x in data.get('imu.pos_array')], 'b')
            line31a[0].set_label(f'pos {n[i]} Desired')
            line31b[0].set_label(f'pos {n[i]} true')
            line31c[0].set_label(f'pos {n[i]} {data_name}')
        # line31d[0].set_label('pos x ekf')
            ax.set_xlabel('Time (in sec)')
            ax.set_ylabel('Distance')
            ax.legend(loc = 0)
            ax.set_title(f'{n[i]}_des vs {n[i]}')
            ax.grid(color='k', linestyle='-', linewidth=2)

    
    def graph_for_pos_difference(self, data_in):
        for key, value in data_in.items():
            data_name = key
            data = value
        fig3 = plt.figure()
        # print(data.get('glb.qpos_array'))
        n = ['x','y','z']
        for i in range(0,1):
            # ax = fig3.add_subplot(131+i)
            ax = fig3.add_subplot(111)
            X_scale = np.linspace(0,(len(data.get('glb.position_final_required_array'))+1)*params.dt,len(data.get('glb.position_final_required_array'))+1)
            X_scale = X_scale[0:-1]
            line31d = ax.plot( X_scale, [i-j for i,j in zip(data.get('glb.qpos_array'), data.get('imu.pos_array'))])
            # line31a=ax.plot(X_scale, [x[i] for x in data.get('glb.position_final_required_array')],'r')
            # line31b=ax.plot(X_scale, [x[i] for x in data.get('glb.qpos_array')], 'b')
            # line31c=ax.plot(X_scale, [x[i] for x in data.get('imu.pos_array')], 'g')
            # line31a[0].set_label(f'pos {n[i]} Desired')
            # line31b[0].set_label(f'pos {n[i]} true')
            # line31c[0].set_label(f'pos {n[i]} {data_name}')
            line31d[0].set_label('Differece in x')
            line31d[1].set_label('Differece in y')
            line31d[2].set_label('Differece in z')
            ax.set_xlabel('Time (in sec)')
            ax.set_ylabel('Distance')
            ax.legend(loc = 0)
            ax.set_title(f'{n[i]}_des vs {n[i]}')
    
    ##########################################################################################################
        ############################## New code same as in Plot_graph ####################################
    ##########################################################################################################


  

        #####################Rough work#######################################
        # print(list(no_noise.keys()))
        # print(noise.keys())
        # print(ekf.get('glb.angles_desired_array'))

