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
            self.graph_for_attitude_related_error(no_noise, glb)
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
                self.graph_for_position_error( ekf, glb)
                # self.graph_for_attitude_related_error(ekf, glb)
                self.graph_for_position_values(ekf, glb)
                # self.graph_for_angular_speed_motor(ekf, glb)
        except:
            e = sys.exc_info()
            print(f"ekf_Error:{e}")
        try:
            neural = sim_data['neural_nets']
            # self.graph_for_position_error(neural, glb)
            # self.graph_for_attitude_related_error(neural, glb)
            # self.graph_for_position_values(neural, glb)
            # self.graph_for_angular_speed_motor(neural, glb)
        except:
            e = sys.exc_info()
            print(f"Nuera_nets: {e}")
        try:
            # self.sig_acc_array(no_noise)
            # self.duo_graphs3d_pos(ekf, no_noise)
            # self.sig_graphs3d_pos(no_noise)
            self.sig_graphs3d_pos(no_noise)
            # self.plot_acc_array(no_noise, noise, ekf, neural, ekf_flag)
            # self.plot_graphs3d(no_noise, noise, ekf, ekf_flag)
            # self.plot_graphs3d_nu(no_noise, noise, ekf, neural, ekf_flag)
            # self.plot_graphs3d_pos(no_noise, noise, ekf, neural, ekf_flag)
            # self.sig_graphs3d_pos2(ekf, no_noise)
            # self.sig_graphs3d_pos2(neural, no_noise)

        except:
            e = sys.exc_info()
            # self.plot_acc_array(no_noise, noise, no_noise,ekf_flag)
            print(f'No z axis graph for you.: {e}')
        # ani =  animation.sameAxisAnimation(noise, glb)        
        plt.show()   
    

    
    def plot_graphs3d_nu(self, no_noise, noise, ekf, neural,ekf_flag):
        figr = plt.figure()
        ax = figr.add_subplot(111, projection='3d')
        line11a = ax.plot([x[0] for x in no_noise.get('acc_array')],[y[1] for y in no_noise.get('acc_array')], [z[2] for z in no_noise.get('acc_array')], 'b')
        line11b = ax.plot([x[0] for x in noise.get('acc_array')],[y[1] for y in noise.get('acc_array')], [z[2] for z in noise.get('acc_array')], 'r')
        line11c = ax.plot([x[0] for x in ekf.get('acc_array')],[y[1] for y in ekf.get('acc_array')], [z[2] for z in ekf.get('acc_array')], 'g')
        # line11d = ax.plot([x[0] for x in neural.get('acc_array')],[y[1] for y in neural.get('acc_array')], [z[2] for z in neural.get('acc_array')], 'm')
        line11a[0].set_label('xyz-true position')
        line11b[0].set_label('xyz-noise')
        line11c[0].set_label('xyz-ekf')
        # line11d[0].set_label('xyz-neural')
        ax.legend(loc = 0)
    
    def plot_graphs3d_pos(self, no_noise, noise, ekf, neural,ekf_flag):
        figr = plt.figure()
        ax = figr.add_subplot(111, projection='3d')
        # line31b=ax31.plot(range(0,len(data.get('glb.pos_array'))), [x[0] for x in data.get('glb.pos_array')], 'b')
        line11a = ax.plot([x[0] for x in no_noise.get('glb.pos_array')],[y[1] for y in no_noise.get('glb.pos_array')], [z[2] for z in no_noise.get('glb.pos_array')], 'b')
        line11b = ax.plot([x[0] for x in noise.get('imu.pos_array')],[y[1] for y in noise.get('imu.pos_array')], [z[2] for z in noise.get('imu.pos_array')], 'r')
        line11c = ax.plot([x[0] for x in ekf.get('glb.pos_array')],[y[1] for y in ekf.get('glb.pos_array')], [z[2] for z in ekf.get('glb.pos_array')], 'g')
        line11d = ax.plot([x[0] for x in neural.get('glb.pos_array')],[y[1] for y in neural.get('glb.pos_array')], [z[2] for z in neural.get('glb.pos_array')], 'm')
        line11a[0].set_label('xyz-true position')
        line11b[0].set_label('xyz-noise')
        line11c[0].set_label('xyz-ekf')
        line11d[0].set_label('xyz-neural')
        ax.set_title("position")
        ax.legend(loc = 0)


    def plot_acc_array(self, no_noise, noise, ekf, neural, ekf_flag):
        try:
            fig0a = plt.figure()
            ax = fig0a.add_subplot(111)
            v=2
            line11a = ax.plot(range(0,len([z[v] for z in no_noise.get('acc_array')])),[z[v] for z in no_noise.get('acc_array')], 'b')
            line11a[0].set_label('z-true position')
            # line11b = ax.plot(range(0,len([z[v] for z in noise.get('acc_array')])),[z[v] for z in noise.get('acc_array')], 'r')
            # line11b[0].set_label('z- noise')
            if ekf_flag:
                line11c = ax.plot(range(0,len([z[v] for z in ekf.get('acc_array')])),[z[v] for z in ekf.get('acc_array')], 'g')
                line11c[0].set_label('z-ekf')
                # if neural:
                #     line11d = ax.plot(range(0,len([z[v] for z in neural.get('acc_array')])),[z[v] for z in neural.get('acc_array')], 'm')
                #     line11d[0].set_label('z-neural')

            ax.legend(loc = 0)
            ax.set_title("z accelaration")
        except Exception as inst:
            print(inst)

          
    def graph_for_position_error(self, data, glb):
        fig1 = plt.figure()
        ax11 = fig1.add_subplot(231)
        line11a = ax11.plot(np.linspace(0,len(data.get('glb.error_xdes_array'))*params.dt, len(data.get('glb.error_xdes_array'))), data.get('glb.error_xdes_array'),'r')
        line11b = ax11.plot(range(0,len(data.get('glb.error_x_array'))), data.get('glb.error_x_array'),'b')
        line11a[0].set_label('error_des')
        line11b[0].set_label('error')
        ax11.legend(loc = 0)
        ax11.set_title("error_x")
        #-------------------------
        ax12 = fig1.add_subplot(232)
        ax12.plot(range(0,len(data.get('glb.error_ydes_array'))), data.get('glb.error_ydes_array'),'r')
        ax12.plot(range(0,len(data.get('glb.error_y_array'))), data.get('glb.error_y_array'),'b')
        ax12.set_title("error_y")
        #-------------------------
        ax13 = fig1.add_subplot(233)
        ax13.plot(range(0,len(data.get('glb.error_zdes_array'))), data.get('glb.error_zdes_array'),'r')
        ax13.plot(range(0,len(data.get('glb.error_z_array'))), data.get('glb.error_z_array'),'b')
        ax13.set_title("error_z")
        #-------------------------
        ax14 = fig1.add_subplot(234)
        ax14.plot(range(0,len(data.get('glb.error_x_virtual_array'))), data.get('glb.error_x_virtual_array'),'r')
        ax14.set_title("error_x_virtual")
        #-------------------------
        ax15 = fig1.add_subplot(235)
        ax15.plot(range(0,len(data.get('glb.error_y_virtual_array'))), data.get('glb.error_y_virtual_array'),'r')
        ax15.set_title("error_y_virtual")
        #-------------------------
        ax16 = fig1.add_subplot(236)
        ax16.plot(range(0,len(data.get('glb.error_z_virtual_array'))), data.get('glb.error_z_virtual_array'),'r')
        ax16.set_title("error_z_virtual")
        # ax16.set_title(f"{data}")

    def graph_for_attitude_related_error(self, data, glb):
        # print(data.get('glb.angles_desired_array'))
        fig2 = plt.figure()
        ax21 = fig2.add_subplot(231)
        line21a = ax21.plot(range(0,len([x[0] for x in data.get('glb.angles_desired_array')])), [x[0] for x in data.get('glb.angles_desired_array')],'r')
        line21b = ax21.plot(range(0,len([x[0] for x in data.get('glb.phi_theta_psi_array')])), [x[0] for x in data.get('glb.phi_theta_psi_array')],'b')
        line21c = ax21.plot(range(0,len([x[0] for x in data.get('imu.phi_theta_psi_array')])), [x[0] for x in data.get('imu.phi_theta_psi_array')],'g')
        # line21d = ax21.plot(range(0,len([x[0] for x in data.get('glb.state_ekf')])), [x[0] for x in data.get('glb.state_ekf')],'g')
        line21a[0].set_label('Angle Desired')
        line21b[0].set_label('Angle')
        line21c[0].set_label('Angle IMU')
        ax21.legend(loc = 0)
        ax21.set_title('phi_des vs phi')
        #-------------------------
        ax22 = fig2.add_subplot(232)
        line22a = ax22.plot(range(0,len([y[1] for y in data.get('glb.angles_desired_array')])), [y[1] for y in data.get('glb.angles_desired_array')],'r')
        line22b = ax22.plot(range(0,len([y[1] for y in data.get('glb.phi_theta_psi_array')])), [y[1] for y in data.get('glb.phi_theta_psi_array')],'b')
        line22c = ax22.plot(range(0,len([y[1] for y in data.get('imu.phi_theta_psi_array')])), [y[1] for y in data.get('imu.phi_theta_psi_array')],'g')
        # ax22.plot(range(0,len(glb.phi_theta_psi_array_ekf)), [y[1] for y in glb.phi_theta_psi_array_ekf],'g')
        ax22.set_title('theta_des vs theta')
        #-------------------------
        ax23 = fig2.add_subplot(233)
        line23a = ax23.plot(range(0,len([z[2] for z in data.get('glb.angles_desired_array')])), [z[2] for z in data.get('glb.angles_desired_array')],'r')
        line23b = ax23.plot(range(0,len([z[2] for z in data.get('glb.phi_theta_psi_array')])), [z[2] for z in data.get('glb.phi_theta_psi_array')],'b')
        line23c = ax23.plot(range(0,len([z[2] for z in data.get('imu.phi_theta_psi_array')])), [z[2] for z in data.get('imu.phi_theta_psi_array')],'g')
        # ax22.plot(range(0,len(glb.phi_theta_psi_array_ekf)), [y[1] for y in glb.phi_theta_psi_array_ekf],'g')
        ax23.set_title('psi_des vs psi')
        #-------------------------
        ax24 = fig2.add_subplot(234)
        ax24.plot(range(0,len([x[0] for x in data.get('glb.angular_vel_required_array')])), [ x[0] for x in data.get('glb.angular_vel_required_array')],'r')
        ax24.plot(range(0,len([x[0] for x in data.get('glb.p_q_r_array')])), [ x[0] for x in data.get('glb.p_q_r_array')],'b')
        ax24.plot(range(0,len([x[0] for x in data.get('imu.pqr_array')])), [x[0] for x in data.get('imu.pqr_array')],'g')
        ax24.set_title('p_des vs p')
#       #-------------------------
        ax25 = fig2.add_subplot(235)
        ax25.plot(range(0,len([ y[1] for y in data.get('glb.angular_vel_required_array')])), [ y[1] for y in data.get('glb.angular_vel_required_array')],'r')
        ax25.plot(range(0,len([ y[1] for y in data.get('glb.p_q_r_array')])), [ y[1] for y in data.get('glb.p_q_r_array')],'b')
        ax25.plot(range(0,len([y[1] for y in data.get('imu.pqr_array')])), [y[1] for y in data.get('imu.pqr_array')],'g')
        ax25.set_title('q_des vs q')
#       #-------------------------
        ax26 = fig2.add_subplot(236)
        ax26.plot(range(0,len([z[2] for z in data.get('glb.angular_vel_required_array')])), [ z[2] for z in data.get('glb.angular_vel_required_array')],'r')
        ax26.plot(range(0,len([z[2] for z in data.get('glb.p_q_r_array')])), [ z[2] for z in data.get('glb.p_q_r_array')],'b')
        ax26.plot(range(0,len([z[2] for z in data.get('imu.pqr_array')])), [z[2] for z in data.get('imu.pqr_array')],'g')
        ax26.set_title('r_des vs r')
        # ax26.set_title(f"{data}")
#       #-------------------------        
        
    def graph_for_position_values(self, data, glb):
        fig3 = plt.figure()
        ax31 = fig3.add_subplot(234)
        X_scale = np.linspace(0,(len(data.get('glb.position_final_required_array'))+1)*params.dt,len(data.get('glb.position_final_required_array'))+1)
        X_scale = X_scale[0:-1]
        line31a=ax31.plot(X_scale, [x[0] for x in data.get('glb.position_final_required_array')],'r')
        line31b=ax31.plot(X_scale, [x[0] for x in data.get('glb.pos_array')], 'b')
        line31c=ax31.plot(X_scale, [x[0] for x in data.get('imu.pos_array')], 'g')
        # line31d=ax31.plot(range(0,len(data.get('glb.state_ekf'))), [x[0] for x in data.get('glb.state_ekf')], 'k')
        line31a[0].set_label('pos x Desired')
        line31b[0].set_label('pos x')
        line31c[0].set_label('pos x imu')
        # line31d[0].set_label('pos x ekf')
        ax31.legend(loc = 0)
        ax31.set_title('x_des vs x')
        #-------------------------  
        ax32 = fig3.add_subplot(235)
        ax32.plot(X_scale, [y[1] for y in data.get('glb.position_final_required_array')],'r')
        ax32.plot(X_scale, [y[1] for y in data.get('glb.pos_array')], 'b')
        ax32.plot(X_scale, [y[1] for y in data.get('imu.pos_array')], 'g')
        # ax32.plot(range(0,len(data.get('glb.state_ekf'))), [y[1] for y in data.get('glb.state_ekf')], 'k')
#       # ax32.plot(range(0,len(glb.pos_array_ekf)), [y[1] for y in glb.pos_array_ekf],'g')
#       # ax32.plot(range(0,len(glb.no_noise_pose)), [y[1] for y in glb.no_noise_pose],'k')
        ax32.set_title('y_des vs y')                      
        #-------------------------  
        ax33 = fig3.add_subplot(236)
        ax33.plot(X_scale, [z[2] for z in data.get('glb.position_final_required_array')],'r')
        ax33.plot(X_scale, [z[2] for z in data.get('glb.pos_array')], 'b')
        ax33.plot(X_scale, [z[2] for z in data.get('imu.pos_array')], 'g')
        # ax33.plot(range(0,len(data.get('glb.state_ekf'))), [z[2] for z in data.get('glb.state_ekf')], 'k')
        # ax33.plot(range(0,len(glb.pos_array_ekf)), [z[2] for z in glb.pos_array_ekf],'g')
        ax33.set_title('z_des vs z')
        #-------------------------
        ax34 = fig3.add_subplot(231)
        ax34.plot(X_scale, [x[0] for x in data.get('glb.vel_final_required_array')],'r')
        ax34.plot(X_scale, [x[0] for x in data.get('glb.vel_array')], 'b')
        ax34.plot(X_scale, [x[0] for x in data.get('imu.uvw_array')], 'g')
        # ax34.plot(range(0,len(glb.vel_array)), [x[0] for x in glb.vel_array_ekf], 'g')
        ax34.set_title('velx_des vs velx')
        #-------------------------
        ax35 = fig3.add_subplot(232)
        ax35.plot(X_scale, [y[1] for y in data.get('glb.vel_final_required_array')],'r')
        ax35.plot(X_scale, [y[1] for y in data.get('glb.vel_array')], 'b')
        ax35.plot(X_scale, [y[1] for y in data.get('imu.uvw_array')], 'g')
        # ax35.plot(range(0,len(glb.vel_array)), [y[1] for y in glb.vel_array_ekf], 'g')
        ax35.set_title('vely_des vs vely')
        #-------------------------
        ax36 = fig3.add_subplot(233)
        ax36.plot(X_scale, [z[2] for z in data.get('glb.vel_final_required_array')],'r' )
        ax36.plot(X_scale, [z[2] for z in data.get('glb.vel_array')], 'b')
        ax36.plot(X_scale, [z[2] for z in data.get('imu.uvw_array')], 'g')
        # ax36.plot(range(0,len(glb.vel_array)), [z[2] for z in glb.vel_array_ekf], 'g')
        ax36.set_title('velz_des vs velz')
        # ax36.set_title(f"{data}")
        #-------------------------
            
    def graph_for_angular_speed_motor(self, data, glb):
        fig4 = plt.figure()
        # print([x[0] for x in data.get('glb.w_des_array')])
        ax41 = fig4.add_subplot(231)
        line41a=ax41.plot(range(0,len(data.get('glb.w_des_array'))), [x[0] for x in data.get('glb.w_des_array')],'r')
        ax42 = fig4.add_subplot(232)
        line42a=ax42.plot(range(0,len(data.get('glb.w_des_array'))), [y[1] for y in data.get('glb.w_des_array')],'r')
        ax43 = fig4.add_subplot(233)
        line43a=ax43.plot(range(0,len(data.get('glb.w_des_array'))), [z[2] for z in data.get('glb.w_des_array')],'r')
        ax44 = fig4.add_subplot(234)
        line44a=ax44.plot(range(0,len(data.get('glb.w_des_array'))), [w[3] for w in data.get('glb.w_des_array')],'r')
        # line41b=ax41.plot(range(0,len(data.get('glb.pos_array'))), [x[0] for x in data.get('glb.pos_array')], 'b')
        # line41a[0].set_label('pos x Desired')
        # line41b[0].set_label('pos x')
        # ax41.legend(loc = 1)
        ax41.set_title('w_des vs w')
        #-------------------------  
  
    # def plot_graphs3d(self, no_noise, noise, ekf, ekf_flag):
    #     fig = plt.figure()
    #     ax = fig.add_subplot(111, projection='3d')
    #     line11a = ax.plot([x[0] for x in no_noise.get('acc_array')],[y[1] for y in no_noise.get('acc_array')], [z[2] for z in no_noise.get('acc_array')], 'b')
    #     line11b = ax.plot([x[0] for x in noise.get('acc_array')],[y[1] for y in noise.get('acc_array')], [z[2] for z in noise.get('acc_array')], 'r')
    #     line11c = ax.plot([x[0] for x in ekf.get('acc_array')],[y[1] for y in ekf.get('acc_array')], [z[2] for z in ekf.get('acc_array')], 'g')
    #     line11a[0].set_label('z-no noise')


    def plot_loss_train(self,sim_type):
        temp = self.file_read('loss_array_train')
        fig_loss = plt.figure()
        ax_loss = fig_loss.add_subplot(111)
        line_loss = ax_loss.plot(range(0,len(temp)),temp)
        ax_loss.set_title('loss_array_train function.')
    
    def plot_loss_test(self,sim_type):
        temp = self.file_read('test_loss_array')
        fig_loss = plt.figure()
        ax_loss = fig_loss.add_subplot(111)
        line_loss = ax_loss.plot(range(0,len(temp)),temp)
        ax_loss.set_title('test_loss_array function.')

    @staticmethod
    def file_read(file_name):
        f = open(f'.\\temp\\glb_temp_{file_name}', 'r')
        lines = f.readlines()
        f.close()
        temp = []
        for line in lines:
            # print(line)
            foo = line.replace('[','').replace(']','').replace('\n','').replace('\'','')
            # print(foo)
            # # foo = foo.split(',')
            temp.append([float(foo)])
            # # print(float(foo[0]))    
            # # print(temp)
        return temp


        #####################Rough work#######################################
        # print(list(no_noise.keys()))
        # print(noise.keys())
        # print(ekf.get('glb.angles_desired_array'))
    def sig_acc_array(self, no_noise):
        try:
            fig0a = plt.figure()
            ax = fig0a.add_subplot(111)
            v=2
            line11a = ax.plot(range(0,len([z[v] for z in no_noise.get('acc_array')])),[z[v] for z in no_noise.get('acc_array')], 'b')
            line11a[0].set_label('z-true position')
            ax.legend(loc = 0)
            ax.set_title("z accelaration")
        except Exception as inst:
            print(inst)
    
    def duo_graphs3d_pos(self, no_noise, ekf):
        figr = plt.figure()
        ax = figr.add_subplot(111, projection='3d')
        line11c = ax.plot([x[0] for x in ekf.get('glb.pos_array')],[y[1] for y in ekf.get('glb.pos_array')], [z[2] for z in ekf.get('glb.pos_array')], 'g')
        line11c[0].set_label('xyz-ekf')
        line11b = ax.plot([x[0] for x in no_noise.get('glb.pos_array')],[y[1] for y in no_noise.get('glb.pos_array')], [z[2] for z in no_noise.get('glb.pos_array')], 'b')
        line11b[0].set_label('xyz-true')
        ax.set_title("position")
        ax.legend(loc = 0)
    
    def sig_graphs3d_pos(self, ekf):
        figr = plt.figure()
        ax = figr.add_subplot(111, projection='3d')
        line11c = ax.plot([x[0] for x in ekf.get('glb.pos_array')],[y[1] for y in ekf.get('glb.pos_array')], [z[2] for z in ekf.get('glb.pos_array')], 'g')
        line11c[0].set_label('xyz-true')
        ax.set_title("position")
        ax.legend(loc = 0)

    def sig_graphs3d_pos2(self, ekf, no_noise):
        figr = plt.figure()
        ax = figr.add_subplot(111, projection='3d')
        line11c = ax.plot([x[0] for x in ekf.get('glb.pos_array')],[y[1] for y in ekf.get('glb.pos_array')], [z[2] for z in ekf.get('glb.pos_array')], 'g')
        line11d = ax.plot([x[0] for x in no_noise.get('glb.pos_array')],[y[1] for y in no_noise.get('glb.pos_array')], [z[2] for z in no_noise.get('glb.pos_array')], 'b')
        line11d[0].set_label('xyz-true')
        line11c[0].set_label('xyz-ekf')
        ax.set_title("position")
        ax.legend(loc = 0)