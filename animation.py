# import params
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import sys
from matplotlib import animation
import params
import numpy as np
from scipy.spatial.transform import Rotation as Ro
from numpy import cos, sin, tan, pi
# import utils


numFrames = 8
        # return self.line1, self.line2, self.line3

def sameAxisAnimation(data, glb):
        
    Ts = params.dt
    # numFrames = self.numFrames
    t_all = list(range(0,len(data.get('glb.pos_array'))))
    x = [x[0] for x in data.get('glb.pos_array')]
    y = [y[1] for y in data.get('glb.pos_array')]
    z = [z[2] for z in data.get('glb.pos_array')]
    xDes = [x[0] for x in data.get('glb.position_final_required_array')]
    yDes = [y[1] for y in data.get('glb.position_final_required_array')]
    zDes = [z[2] for z in data.get('glb.position_final_required_array')]

    waypoints = glb.waypoints         
    x_wp = waypoints[:,0]
    y_wp = waypoints[:,1]
    z_wp = waypoints[:,2]
        
        
    fig = plt.figure()
    ax = Axes3D(fig)
    line1, = ax.plot([], [], [], lw=2, color='red')
    line2, = ax.plot([], [], [], lw=2, color='blue')
    line3, = ax.plot([], [], [], '--', lw=1, color='blue')
    # Setting the axes properties
    extraEachSide = 0.5
    maxRange = 0.5*max(np.array([max(x)-min(x), max(y)-min(y), max(z)-min(z)])) + extraEachSide
    mid_x = 0.5*(max(x)+min(x))
    mid_y = 0.5*(max(y)+min(y))
    mid_z = 0.5*(max(z)+min(z))
        
    ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
    ax.set_xlabel('X')
    ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
    ax.set_ylabel('Y')
    ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
    ax.set_zlabel('Altitude')

    titleTime = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)
    trajType = ''
    yawTrajType = ''
    ax.scatter(x_wp, y_wp, z_wp, color='green', alpha=1, marker = 'o', s = 25)
    ax.plot(xDes, yDes, zDes, ':', lw=1.3, color='green')
    titleType1 = ax.text2D(0.95, 0.95, trajType, transform=ax.transAxes, horizontalalignment='right')
    titleType2 = ax.text2D(0.95, 0.91, 'Yaw: '+ yawTrajType, transform=ax.transAxes, horizontalalignment='right')
    
    def updateLines(i):
        # numFrames = self.numFrames
        t_all = list(range(0,len(data.get('glb.pos_array'))))
        # pos_all = [x.tolist() for x in data.get('glb.pos_array')]
        pos_all = data.get('glb.pos_array')
        time = t_all[i*numFrames]
        pos = pos_all[i*numFrames]
        x = pos[0]
        y = pos[1]
        z = pos[2]
        # print(x)
        if i <1:
            i=1
        # print(pos_all[0:1][0])
        x_from0 = [x[0] for x in pos_all[0:i*numFrames]]
        y_from0 = [y[1] for y in pos_all[0:i*numFrames]]
        z_from0 = [z[2] for z in pos_all[0:i*numFrames]]
        # l = [x[0] for x in pos_all[0:i*numFrames]]
        # print(l)
        
        dxm = params.dxm
        dym = params.dym
        dzm = params.dzm
            
        phi_theta_psi = data.get('glb.phi_theta_psi_array')[i*numFrames]
        
        # if (config.orient == "NED"):
        #     z = -z
        #     z_from0 = -z_from0
        #     quat = np.array([quat[0], -quat[1], -quat[2], quat[3]])
        phi_theta_psi = [x.tolist() for x in phi_theta_psi]
        # print(phi_theta_psi)
        # R = Ro.from_euler('zyx', phi_theta_psi, degrees=True)
        # R =  phi_theta_psi 
        phi = phi_theta_psi[0:i*numFrames][0]
        theta = phi_theta_psi[0:i*numFrames][1]
        psi  = phi_theta_psi[0:i*numFrames][2]
        R = np.array([
                [1,     sin(phi)*tan(theta),                   cos(phi)*tan(theta)],
                [0,                cos(phi),                             -sin(phi)],
                [0,     sin(phi)*(cos(theta)**-1),       cos(phi)*(cos(theta)**-1)]])
        # print(np.shape(R))
        motorPoints = np.array([[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
        # print(f'np.shape(R):{np.shape(R)},np.shape(motorPoints):{np.shape(motorPoints)}')
        motorPoints = np.dot(R, np.transpose(motorPoints))
        # print(motorPoints)
        motorPoints[0][:] += x 
        motorPoints[1][:] += y 
        motorPoints[2][:] += z 
            
        line1.set_data(motorPoints[0,0:3], motorPoints[1,0:3])
        line1.set_3d_properties(motorPoints[2,0:3])
        line2.set_data(motorPoints[0,3:6], motorPoints[1,3:6])
        line2.set_3d_properties(motorPoints[2,3:6])
        line3.set_data(x_from0, y_from0)
        line3.set_3d_properties(z_from0)
        titleTime.set_text(u"Time = {:.2f} s".format(time))
            
        return line1, line2

    def ini_plot():
        line1.set_data([], [])
        line1.set_3d_properties([])
        line2.set_data([], [])
        line2.set_3d_properties([])
        line3.set_data([], [])
        line3.set_3d_properties([])

        return line1, line2, line3

    line_ani = animation.FuncAnimation(fig, updateLines, init_func=ini_plot, frames=len(t_all[0:-2:numFrames]), interval=(Ts*100*numFrames), blit=False)        

    if True:
        line_ani.save('C:\\Users\\anujj\\OneDrive - University of Cincinnati\\Thesis\\Figure\\animation2\\animation_noise2.gif', writer='imagemagick', fps=60)
    return line_ani
