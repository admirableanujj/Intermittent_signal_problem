import numpy as np
import math
import params
dt= 0.1
# dt= 0.1
m = 1.08                      #% mass of the quadrotor in kg
l = 0.12                      #% length of arm of the quadrotor in m
g = 9.81                      #% gravity
i = 0
k = 1
index = 1   

Ixx = 2*m*l**2                 #% moment of inertia of x
Iyy = 2*m*l**2                 #% moment of inertia of y
Izz = 4*m*l**2                 #% moment of inertia of z


Kf = 2.2*10**-4                #% constant to relate force and angular speed
Km = 5.4*10**-6                #% constant to relate moment and angular speed
wh = math.sqrt(m*g/(4*Kf))

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
error_x = 0
error_y = 0
error_z = 0

dxm = 0.16      # arm length (m)
dym = 0.16      # arm length (m)
dzm = 0.05      # motor height (m)

