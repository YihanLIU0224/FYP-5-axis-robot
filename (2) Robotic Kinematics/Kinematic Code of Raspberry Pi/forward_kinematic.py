'''
Code for forward kinematics of designed robot
Copyright Yihan LIU
Date 2023.05

Description: This is the code to transform rotational angles of joints 
             to the coordinates of end effector 

Input: Rotational angles of joints
Output: 3D coordinates of end point
'''
import math as m

# Define configuration parameters
L1 = 261
L2 = 220
L3 = 223
L4 = 158

# Define the math functions
cos = m.cos
sin = m.sin

# Function for forward kinematics
def forward(rotation):
    # Input rotational angles of joints
    st1 = m.radians(rotation[0])
    st2 = m.radians(rotation[1])
    st3 = m.radians(rotation[2])
    st4 = m.radians(rotation[3])
    st5 = m.radians(rotation[4])

    # Calculate the coordinates of end point
    x_end = L2*cos(st1)*sin(st2) \
        - L4*sin(st5)*(sin(st1)*sin(st4)+cos(st4) * (cos(st1)*sin(st2+st3))) \
        + (L4*cos(st5)+L3)*(cos(st1)*cos(st2+st3))
    
    y_end = L2*sin(st1)*sin(st2) \
        - L4*sin(st5)*(cos(st1)*sin(st4)+cos(st4) * (sin(st1)*sin(st2+st3))) \
        + (L4*cos(st5)+L3)*(sin(st1)*cos(st2+st3))

    z_end = L2*cos(st2) - L3*sin(st2+st3) - L4*cos(st5)*sin(st2+st3) \
        - L4*cos(st4)*sin(st5)*cos(st2+st3) + L1 
    
    location = [x_end, y_end, z_end]
    
    return location
