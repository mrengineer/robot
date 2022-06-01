import numpy as np
import math

#DRAFT TESTS TO IMPLEMENT IDEAS
# READ https://studref.com/472293/tehnika/matrichnye_metody_preobrazovaniya_koordinat_robototehnike?

pnt = np.array([[0], [0], [0], [1]])
alpha = math.radians(0)
print(alpha)


'''
 X
 ^
 |
 /-----> Y
/
Z
'''

#Tooltip shift matrix
motor1_x_shift = 53.0 + 64.5
motor1_y_shift = -39.5
motor1_z_shift = 0

#Mechanical shift because of design
Tm1s = np.array([   [1, 0,  0,  motor1_x_shift],
                    [0, 1,  0,  motor1_y_shift],
                    [0, 0,  1,  motor1_z_shift],
                    [0, 0,  0,  1]  ])


#Linear move mounted on rotary AXIS
Tm1l = np.array([   [1, 0,  0,  0],
                    [0, 1,  0,  0],
                    [0, 0,  1,  0],
                    [0, 0,  0,  1]  ])

#Angular rotation 
Tm1a = np.array([   [1, 0               , 0                 , 0],
                    [0, math.cos(alpha) , -math.sin(alpha)  , 0],
                    [0, math.sin(alpha) ,  math.cos(alpha)  , 0],
                    [0, 0               , 0                 , 1]    ])


T = np.matmul(Tm1a, Tm1l)   #Linear shift mounted on motor
T = np.matmul(T, Tm1s)      #Linear shifts because of design

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
fi = math.radians(0)
print(fi)

motor2_x_shift = 0
motor2_y_shift = 53.0
motor2_z_shift = 155.2

#Mechanical shift because of design
Tm2s = np.array([   [1, 0,  0,  motor2_x_shift],
                    [0, 1,  0,  motor2_y_shift],
                    [0, 0,  1,  motor2_z_shift],
                    [0, 0,  0,  1]  ])


#Linear move mounted on rotary AXIS
Tm2l = np.array([   [1, 0,  0,  0],
                    [0, 1,  0,  0],
                    [0, 0,  1,  0],
                    [0, 0,  0,  1]  ])

#Angular rotation 
Tm2a = np.array([   [math.cos(fi) , 0 , math.sin(fi) , 0],
                    [0            , 1 , 0            , 0],
                    [-math.sin(fi), 0 , math.cos(fi) , 0],
                    [0            , 0 , 0            , 1]    ])

T = np.matmul(T, Tm2a)   #Linear shift mounted on motor
T = np.matmul(T, Tm2l)   #Linear shift mounted on motor
T = np.matmul(T, Tm2s)   #Linear shifts because of design

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
theta = math.radians(-0)
print(theta)

motor3_x_shift =  0
motor3_y_shift = -35
motor3_z_shift =  98

#Mechanical shift because of design
Tm3s = np.array([   [1, 0,  0,  motor3_x_shift],
                    [0, 1,  0,  motor3_y_shift],
                    [0, 0,  1,  motor3_z_shift],
                    [0, 0,  0,  1]  ])


#Linear move mounted on rotary AXIS
Tm3l = np.array([   [1, 0,  0,  0],
                    [0, 1,  0,  0],
                    [0, 0,  1,  0],
                    [0, 0,  0,  1]  ])

#Angular rotation 
Tm3a = np.array([   [math.cos(theta) , -math.sin(theta) , 0, 0],                    
                    [math.sin(theta) , math.cos(theta)  , 0, 0],
                    [0               , 0                , 1, 0],                    
                    [0               , 0                , 0, 1]    ])


#T = np.matmul(T, Tm3a)   #Linear shift mounted on motor
T = np.matmul(Tm3a, Tm3l)   #Linear shift mounted on motor
T = np.matmul(T, Tm3s)   #Linear shifts because of design


R = np.dot(T, pnt)
print(T)
print("--------------------------------")
print(R)