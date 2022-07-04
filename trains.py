from ctypes import sizeof
from tokenize import single_quoted
from turtle import color
import numpy as np
from scipy import optimize
import math, os, time


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# DRAFT TESTS TO IMPLEMENT IDEAS
# READ https://studref.com/472293/tehnika/matrichnye_metody_preobrazovaniya_koordinat_robototehnike?


def time_of_function(function):
    def wrapped(*args):
        start_time = time.perf_counter_ns()
        res = function(*args)
        print((time.perf_counter_ns() - start_time) / 10**6, "millisec\n")
        return res
    return wrapped


'''
      X
      ^
      |
      /-----> Y
     /
    Z 
'''

# Motor settings are directly in code according prototype
# It will be fixed while implementing Robot class
def calc(alpha: float, fi: float, theta: float):
    pnt = np.array([[0], [0], [0], [1]])    

    '''
    print(f"""INPUT:
        alpha   CW  X:             {alpha} deg
        fi      CW  Y:             {fi} deg
        theta   CCW Y after shift: {theta} deg
{'-'*45}""")
    '''

    motor1_CW = True

    if motor1_CW:   k = 1
    else:           k = -1

    alpha = math.radians(alpha)

    #Tooltip shift matrix
    motor1_x_shift = 112.2 #53.0 + 64.5  
    motor1_y_shift = -45
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

    cosine  = (math.cos(k*alpha))
    sine    = (math.sin(k*alpha))

    #print(f"cosine {cosine} sine {sine} alpha={alpha}")

    Tm1a = np.array([   [1, 0, 0, 0],       #X axis (no changes because of angle)
                        [0, cosine, -sine, 0],       #Rotation in YZ plane
                        [0, sine, cosine, 0],
                        [0, 0, 0, 1]    ])


    T = np.matmul(Tm1a, Tm1l)   #Linear shift mounted on motor
    T = np.matmul(T, Tm1s)      #Linear shifts because of design
    
        
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    motor2_CW = True

    if motor2_CW:   k = 1
    else:           k = -1

    fi = math.radians(fi)

    motor2_x_shift = 126
    motor2_y_shift = 53
    motor2_z_shift = 0

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

    cosine  = math.cos(k*fi)
    sine    = math.sin(k*fi)
    
    #Angular rotation
    Tm2a = np.array([   [cosine, 0 , -sine  , 0],          #Rotation affects X axis
                        [0     , 1 , 0      , 0],
                        [-sine , 0 , cosine , 0],          #Rotation affects Z axis (ie XZ plane)
                        [0     , 0 , 0      , 1]    ])

    T = np.matmul(T, Tm2a)   #Linear shift mounted on motor
    T = np.matmul(T, Tm2l)   #Linear shift mounted on motor
    T = np.matmul(T, Tm2s)   #Linear shifts because of design

    

    
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    motor3_CW = False

    if motor3_CW:   k = 1
    else:           k = -1

    theta = math.radians(theta)

    motor3_x_shift =  105.7
    motor3_y_shift = -34
    motor3_z_shift =  0

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

    cosine  = math.cos(k*theta)
    sine    = math.sin(k*theta)

    #Angular rotation
    Tm3a = np.array([   [cosine , 0 , -sine  , 0],            #rotation affects X axis
                        [0      , 1 , 0      , 0],
                        [sine   , 0 , cosine , 0],            #affects Z axis
                        [0      , 0 , 0      , 1]    ])

    T = np.matmul(T, Tm3a)   #Linear shift mounted on motor
    T = np.matmul(T, Tm3l)   #Linear shift mounted on motor
    T = np.matmul(T, Tm3s)   #Linear shifts because of design
    
    

    '''
    print("- " * 20)
    s = ""
    for i in T:
        s = ""
        for j in i:
            s = s + str(j) + "      "
        print (s)


    print("- " * 20)
    '''

    R = np.dot(T, pnt)
    #print(R)
    rez = [round(R[0][0], 9), round(R[1][0], 9), round(R[2][0], 9)]
    return rez
    #return (round(R[0][0], 3), round(R[1][0], 3), round(R[2][0], 3))


if __name__ == '__main__':
    os.system('clear')

    t_x = 335.604
    t_y = -10.014
    t_z = -92.959

    print (calc(10, 10, 30)) #-> XYZ (335.604, -10.014, -92.959)
    
    def jacobian(f, x):
        h   = 0.01
        n   = len(x)
        Jac = np.zeros([n,n])
        f0  = f(x)
        

        for i in range(0, n, 1):
            tt      = x[i]
            x[i]    = tt + h

            f1      = f(x)
            x[i]    = tt
            Jac [:,i] = (f1 - f0)/h
        return Jac, f0

    @time_of_function
    def newton(f, x, tol=1.0e-3):
            iterMax = 500
            for i in range(iterMax):
                    
                    Jac, fO = jacobian(f, x)

                    err = math.sqrt(np.dot(fO, fO) / len(x))

                    if err < tol:   return x, i                    

                    dx = np.linalg.solve(Jac, fO)
                    x = x - dx
            raise ("Too many iterations for the Newton method")
    
    def f(x):
        f = np.zeros([3])

        r = calc(x[0], x[1], x[2])
        f[0] = r[0]-t_x
        f[1] = r[1]-t_y
        f[2] = r[2]-t_z

        return f
    
    x0 = np.zeros([3])
    print("f(x):", f(x0))

    r, iter = newton(f, x0)
    print ('Solution:', r)
    print ('Newton iteration = ', iter)
    
    print (calc(r[0], r[1], r[2])) #-> XYZ (335.604, -10.014, -92.959)

