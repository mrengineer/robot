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
    #os.system('clear')
    s = []  #x+ y+ z+
    print("Start")

    def get_segm(coords):
        ix = 0
        i = 0
        for p in coords:
            if p < 0:   ix = ix | (1<<i)
            i = i + 1        
        return ix

    for seg in range(0, 8):               #0...7 segments of axis
        s.append([seg, []])

    print("Fast search zones cnt:", len(s))
    
    

    def opt_calc(ar):
        a, f, t = ar
        res = calc(a, f, t)

        #Angles 10 10 30
        x = 335.604
        y = -10.014
        z = -92.959

        target = np.array([x, y, z])
        return np.linalg.norm(target - res)

    def calc2(inp):
        return calc(inp[0], inp[1], inp[2])

    print("Function check")
    #print(opt_calc([0,0,0]))
    #print(opt_calc([90,0,0]))
    #print(opt_calc([90,90,0]))
    #print(opt_calc([0,20,-20]))

    print (calc(10, 10, 30)) #-> XYZ (335.604, -10.014, -92.959)

    n=3

    
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
                    #print (i, err)

                    if err < tol:   return x, i                    

                    dx = np.linalg.solve(Jac, fO)
                    x = x - dx
            raise ("Too many iterations for the Newton method")
    
    
    def f(x):
        f = np.zeros([n])

        t_x = 335.604
        t_y = -10.014
        t_z = -92.959

        r = calc(x[0], x[1], x[2])
        f[0] = r[0]-t_x
        f[1] = r[1]-t_y
        f[2] = r[2]-t_z

        return f
    
    x0 = np.zeros([n])
    print("f(x):", f(x0))
    
    
    #x, iter 
    r, iter = newton(f, x0)
    print ('Solution:\n', r)
    print ('Newton iteration = ', iter)
    
    print (calc(r[0], r[1], r[2])) #-> XYZ (335.604, -10.014, -92.959)


    exit()


    def add(a, f, t):
        x, y, z = calc(a, f, t)

        ix = None
        d = int((x*x+y*y+z*z)**0.5)
        seg = get_segm([x, y, z])
        #k = s[seg]
        #ix = k[d]
        ix = s[seg]
        
        ix[1].append([np.array([x, y, z]), np.array([a, f, t])])

    @time_of_function
    def find(x, y, z):
        target  = np.array([x, y, z])
        seg     = get_segm([x, y, z])

        ix      = s[seg]
        
        #d       = int((x*x+y*y+z*z)**0.5)
        #print("FIND in index ", d, "with values", len(ix[d][1]))        
        #res = min(ix[d][1], key=lambda v: np.linalg.norm(target - v[0]))

        res = min(ix[1], key=lambda v: np.linalg.norm(target - v[0]))
        d = np.linalg.norm(target - res[0])
        print("DIST to selected", d)
        return res

    @time_of_function
    def find_angles(s_alpha, s_fi, s_theta, target):
        tol = 0.02
        
        #current alpha angle
        alpha = s_alpha
        fi = s_fi
        theta = s_theta
    

        alpha_step = 3
        fi_step = 3
        theta_step = 3

        steps = 0
        prev_step_dist = 0
        same_steps = 0

        while (steps < 600):
            steps = steps + 1
            
            
            #ALPHA AXIS
            cord    = calc(alpha, fi, theta)            
            dist    = np.linalg.norm(target - cord)

            ccw_alpha = alpha - alpha_step            
            ccw_dist  = np.linalg.norm(target - calc(ccw_alpha, fi, theta))
                
            cw_alpha = alpha + alpha_step
            cw_dist  = np.linalg.norm(target - calc(cw_alpha, fi, theta))

            if cw_dist < ccw_dist:
                    n_dist = cw_dist
                    n_alpha = cw_alpha
            else:
                    n_dist = ccw_dist
                    n_alpha = ccw_alpha
                
            if dist - n_dist > tol:
                    alpha = n_alpha
                    #print(f"{steps} ALPHA = {round(alpha, 2)} dist = {round(dist, 2)} new dist {round(n_dist,2)}    at alpha_step= {round(alpha_step, 2)}")
            else:
                    if alpha_step > 0.02: alpha_step = round(alpha_step * 0.80, 3)



            #FI AXIS
            
            dist    = np.linalg.norm(target - calc(alpha, fi, theta))

            ccw_fi = fi - fi_step
            ccw_dist  = np.linalg.norm(target - calc(alpha, ccw_fi, theta))
                
            cw_fi = fi + fi_step
            cw_dist  = np.linalg.norm(target - calc(alpha, cw_fi, theta))

            if cw_dist < ccw_dist:
                    n_dist = cw_dist
                    n_fi = cw_fi
            else:
                    n_dist = ccw_dist
                    n_fi = ccw_fi
                
            if dist - n_dist > tol:
                    fi = n_fi
                    #print(f"{steps} FI = {round(fi, 2)} dist = {round(dist, 2)} new dist {round(n_dist,2)}    at fi_step= {round(fi_step, 2)}")
            else:
                    if fi_step > 0.02: fi_step = round(fi_step * 0.80, 3)



            #THETA AXIS
            
            dist    = np.linalg.norm(target - calc(alpha, fi, theta))

            ccw_theta = theta - theta_step
            ccw_dist  = np.linalg.norm(target - calc(alpha, fi, ccw_theta))
                
            cw_theta = theta + theta_step
            cw_dist  = np.linalg.norm(target - calc(alpha, fi, ccw_theta))

            if cw_dist < ccw_dist:
                    n_dist = cw_dist
                    n_theta = cw_theta
                    
            else:
                    n_dist = ccw_dist
                    n_theta = ccw_theta
                    
                
            if dist - n_dist > tol:
                    theta = n_theta
                    #print(f"{steps} THETA = {round(theta, 2)} dist = {round(dist, 2)} new dist {round(n_dist,2)}    at fi_step= {round(theta_step, 2)}")
            else:
                    if theta_step > 0.02:  theta_step = round(theta_step * 0.80, 3)

            step_dist  = np.linalg.norm(target - calc(alpha, fi, theta))
            #print (step_dist)

            if step_dist < 0.11: break

            if step_dist == prev_step_dist:
                same_steps = same_steps + 1
                if (same_steps == 18): break
            else:
                prev_step_dist = step_dist
                same_steps = 0

        print("A %3.2f  F %3.2f T %3.2f -> dist %4.3f @ %5d stps" % (round(alpha, 2), round(fi,2), round(theta, 2), round(dist, 3), steps))


    xs = [0]
    ys = [0]
    zs = [0]

    c = 0
    for a in range (-180, 180+1, 3):
        for f in range (0, 120+1, 3):
            for t in range (-90, 90+1, 3*3):
                add(a, f, t)

                co = calc(a, f, t)

                xs.append(co[0])
                ys.append(co[1])
                zs.append(co[2])
                c =c +1

    print(c)


    print("Search angles")

    x = 100.3
    y = 51.0
    z = 49.1
    
    target = np.array([x, y, z])

    r=find(x, y, z)         #closest point and angles
    print(r, "\r\r\r")


    find_angles(r[1][0], r[1][1], r[1][2], target)



    x = 50.3
    y = -131.0
    z = 58.1
    
    target = np.array([x, y, z])

    r=find(x, y, z)
    print(r, "\r\r\r")

    find_angles(r[1][0], r[1][1], r[1][2], target)





    
    '''

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.plot([0, 350], [0, 0], [0, 0], label='ax', color="red")
    ax.plot([0, 0], [0, 350], [0, 0], label='ax', color="green")
    ax.plot([0, 0], [0, 0], [0, 350], label='ax', color="blue")

    ax.scatter3D(xs, ys, zs, s=1, color='black')


    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    #fig.tight_layout()
    #plt.show()

'''