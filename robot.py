from array import array
import copy
import math
from multiprocessing.dummy import Array
import os
import time, sys
from struct import *
from xmlrpc.client import Boolean

import numpy as np
import serial

CMD_HEADER                     = 0x3E
CMD_ASK_MULTI_LOOP_ANGLE       = 0x92        #Read multi -loop Angle command
CMD_ASK_SINGLE_LOOP_ANGLE      = 0x94        #Read single -loop Angle command
CMD_ABS_MULTI_LOOP_ANGLE_SPEED = 0xA4        #MULTI ROTATION ABS ANGLE Multi position closed loop control command 2
CMD_ABS_ANGLE_SPEED            = 0xA6        #Single position closed loop control command 2
CMD_INC_ANGLE_SPEED            = 0xA8        #INCREMENT angle with speed
CMD_SET_ZERO                   = 0x19        #Set current poosition as zero for driver
CMD_MOTOR_SHUTDOWN             = 0x80        #MOTOR stop (but power on coils will keep?)
CMD_MOTOR_STOP                 = 0x81        #MOTOR stop (but power on coils will keep?)
CMD_MOTOR_START                = 0x88        #MOTOR operation
CMD_MOTOR_MODEL                = 0x12        #Read driver and motor model commands


def time_of_function(function):
    def wrapped(*args):
        start_time = time.perf_counter_ns()
        res = function(*args)
        print((time.perf_counter_ns() - start_time) / 10**6, "millisec")
        return res
    return wrapped

def frange(start, stop, step):
    i = start
    while i < stop:
        yield i
        i += step

class Motor():
    serial_port         = None
    id: hex             = 0x00
    name: str           = None

    
    tolerance: float = 0.1 #deg. Used for different operatins like wait_stop Must be > 0.01!

    # Simulation rotation
    sim_CW: bool       = True    
    
    # Tooltip shift. Motor's zero point is bottom axis of motor.
    sim_shifts             = [0.0, 0.0, 0.0]
    sim_rot_plane: str     = "YZ"

    def __init__(self, id: hex, serial_port, tolerance: float, name: str, CW: bool, sim_shifts: list, sim_rot_plane: str):
        assert id           >= 0
        assert tolerance    >= 0.01
        assert serial_port  != None
        assert name         != None
        
        #Yep, I know about enums!
        assert sim_rot_plane.upper() in ["XY", "XZ", "YZ"], "Wrong rotation plane. It must be XY, XZ or YZ"        

        self.serial_port    = serial_port
        self.id             = id
        self.tolerance      = tolerance
        self.name           = name

        self.sim_CW         = CW
        self.sim_shifts     = sim_shifts
        self.sim_rot_plane  = sim_rot_plane.upper()
    
    def __read_responce(self, bytes_expect: int):
        self.serial_port.timeout = 0.1
        
        res = self.serial_port.read(bytes_expect)

        if (res is None or (len(res) != bytes_expect)): 
            raise TimeoutError(f"Motor does not responded {bytes_expect} byte(s), result is '{res}'")

        return res
    
    # Writes the current encoder position of the motor into ROM as the initial position.  Attention:
    # 1. This command needs to restart to take effect.
    # 2. This command will write zero point into ROM of the driver, multiple writing will affect the chip life,which is not recommended for frequent use
    def set_zero_cur_position(self):
        data_length = 0x00        
        header_crc  = (CMD_HEADER + CMD_SET_ZERO + self.id + data_length) % 256
        
        snd = bytearray(pack('<BBBBB', CMD_HEADER, CMD_SET_ZERO, self.id, data_length, header_crc))
        self.serial_port.write(snd)        
        res = self.__read_responce(26)      #wait 26 bytes

    def abs_multi_loop_angle_speed(self, angle: float, speed: float):
        assert speed > 0, "Speed must be grater than zero"

        #if not self.CW:  angle = -1 * angle
        
        data_length = 0x0C
        
        header_crc  = (CMD_HEADER + CMD_ABS_MULTI_LOOP_ANGLE_SPEED + self.id + data_length) % 256

        angle       = int(angle * 100)
        speed       = int(speed * 100)    #according documentaton

        data        = pack('<qi', angle, speed)
        data_crc    = sum(data) % 256

        snd = bytearray(pack('<BBBBBqiB', CMD_HEADER, CMD_ABS_MULTI_LOOP_ANGLE_SPEED, self.id, data_length, header_crc, angle, speed, data_crc))
        
        self.serial_port.write(snd)

        res = self.__read_responce(13)      #wait 13 bytes

        return res

    def inc_angle_speed(self, angle: float, speed: float):
        assert speed > 0, "Speed must be grater than zero"

        data_length = 0x08

        #if not self.CW:  angle = -1 * angle
        
        header_crc  = (CMD_HEADER + CMD_INC_ANGLE_SPEED + self.id + data_length) % 256

        angle       = int(angle * 100)
        speed       = int(speed * 100)    #according documentaton

        data        = pack('<ii', angle, speed)
        data_crc    = sum(data) % 256

        snd = bytearray(pack('<BBBBBiiB', CMD_HEADER, CMD_INC_ANGLE_SPEED, self.id, data_length, header_crc, angle, speed, data_crc))
        
        print(" ".join(map(lambda b: format(b, "02x"), snd)))
        self.serial_port.write(snd)

        res = self.__read_responce(13)      #wait 13 bytes

        return res

    # 0 ... 365.99 deg
    def get_single_loop_angle(self):
        header_crc = (CMD_HEADER + CMD_ASK_SINGLE_LOOP_ANGLE + self.id + 0x00) % 256
        r = bytearray(pack('BBBBB', CMD_HEADER, CMD_ASK_SINGLE_LOOP_ANGLE, self.id, 0x00, header_crc))

        self.serial_port.write(r)

        #GET RESPONCE FROM MOTOR
        #TODO add motor ID check

        time.sleep(.01)  #give the serial port sometime to receive the data
        res = self.__read_responce(8)

        b = bytearray()
        b.append(res[5])
        b.append(res[6])
        
        angle = unpack("H", b)

        r_angle = float(angle[0]/100)        
        #if not self.CW:  r_angle = -1 * r_angle

        return r_angle

    # 0 ... INF deg
    def get_multi_loop_angle(self):
        header_crc = (CMD_HEADER + CMD_ASK_MULTI_LOOP_ANGLE + self.id + 0x00) % 256
        r = bytearray(pack('BBBBB', CMD_HEADER, CMD_ASK_MULTI_LOOP_ANGLE, self.id, 0x00, header_crc))

        self.serial_port.write(r)

        #GET RESPONCE FROM MOTOR
        #TODO add motor ID check

        #time.sleep(.01)  #give the serial port sometime to receive the data
        res = self.__read_responce(14)
        
        angle = unpack("HHHH", res[5:13])
        
        r_angle = float(angle[0]/100)        
        #if not self.CW:  r_angle = -1 * r_angle

        return r_angle

    #blocks code execution till motor stop (angle do not change because of ANY reason)
    #tolerance to detect angle similarity,  request_period to reduce ammout of requests
    def wait_stop(self, request_period: float, timeout: float):
        prev_value  = 9999999
        start       = time.time()

        assert request_period   >= 0.01
        assert timeout          >= 0.1, "Timeout must be > 0.1 sec"

        while (time.time() - start < timeout):
            cur_angle   = self.get_multi_loop_angle()
            delta       = abs(prev_value-cur_angle)
            
            if (delta < self.tolerance): return

            prev_value = cur_angle
            time.sleep(request_period)

        raise TimeoutError("Motor does not stopped in defined time")

    # ------- SIMULATION FUNCTIONS -------  USE FOR simulation and angles <-> coordinates conversion
    # See https://www.bsuir.by/m/12_113415_1_70397.pdf
    # https://studref.com/472293/tehnika/matrichnye_metody_preobrazovaniya_koordinat_robototehnike?
    
    def T(self, angle_deg: float) -> np.array:
        angle_deg = math.radians(angle_deg)

        if self.sim_CW:   k = 1
        else:             k = -1
        
        #Generate translation and rotation matrix T        

        
        Ts = np.array([   [1, 0,  0,  self.sim_shifts[0]],          #shift matrix
                          [0, 1,  0,  self.sim_shifts[1]],
                          [0, 0,  1,  self.sim_shifts[2]],
                          [0, 0,  0,  1]  ])
        #print("TS:", Ts)
               
        if self.sim_rot_plane == "XY":
            Ta = np.array([ [math.cos(k*angle_deg) , 0 , -math.sin(k*angle_deg) , 0],       #Rotation affects X axis
                            [-math.sin(k*angle_deg), 0 ,  math.cos(k*angle_deg) , 0],       #Rotation affects Y axis (ie XY plane)
                            [0                     , 0 , 1                      , 0],                            
                            [0                     , 0 , 0                      , 1]    ])  #Scale always 1
        elif self.sim_rot_plane == "XZ":
            Ta = np.array([ [math.cos(k*angle_deg) , 0 , -math.sin(k*angle_deg) , 0],       #Rotation affects X axis
                            [0                     , 1 , 0                      , 0],
                            [-math.sin(k*angle_deg), 0 ,  math.cos(k*angle_deg) , 0],       #Rotation affects Z axis (ie XZ plane)
                            [0                     , 0 , 0                      , 1]    ])
        elif self.sim_rot_plane == "YZ":
            Ta = np.array([ [1, 0                     , 0                       , 0],       #X axis (no changes because of angle)
                            [0, math.cos(k*angle_deg) , -math.sin(k*angle_deg)  , 0],       #Rotation in YZ plane
                            [0, math.sin(k*angle_deg) ,  math.cos(k*angle_deg)  , 0],
                            [0, 0                     , 0                       , 1]    ])            
        
        else:
            raise Exception('Wrong rotation plane. It must be XY, XZ or YZ')
        
        return np.matmul(Ta, Ts)         

class Robot():
    __motors = list()

    def __init__(self) -> None:
        pass

    def add_motor(self, 
                    id: hex, serial_port, tolerance: float, 
                    name: str, CW: bool, sim_shifts: list, 
                    sim_rot_plane: str) -> Motor :

        new_motor = Motor(id, serial_port, tolerance, name, CW, sim_shifts, sim_rot_plane)
        self.__motors.append(new_motor)
        return new_motor

    def goto_zero(self, speed: float):
        for motor in self.__motors:
            motor.abs_multi_loop_angle_speed(0, speed)
        self.wait_stop()

    def goto_abs_multi_loop_angles_speeds(self, angles: list, speeds: list):
        assert len(angles) == len(speeds) == len(self.__motors), "Ammout of motors, speeds and angles must be same"

        for motor, angle, speed in zip(self.__motors, angles, speeds):
            try:
                angle = float(angle)
                speed = float(speed)
            except ValueError as e:
                print("Error in robot.goto_abs_multi_loop_angles_speeds", str(e))
                exit()
            motor.abs_multi_loop_angle_speed(angle, speed)
        self.wait_stop()

    #blocks code execution till ALL motors stop (angle do not change because of ANY reason)
    #tolerance to detect angle similarity,  request_period to reduce ammout of requests
    def wait_stop(self, request_period: float = 0.1, motor_timeout: float = 15):
        for motor in self.__motors:
            motor.wait_stop(request_period, motor_timeout)

    def get_multi_loop_angles(self):
        results = list()

        for motor in self.__motors:
            results.append(motor.get_multi_loop_angle())
        
        return results

    def sim_angles_to_coords(self, angles):
        T = np.identity(4)
        result = list()
        P = list()

        for motor, angle in zip(self.__motors, angles):
            try:
                angle   = float(angle)
                Tm      = motor.T(angle)
#                print("R", Tm)
#                print("-"*30)
                T       = np.matmul(T, Tm)
                P.append([0]) # build 1 coloum matrix
            except ValueError as e:
                print("Error in robot.sim_angles_to_coords", str(e))
                exit()

        P.append([1]) # end of creating matrix 1 coloumn. Last one is scale factor

        R = np.dot(T, np.array(P))

        for r in R[:-1]: result.append(round(r[0], 2))

        return np.array(result)
        
robot = Robot()

precalcs = Array

#@time_of_function
def precalcs(s_alpha, s_fi, s_theta, target):
    tol = 0.02
    
    #current alpha angle
    alpha = s_alpha
    fi = s_fi
    theta = s_theta
 

    alpha_step = 18
    fi_step = 18
    theta_step = 18

    steps = 0
    prev_step_dist = 0
    same_steps = 0

    while (steps < 600):
        steps = steps + 1
        
        
        #ALPHA AXIS
        
        dist    = np.linalg.norm(target - (robot.sim_angles_to_coords([alpha, fi, theta])))

        ccw_alpha = alpha - alpha_step
        ccw_dist  = np.linalg.norm(target - robot.sim_angles_to_coords([ccw_alpha, fi, theta]))
            
        cw_alpha = alpha + alpha_step
        cw_dist  = np.linalg.norm(target - robot.sim_angles_to_coords([cw_alpha, fi, theta]))

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
        
        dist    = np.linalg.norm(target - (robot.sim_angles_to_coords([alpha, fi, theta])))

        ccw_fi = fi - fi_step
        ccw_dist  = np.linalg.norm(target - robot.sim_angles_to_coords([alpha, ccw_fi, theta]))
            
        cw_fi = fi + fi_step
        cw_dist  = np.linalg.norm(target - robot.sim_angles_to_coords([alpha, cw_fi, theta]))

        if cw_dist < ccw_dist:
                n_dist = cw_dist
                n_fi = cw_fi
                print ("+", fi_step)
        else:
                n_dist = ccw_dist
                n_fi = ccw_fi
                print ("-", fi_step)
            
        if dist - n_dist > tol:
                fi = n_fi
                #print(f"{steps} FI = {round(fi, 2)} dist = {round(dist, 2)} new dist {round(n_dist,2)}    at fi_step= {round(fi_step, 2)}")
        else:
                if fi_step > 0.02: fi_step = round(fi_step * 0.80, 3)



        #THETA AXIS
        
        dist    = np.linalg.norm(target - (robot.sim_angles_to_coords([alpha, fi, theta])))

        ccw_theta = theta - theta_step
        ccw_dist  = np.linalg.norm(target - robot.sim_angles_to_coords([alpha, fi, ccw_theta]))
            
        cw_theta = theta + theta_step
        cw_dist  = np.linalg.norm(target - robot.sim_angles_to_coords([alpha, fi, ccw_theta]))

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

        step_dist  = np.linalg.norm(target - (robot.sim_angles_to_coords([alpha, fi, theta])))
        #print (step_dist)

        if step_dist < 0.11: break

        if step_dist == prev_step_dist:
            same_steps = same_steps + 1
            if (same_steps == 15): break
        else:
            prev_step_dist = step_dist
            same_steps = 0

    print("A %3.2f  F %3.2f T %3.2f -> %4.3f @ %5d stps" % (round(alpha, 2), round(fi,2), round(theta, 2), round(dist, 3), steps))


    
    '''
    for alpha in range(-180, 180+1, 5):
        for fi in range(-180, 180+1, 5):
            for theta in range(-180, 180+1, 5):
                x,y,z = robot.sim_angles_to_coords([alpha, fi, theta])
                #dist = np.linalg.norm(target - coords)
                i = i + 1

    print("I=", i)
    '''
#    print(robot.sim_angles_to_coords([90, 0, 0]))

# MAIN
def main():
    try:
        ser = serial.Serial(
            port        =   '/dev/ttyUSB0',
            baudrate    =   115200,
            parity      =   serial.PARITY_NONE,
            stopbits    =   serial.STOPBITS_ONE,
            bytesize    =   serial.EIGHTBITS,
            #timeout=1
        )
    except Exception as e:
        print ("Error open serial port: " + str(e))
        exit()

        
    m1 = robot.add_motor(0x01, ser, 0.1, "X", False, [112.2, -45, 0],  "YZ")
    m2 = robot.add_motor(0x02, ser, 0.1, "Y1", False, [126, 53, 0], "XZ")
    m3 = robot.add_motor(0x03, ser, 0.1, "Y2", False, [105.7, -34, 0], "XZ")

    '''
    print("Set angles as ZERO positions? Y=yes")
    inp = input().upper()
    if (inp == "Y"):
        #save zero positions for axes
        m1.set_zero_cur_position()
        m2.set_zero_cur_position()
        m3.set_zero_cur_position()
        print("RESET POWER!")
        exit()
    '''

    #robot.sim_angles_to_coords([cw_alpha, fi, theta])

    #precalcs(-170, -170, -170)
    #precalcs(0, 0, 0)
    #precalcs(170, 170, 170)
    #precalcs(90, 90, 90, np.array([243, 53.6, 64.5]))
    
    exit()
    target = np.array([344, -25.6, 4.5])

    alpha = 0
    fi = 0
    theta = 0

    tol = 0.03
    i = 0
    j = 0
    for alpha in frange(-180.0, 180.+ 0.1, 3):
        for fi in frange(-180.0+18, 180+0.1-18, 3):
            for theta in frange(-180.+9, 180+0.1-9, 3):
                x,y,z = robot.sim_angles_to_coords([alpha, fi, theta])
                j = j + 1
                if ((abs(x - round(x, 0)) < tol) and 
                    (abs(y - round(y, 0)) < tol) and
                    (abs(z - round(z, 0)) < tol)):
                    i = i + 1
                    print(f"{j}>{i} {x} {y} {z} - {alpha} {fi} {theta}")
                                
            

                #dist = np.linalg.norm(target - coords)
                #if dist < 15:
                #   print(f"{dist} - {alpha} {fi} {theta}")

    #print(robot.sim_angles_to_coords([alpha, fi, theta]))



    exit()

    if ser.isOpen():
        try:
            ser.flushInput()    #flush input buffer, discarding all its contents
            ser.flushOutput()   #flush output buffer, aborting current output
            

            #GO ZERO pos
            #robot.goto_zero(10)                      
            
            #save zero positions for axes
            #motor1.set_zero_cur_position()
            #motor2.set_zero_cur_position()
            #motor3.set_zero_cur_position()


            while(True):
                robot.goto_abs_multi_loop_angles_speeds([alpha, fi, theta],    [30, 30, 40])

                '''
                print("Set angles as ZERO positions? Y=yes")
                inp = input().upper()
                if (inp == "Y"):
                    #save zero positions for axes
                    m1.set_zero_cur_position()
                    m2.set_zero_cur_position()
                    m3.set_zero_cur_position()
                    print("RESET POWER!")
                '''
                

                print("Enter alpha fi theta angles Ex.: 4.0 -11 4")
                print(robot.get_multi_loop_angles())
                inp = input()
                a, f, t = inp.split(" ")

                alpha   = float(a)
                fi      = float(f)
                theta   = float(t)

                print("DEGREES", alpha, fi, theta)

                robot.sim_angles_to_coords([alpha, fi, theta])

            exit()
            
                        
            #Moves in cycle
            while True:
                robot.goto_abs_multi_loop_angles_speeds([15, 30, 30],    [60, 80, 80])
                time.sleep(1)
                robot.goto_abs_multi_loop_angles_speeds([0, 0, 0],       [60, 80, 80])
                time.sleep(1)
                robot.goto_abs_multi_loop_angles_speeds([-10, -30, -30], [60, 80, 80])              
                time.sleep(1)
                print(robot.get_multi_loop_angles())

            ser.close()
        except Exception as e1:
            print ("error communicating...: " + str(e1))

    else:
        print("cannot open serial port")

if __name__ == '__main__':
    #os.system('clear')
    main()
