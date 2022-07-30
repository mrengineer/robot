from array import array
import math
import time, sys, os
from struct import *
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


class Motor():
    serial_port                    = None
    id: hex                        = 0x00
    name: str                      = None
    __cur_multi_loop_angle: float    = 0.0
    
    tolerance: float               = 0.1 #deg. Used for different operatins like wait_stop Must be > 0.01!

    # Simulation rotation
    sim_CW: bool                   = True
    
    # Tooltip shift. Motor's zero point is bottom axis of motor.
    sim_shifts                     = [0.0, 0.0, 0.0]
    sim_rot_plane: str             = "YZ"

    def __init__(self, id: hex, serial_port, tolerance: float, name: str, CW: bool, sim_shifts: list, sim_rot_plane: str):
        assert id           >= 0
        assert tolerance    >= 0.01
        #assert serial_port  != None    #no check for case of simulation
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
        
        if not self.serial_port == None:
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
        else:  #if no serial port than simulate
            res = 1
            self.__cur_multi_loop_angle = angle

        return res

    def inc_angle_speed(self, angle: float, speed: float):
        assert speed > 0, "Speed must be grater than zero"

        if not self.serial_port == None:
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
        else:  #if no serial port than simulate
            res = 1
            self.__cur_multi_loop_angle = self.__cur_multi_loop_angle + angle
        return res

    # 0 ... 365.99 deg
    #NB! simulation when serial port is not set is not implemented!
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
        if not self.serial_port == None:
            header_crc = (CMD_HEADER + CMD_ASK_MULTI_LOOP_ANGLE + self.id + 0x00) % 256
            r = bytearray(pack('BBBBB', CMD_HEADER, CMD_ASK_MULTI_LOOP_ANGLE, self.id, 0x00, header_crc))

            self.serial_port.write(r)

            #GET RESPONCE FROM MOTOR
            #TODO add motor ID check

            res = self.__read_responce(14)
            
            angle = unpack("HHHH", res[5:13])
            
            #update value beore return
            self.__cur_multi_loop_angle = float(angle[0]/100)        
            #if not self.CW:  r_angle = -1 * r_angle

        return self.__cur_multi_loop_angle

    #blocks code execution till motor stop (angle do not change because of ANY reason)
    #tolerance to detect angle similarity,  request_period to reduce ammout of requests
    def wait_stop(self, request_period: float, timeout: float):
        prev_value  = 9999999
        start       = time.time()

        assert request_period   >= 0.01
        assert timeout          >= 0.1, "Timeout must be > 0.1 sec"

        while (time.time() - start < timeout):
            cur_multi_loop_angle   = self.get_multi_loop_angle()
            delta                       = abs(prev_value - cur_multi_loop_angle)
            
            if (delta < self.tolerance): return

            prev_value = cur_multi_loop_angle
            time.sleep(request_period)

        raise TimeoutError("Motor does not stopped in defined time")

    '''
    ------- SIMULATION FUNCTIONS -------  USE FOR simulation and angles <-> coordinates conversion
    See https://www.bsuir.by/m/12_113415_1_70397.pdf
    https://studref.com/472293/tehnika/matrichnye_metody_preobrazovaniya_koordinat_robototehnike?
    '''
    
    def T(self, angle_deg: float) -> np.array:
        angle_deg = math.radians(angle_deg)

        if self.sim_CW:   k = 1
        else:             k = -1
        
        #Generate translation and rotation matrix T        

        
        Ts = np.array([   [1, 0,  0,  self.sim_shifts[0]],          #shift matrix
                          [0, 1,  0,  self.sim_shifts[1]],
                          [0, 0,  1,  self.sim_shifts[2]],
                          [0, 0,  0,  1]  ])
        
        sine = math.sin(k*angle_deg)
        cosine = math.cos(k*angle_deg)
               
        if self.sim_rot_plane == "XY":
            Ta = np.array([ [cosine , 0 , -sine , 0],       #Rotation affects X axis
                            [-sine, 0 ,  cosine , 0],       #Rotation affects Y axis (ie XY plane)
                            [0    , 0 , 1 , 0],                            
                            [0    , 0 , 0 , 1]    ])  #Scale always 1
        elif self.sim_rot_plane == "XZ":
            Ta = np.array([ [cosine , 0 , -sine , 0],       #Rotation affects X axis
                            [0      , 1 , 0     , 0],
                            [-sine  , 0 , cosine , 0],       #Rotation affects Z axis (ie XZ plane)
                            [0      , 0 , 0      , 1]    ])
        elif self.sim_rot_plane == "YZ":
            Ta = np.array([ [1, 0 , 0 , 0],       #X axis (no changes because of angle)
                            [0, cosine , -sine  , 0],       #Rotation in YZ plane
                            [0, sine ,  cosine , 0],
                            [0, 0 , 0  , 1]    ])            
        
        else:
            raise Exception('Wrong rotation plane. It must be XY, XZ or YZ')
        
        return np.matmul(Ta, Ts)         

class Robot():
    __motors = list()
    __port: serial.Serial = None    #if sumulation, serial port is not assigned

    def __init__(self, port_name: str) -> None:
        try:
            self.__port = serial.Serial(
                port        =   f"/dev/{port_name}",
                baudrate    =   115200,
                parity      =   serial.PARITY_NONE,
                stopbits    =   serial.STOPBITS_ONE,
                bytesize    =   serial.EIGHTBITS,
                #timeout=1
            )

        except Exception as e:
            print ("Error open serial port: " + str(e))

    def add_motor(self, 
                    id: hex, tolerance: float, 
                    name: str, CW: bool, sim_shifts: list, 
                    sim_rot_plane: str) -> Motor :

        new_motor = Motor(id, self.__port, tolerance, name, CW, sim_shifts, sim_rot_plane)
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

    # Calculates coordinates usng angles
    def sim_angles_to_coords(self, angles: np.array) -> np.array:
        T       = np.identity(4)
        result  = list()
        P       = list()

        for motor, angle in zip(self.__motors, angles):
            try:
                angle   = float(angle)
                Tm      = motor.T(angle)
                T       = np.matmul(T, Tm)
                P.append([0]) # build 1 coloum matrix
            except ValueError as e:
                print("Error in sim_angles_to_coords", str(e))
                exit()

        P.append([1]) # end of creating matrix 1 coloumn. Last one is scale factor

        R = np.dot(T, np.array(P))

        for r in R[:-1]: result.append(r[0])

        return np.array(result)
    
    # Calculates chains angles using from target XYZ coordinates
    #@time_of_function
    def sim_coords_to_angles(self, target: np.array, guess: np.array): # -> np.array, np.array:
        def jacobian(f, x: np.array, h = 0.01):
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
        
        def newton(f, x: np.array, tol=1.0e-2, h = 0.002):
            iterMax = 350
            for i in range(iterMax):
                Jac, fO = jacobian(f, x, h)
                err = math.sqrt(np.dot(fO, fO) / len(x))

                if err < tol:   return x, i

                dx = np.linalg.solve(Jac, fO)
                x = x - dx
            raise ("Too many iterations for the Newton method")
        
        def f(x: np.array):
            f = np.zeros([3])
            r = self.sim_angles_to_coords([x[0], x[1], x[2]])
            f = r - target

            return f

        
        
        r, iter = newton(f, guess)
    
      
        # this area suould be reconsidered for linear axises
        r[0] = r[0] - int(r[0] / 360) * 360  #Reduce angles to range +/- 360 deg or U get > 360 deg
        r[1] = r[1] - int(r[1] / 360) * 360
        r[2] = r[2] - int(r[2] / 360) * 360
        

        #result and tolerance

        C_XYZ = robot.sim_angles_to_coords(r)
        TOL = [C_XYZ[0] - target[0], C_XYZ[1] - target[1], C_XYZ[2] - target[2]]
      
        return r.round(3), TOL

    def get_coords(self) -> np.array:
        return self.sim_angles_to_coords(self.get_multi_loop_angles())

    

# MAIN
if __name__ == '__main__':
     

    target = np.array([335.6, -10., -92.96])

    #use previous angles for previous point as guess to achieve shortest change in angles
    guess = np.zeros([3])
    guess[0] = 9
    guess[1] = 9
    res = robot.sim_coords_to_angles(target, guess)

    print ("Checking angles calculation")

    print ("sim_angles_to_coords", robot.sim_angles_to_coords([10, 10, 30])) #-> XYZ (335.604, -10.014, -92.959)

    print ('TARGET COORDINATES:', target)
    print ('ANGLES SOLUTION:', res[0])
    print ('RESULT TOLERANCE', res[1])
    print ("\n\n")
    
    

    if ser.isOpen():
        try:
            ser.flushInput()    #flush input buffer, discarding all its contents
            ser.flushOutput()   #flush output buffer, aborting current output
            
            while(True):
                #robot.goto_abs_multi_loop_angles_speeds([alpha, fi, theta],    [30, 30, 40])

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
                

                print("Enter alpha fi theta angles Ex.: 4.0 -11 4")
                print("get_multi_loop_angles", robot.get_multi_loop_angles())
                inp = input()
                a, f, t = inp.split(" ")

                alpha   = float(a)
                fi      = float(f)
                theta   = float(t)

                print("DEGREES", alpha, fi, theta)

                print(robot.sim_angles_to_coords([alpha, fi, theta]))
                robot.goto_abs_multi_loop_angles_speeds([alpha, fi, theta],  [60, 80, 80])

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

