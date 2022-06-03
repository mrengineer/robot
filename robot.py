import copy, time, serial, math
from xmlrpc.client import Boolean
import pytest
from struct import *


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

class Motor():
    serial_port = None
    id: hex     = 0x00
    name: str   = None

    tolerance: float = 0.1 #deg. Used for different operatins like wait_stop Must be > 0.01!

    # Simulation rotation
    sim_CW: bool       = True

    # Tooltip shift. Motor's zero point is bottom axis of motor.
    sim_x_shift: float = 0.0
    sim_y_shift: float = 0.0
    sim_z_shift: float = 0.0


    def __init__(self, id: hex, serial_port, tolerance: float, name: str):
        assert id >= 0
        assert tolerance >= 0.01
        assert serial_port != None
        assert name != None

        self.serial_port = serial_port
        self.id          = id
        self.tolerance   = tolerance
        self.name        = name

    
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


class Robot():
    __motors = list()

    def __init__(self) -> None:
        pass

    def add_motor(self, id: hex, serial_port, tolerance: float, name: str):
        new_motor = Motor(id, serial_port, tolerance, name)
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

    if ser.isOpen():
        try:
            ser.flushInput()    #flush input buffer, discarding all its contents
            ser.flushOutput()   #flush output buffer, aborting current output
            
            robot = Robot()
            robot.add_motor(0x01, ser, 0.1, "X")
            robot.add_motor(0x02, ser, 0.1, "Y")
            robot.add_motor(0x03, ser, 0.1, "Y2CCW")


            #GO ZERO pos
            robot.goto_zero(10)
                        
            
            #save zero positions for axes
            #motor1.set_zero_cur_position()
            #motor2.set_zero_cur_position()
            #motor3.set_zero_cur_position()
                        
            #Moves in cycle
            while True:

                #motor1.abs_multi_loop_angle_speed(0.0, 10)
                #motor2.abs_multi_loop_angle_speed(0.0, 10)
                #motor3.abs_multi_loop_angle_speed(-90.0, 50)

                robot.goto_abs_multi_loop_angles_speeds([15, 30, 30], [40, 60, 60])

                time.sleep(1)
                robot.goto_abs_multi_loop_angles_speeds([0, 0, 0], [40, 60, 60])

                time.sleep(1)
                robot.goto_abs_multi_loop_angles_speeds([-10, -30, -30], [40, 60, 60])

                #print("M1 ", motor1.get_multi_loop_angle(), ", M2", motor2.get_multi_loop_angle(), ", M3 ", motor3.get_multi_loop_angle())
                time.sleep(1)

            ser.close()
        except Exception as e1:
            print ("error communicating...: " + str(e1))

    else:
        print("cannot open serial port")

if __name__ == '__main__':
    main()