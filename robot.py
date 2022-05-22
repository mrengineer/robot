import copy, time, serial, math
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

    def __init__(self, id: hex, serial_port):
        assert id >= 0
        assert serial_port != None

        self.serial_port = serial_port
        self.id          = id
    
    def __read_responce(self, bytes_expect: int):
        self.serial_port.timeout = 0.1
        
        res = ser.read(bytes_expect)

        if (res is None or (len(res) < bytes_expect)): 
            raise TimeoutError(f"Motor does not responded {bytes_expect} byte(s)")

        return res

    
    # Writes the current encoder position of the motor into ROM as the initial position.  Attention:
    # 1. This command needs to restart to take effect.
    # 2. This command will write zero point into ROM of the driver, multiple writing will affect the chip life,which is not recommended for frequent use
    def set_zero_cur_position(self):
        data_length = 0x00        
        header_crc  = (CMD_HEADER + CMD_SET_ZERO + self.id + data_length) % 256
        
        snd = bytearray(pack('<BBBBB', CMD_HEADER, CMD_SET_ZERO, self.id, data_length, header_crc))
        self.serial_port.write(snd)
        #time.sleep(.01)  #give the serial port sometime to receive the data
        res = self.__read_responce(26)      #wait 26 bytes

    def abs_multi_loop_angle_speed(self, angle: float, speed: float):
        assert speed > 0
        
        data_length = 0x0C
        
        header_crc  = (CMD_HEADER + CMD_ABS_MULTI_LOOP_ANGLE_SPEED + self.id + data_length) % 256

        angle       = int(angle * 100)
        speed       = int(speed * 100)    #according documentaton

        data        = pack('<qi', angle, speed)
        data_crc    = sum(data) % 256

        snd = bytearray(pack('<BBBBBqiB', CMD_HEADER, CMD_ABS_MULTI_LOOP_ANGLE_SPEED, self.id, data_length, header_crc, angle, speed, data_crc))
        
        self.serial_port.write(snd)

        #time.sleep(.01)  #give the serial port sometime to receive the data
        res = self.__read_responce(13)      #wait 13 bytes

        return res

    def inc_angle_speed(self, angle: float, speed: float):
        data_length = 0x08
        
        header_crc  = (CMD_HEADER + CMD_INC_ANGLE_SPEED + self.id + data_length) % 256

        angle       = int(angle * 100)
        speed       = int(speed * 100)    #according documentaton

        data        = pack('<ii', angle, speed)
        data_crc    = sum(data) % 256

        snd = bytearray(pack('<BBBBBiiB', CMD_HEADER, CMD_INC_ANGLE_SPEED, self.id, data_length, header_crc, angle, speed, data_crc))
        
        print(" ".join(map(lambda b: format(b, "02x"), snd)))
        self.serial_port.write(snd)

        #time.sleep(.01)  #give the serial port sometime to receive the data

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
        
        return float(angle[0]/100)

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
        
        return angle[0]/100

    #blocks code execution till motor stop (angle do not change)
    #tolerance to detect angle similarity,  request_period to reduce ammout of requests
    def wait_stop(self, tolerance: float, request_period: float, timeout: float):
        prev_value  = 9999999
        start       = time.time()

        assert request_period   >= 0.01
        assert tolerance        >= 0.01
        assert timeout          >= 0.1

        while (time.time() - start < timeout):
            cur_angle   = self.get_multi_loop_angle()
            delta       = abs(prev_value-cur_angle)
            
            if (delta < tolerance): return

            prev_value = cur_angle
            time.sleep(request_period)

        raise TimeoutError("Motor does not stopped in defined time")

# MAIN

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
    print ("error open serial port: " + str(e))
    exit()

if ser.isOpen():
    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output

        
        motor1 = Motor(0x01, ser)
        motor2 = Motor(0x02, ser)
        motor3 = Motor(0x03, ser)

        #save zero positions for axes
        #motor1.set_zero_cur_position()
        #motor2.set_zero_cur_position()
        #motor3.set_zero_cur_position()
        #exit()


        #GO ZERO pos
        motor1.abs_multi_loop_angle_speed(0.0, 30)
        motor2.abs_multi_loop_angle_speed(0.0, 30)
        motor3.abs_multi_loop_angle_speed(0.0, 30)

        
        motor1.wait_stop(0.1, 0.1, 15.0)
        motor2.wait_stop(0.1, 0.1, 15.0)
        motor3.wait_stop(0.1, 0.1, 15.0)

        print("ANGLE1:", motor1.get_multi_loop_angle())
        print("ANGLE2:", motor2.get_multi_loop_angle())
        print("ANGLE3:", motor3.get_multi_loop_angle())
        print("---")
        
        #Moves in cycle
        while True:

            #G0 X6.78 Y

            motor1.abs_multi_loop_angle_speed(0.0, 100)
            motor2.abs_multi_loop_angle_speed(0.0, 100)
            motor3.abs_multi_loop_angle_speed(0.0, 100)

            motor1.wait_stop(0.1, 0.1, 15.0)
            motor2.wait_stop(0.1, 0.1, 15.0)
            motor3.wait_stop(0.1, 0.1, 15.0)

            time.sleep(1)
            
            motor1.abs_multi_loop_angle_speed(20.0, 40)
            motor2.abs_multi_loop_angle_speed(90.0, 40)
            motor3.abs_multi_loop_angle_speed(90.0, 110)
                
            motor1.wait_stop(0.1, 0.1, 15.0)
            motor2.wait_stop(0.1, 0.1, 15.0)
            motor3.wait_stop(0.1, 0.1, 15.0)
            
            time.sleep(1)

            motor1.abs_multi_loop_angle_speed(40.0, 80)
            motor1.wait_stop(0.1, 0.1, 15.0)


            time.sleep(1)

            motor3.abs_multi_loop_angle_speed(180.0, 240)
            motor3.wait_stop(0.1, 0.1, 15.0)

            time.sleep(1)

        ser.close()
    except Exception as e1:
        print ("error communicating...: " + str(e1))

else:
    print("cannot open serial port")