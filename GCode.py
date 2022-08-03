import logging
import re, os
from trace import Trace
import numpy as np
from parso import split_lines
import robot


logging.basicConfig(format='%(asctime)s %(name)s - %(levelname)s: %(message)s')

# extract letter-digit pairs
g_pattern = re.compile('([A-Z])([-+]?[0-9.]+)')

# white spaces and comments start with ';' and in '()'
clean_pattern = re.compile('\s+|\(.*?\)|;.*')

class GInterpreter():
    robot: robot.Robot
    _GCode = list()
    __is_run = False


    def __init__(self, port_name: str) -> None:
        assert len(port_name) >= 3, "Incorrect portname"        
        self.robot = robot.Robot(port_name)


    @property
    def code(self):
        return self._GCode

    @code.setter
    def code(self, value: str):
        self._GCode = split_lines(value)

    def step(self):
        start_coords = self.robot.get_coords()

        X = start_coords[0]
        Y = start_coords[1]
        Z = start_coords[2]

        S = 0.0
        T = 0
        F = 0.0
        Abs = True
        
        #Use previous angles for correct angles calculation. At first step they are all zeros
        
        #guess = np.zeros([self.robot.motors_count])
        guess = self.robot.get_multi_loop_angles()

        for code_line in self._GCode:
            code_line = code_line.strip()
            line = code_line.upper()
            line = re.sub(clean_pattern, '', line)
            

            if len(line) == 0:
                print(code_line)
                continue

            if line[0] == '%':
                print(code_line)
                continue

            m = g_pattern.findall(line)

            if not m:
                raise Exception('G-code is not found')

            if len(''.join(["%s%s" % i for i in m])) != len(line):
                raise Exception('Extra characters in line')

            params = dict(m)

            if len(params) != len(m):
                raise Exception('Duplicated G-code entries')

            if 'G' in params and 'M' in params:
                raise Exception('G and M command found')
            
            print(code_line, "->", params)
            
            

            X, Y, Z, T, S, F, Abs = self.do_step(params, X, Y, Z, T, S, F, Abs)

            target = np.array([X, Y, Z])

            #use previous angles for previous point as guess to achieve shortest change in angles
            
            print("GUESS", guess)
            print("TARGET", target, "\n")

            res = self.robot.sim_coords_to_angles(target, guess)
            guess = res[0]

            print("Res angles [angles], [coord tolerances]\n", res)
            yield X, Y, Z
            


    def do_step(self, params, X: float, Y: float, Z: float, T: int, S: float, F: float, Abs: bool):
        if "T" in params: T =   int(params["T"])
        if "S" in params: S = float(params["S"])
        if "F" in params: F = float(params["F"])

        if "G" in params:
            G = int(params["G"])
            if (G==0):
                print("Fast jump to point")
                if Abs:
                    if "X" in params: X = float(params["X"])
                    if "Y" in params: Y = float(params["Y"])
                    if "Z" in params: Z = float(params["Z"])
                else:
                    if "X" in params: X = X + float(params["X"])
                    if "Y" in params: Y = Y + float(params["Y"])
                    if "Z" in params: Z = Z + float(params["Z"])
            elif (G==1):
                print("Linear move to point with feed F")
 
                logging.warning("Feed is not implemented for G01 yet!")
                if Abs:
                    if "X" in params: X = float(params["X"])
                    if "Y" in params: Y = float(params["Y"])
                    if "Z" in params: Z = float(params["Z"])
                else:
                    if "X" in params: X = X + float(params["X"])
                    if "Y" in params: Y = Y + float(params["Y"])
                    if "Z" in params: Z = Z + float(params["Z"])                                
            elif (G==2):
                logging.warning("G02 is not implemented")
                print("Clockwise circular interpolation")
            elif (G==3):
                logging.warning("G03 is not implemented")
                print("Counter-Clockwise circular interpolation")
            elif (G==4):
                logging.warning("G04 is not implemented")
                print("Delay")
            elif (G==10):
                logging.warning("G05 is not implemented")
                print("Absolute coordinates system shift")
            elif (G==30):
                print("Z go to tool change")                
            elif (G==90):
                print("Absolute coordinates system has been selected")
                self.Abs_coord_sys = True
            elif (G==91):
                print("Releative coordinates system has been selected")
                self.Abs_coord_sys = False
            else:
                raise Exception(f"Unknown or unsupported G code G{G}")

        elif "M" in params:
            M = int(params["M"])
            if (M==0):
                print("Pause and wait till start press")
            elif (M==1):
                print("Pause and wait till start press if step-by-step mode is enabled")
            elif (M==2):
                print("End of program")
            elif (M==3):
                print("Spindle rotation CW")
            elif (M==4):
                print("Spindle rotation CCW")
            elif (M==5):
                print("Spindle rotation STOP")
            elif (M==6):
                print("Tool change")
            elif (M==7):
                print("Enable additional cooliant")
            elif (M==8):
                print("Enable main cooliant")
            elif (M==9):
                print("Diasble cooliant")
            elif (M==30):
                print("End of program")
            else:
                raise Exception(f"Unknown or unsupported M code M{M}")
        else:
            raise Exception(f"There is no G or M code in line or some another error in line interpretation")

        return X, Y, Z, T, S, F, Abs


def main():
    GInt = GInterpreter("ttyUSB0")

    m1 = GInt.robot.add_motor(0x01, 0.1, "X", False, [112.2, -45, 0],  "YZ")
    m2 = GInt.robot.add_motor(0x02, 0.1, "Y1", False, [126, 53, 0], "XZ")
    m3 = GInt.robot.add_motor(0x03, 0.1, "Y2", False, [105.7, -34, 0], "XZ")
           
    
    GInt.code = """%
    ( Данная управляющая программа для станков с ЧПУ создана )
    ( Файл создан:  2022-07-17  01:27:15  )

    N010 G00 Z0.5 F70
    N020 G00 X340 Y-26 F70
    N030 G01 Z-1 F50
    N040 G01 X5 Y35 F50
    N050 G01 X35 Y15
    N060 G01 X5 Y15
    N070 G00 Z0.5 F70
    N080 G00 X0 Y0 F70
    N090 M30
    %"""

    g = GInt.step()
    res = next(g)
    print("res", res)

    print("---------------------------------------\n\n\n")

    res = next(g)
    print("res", res)

    print("---------------------------------------\n\n\n")

    res = next(g)
    print("res", res)

if __name__ == '__main__':
    os.system('clear')
    main()