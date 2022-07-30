import logging
import re, os
import numpy as np
from parso import split_lines

log = logging.getLogger("robot")


import robot



# extract letter-digit pairs
g_pattern = re.compile('([A-Z])([-+]?[0-9.]+)')

# white spaces and comments start with ';' and in '()'
clean_pattern = re.compile('\s+|\(.*?\)|;.*')

class GInterpreter():
    robot: robot.Robot
    _GCode = list()
    _run_ix = 0

    # Machine settings
    F: float = 0.0          #Linear feed
    S: float = 0.0          #Tool rotation speed
    T: int   = 0          #Tool number

    Abs_X: float = 0.0
    Abs_Y: float = 0.0
    Abs_Z: float = 0.0

    Abs_coord: bool = True   #Absolute coordinate system


    def __init__(self, port_name: str) -> None:
        assert len(port_name) >= 3, "Incorrect portname"
        log.warning("Start")
        self.robot = robot.Robot(port_name)


    @property
    def code(self):
        return self._GCode

    @code.setter
    def code(self, value: str):
        self._GCode = split_lines(value)

    def run(self):
        cur_coords = self.robot.get_coords()
        print(cur_coords)

        for code_line in self._GCode:
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
            self.step(params)
            print(f"ABS machine XYZ ({self.Abs_X}, {self.Abs_Y}, {self.Abs_Z}), S {self.S}, F {self.F}, T {self.T}\n")


    def step(self, params):            
        if "T" in params: self.T =   int(params["T"])
        if "S" in params: self.T = float(params["S"])
        if "F" in params: self.T = float(params["F"])

        if "G" in params:
            G = int(params["G"])
            if (G==0):
                print("Fast jump to point")
                if self.Abs_coord: 
                    if "X" in params: self.Abs_X = float(params["X"])
                    if "Y" in params: self.Abs_Y = float(params["Y"])
                    if "Z" in params: self.Abs_Z = float(params["Z"])
                else:
                    if "X" in params: self.Abs_X = self.Abs_X + float(params["X"])
                    if "Y" in params: self.Abs_Y = self.Abs_Y + float(params["Y"])
                    if "Z" in params: self.Abs_Z = self.Abs_Z + float(params["Z"])                    
            elif (G==1):
                print("Linear move to point with feed F")

                #NB Feed is not implemented yet!
                
                if self.Abs_coord: 
                    if "X" in params: self.Abs_X = float(params["X"])
                    if "Y" in params: self.Abs_Y = float(params["Y"])
                    if "Z" in params: self.Abs_Z = float(params["Z"])
                else:
                    if "X" in params: self.Abs_X = self.Abs_X + float(params["X"])
                    if "Y" in params: self.Abs_Y = self.Abs_Y + float(params["Y"])
                    if "Z" in params: self.Abs_Z = self.Abs_Z + float(params["Z"])                 
            elif (G==2):
                print("Clockwise circular interpolation")
            elif (G==3):
                print("Counter-Clockwise circular interpolation")
            elif (G==4):
                print("Delay")
            elif (G==10):
                print("Absolute coordinates system shift")
            elif (G==30):
                print("Z go to tool change")                
            elif (G==90):
                print("Absolute coordinates system has been selected")
                self.Abs_coord = True
            elif (G==91):
                print("Releative coordinates system has been selected")
                self.Abs_coord = False
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



def main():
    GInt = GInterpreter("ttyUSB0")

    m1 = GInt.robot.add_motor(0x01, 0.1, "X", False, [112.2, -45, 0],  "YZ")
    m2 = GInt.robot.add_motor(0x02, 0.1, "Y1", False, [126, 53, 0], "XZ")
    m3 = GInt.robot.add_motor(0x03, 0.1, "Y2", False, [105.7, -34, 0], "XZ")
           
    
    GInt.code = """%
    ( Данная управляющая программа для станков с ЧПУ создана )
    ( Файл создан:  2022-07-17  01:27:15  )

    N010 G00 Z0.5 F70
    N020 G00 X5 Y15 F70
    N030 G01 Z-1 F50
    N040 G01 X5 Y35 F50
    N050 G01 X35 Y15
    N060 G01 X5 Y15
    N070 G00 Z0.5 F70
    N080 G00 X0 Y0 F70
    N090 M30
    %"""

    GInt.run()

if __name__ == '__main__':
    os.system('clear')
    main()