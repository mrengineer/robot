from multiprocessing.dummy import Array
import re
from tokenize import String
from xmlrpc.client import Boolean

from parso import split_lines
from py import process
from traitlets import Bool, Float, Integer

# extract letter-digit pairs
g_pattern = re.compile('([A-Z])([-+]?[0-9.]+)')

# white spaces and comments start with ';' and in '()'
clean_pattern = re.compile('\s+|\(.*?\)|;.*')

class GInterpreter():
    GCode = list()

    # Machine settings
    F: Float = 0.0          #Linear feed
    S: Float = 0.0          #Tool rotation speed
    T: Integer = 0          #Tool number

    Abs_X: Float = 0.0
    Abs_Y: Float = 0.0
    Abs_Z: Float = 0.0

    Abs_coord: Boolean = True   #Absolute coordinate system


    def __init__(self, GCode: String) -> None:
        self.GCode = split_lines(GCode)


    def start(self):
        print("START")
        for code_line in self.GCode:
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
            self.process(params)

    def process(self, params):
        if "G" in params:
            G = int(params["G"])
            if (G==0):
                print("Fast jump to point")
            if (G==1):
                print("Linear move to point with feed F")
            if (G==2):
                print("Clockwise circular interpolation")
            if (G==3):
                print("Counter-Clockwise circular interpolation")
            if (G==4):
                print("Delay")
            if (G==10):
                print("Absolute coordinates system shift")
            if (G==90):
                print("Absolute coordinates system has been selected")
                self.Abs_coord = True
            if (G==90):
                print("Releative coordinates system has been selected")
                self.Abs_coord = False

        elif "M" in params:
            M = int(params["M"])
            if (M==0):
                print("Pause and wait till start press")
            if (M==1):
                print("Pause and wait till start press if step-by-step mode is enabled")
        else:
            print ("ERR")
    