from ast import Try
import logging
import re, os, json
from json import JSONEncoder
import asyncio
from turtle import delay
from construct import Int
import numpy as np
from parso import split_lines
import time, datetime

from traitlets import Integer
from robot import Robot, Motor, time_of_function, Serializer


# API for web UI
import asyncio
import websockets  #https://websockets.readthedocs.io/en/5.0/intro.html
from websockets import server

wsockets = list()

logging.basicConfig(format='%(asctime)s %(name)s - %(levelname)s: %(message)s')

# extract letter-digit pairs
g_pattern = re.compile('([A-Z])([-+]?[0-9.]+)')

# white spaces and comments start with ';' and in '()'
clean_pattern = re.compile('\s+|\(.*?\)|;.*')


class GInterpreter(Serializer):
    robot: Robot
    _GCode = list()
    _active_line: Integer = 0 #Line for/in execution


    def __init__(self, port_name: str) -> None:
        assert len(port_name) >= 3, "Incorrect portname"        
        self.robot = Robot(port_name)
        self._active_line = 0

    @property
    def code(self):
        return self._GCode

    @property
    def active_line(self):
        return self._active_line

    @property
    def lines_count(self):
        return self.code.count


    @code.setter
    def code(self, value: str):
        self._active_line = 0
        self._GCode = split_lines(value)

    
    def step(self):
        start_coords = self.robot.get_coords()

        X, Y, Z = start_coords[0], start_coords[1], start_coords[2]

        S = 0.0
        T = 0
        F = 0.0
        Abs = True
        
        #Use previous angles for correct angles calculation. At first step they are all zeros
        
        guess = self.robot.get_single_loop_angles()

        self._lines_count = self._GCode.count
        self._active_line = 0

        for code_line in self._GCode:
            self._active_line = self._active_line + 1

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
            
                       
            #self._active_line = code_line

            X, Y, Z, T, S, F, Abs = self.do_step(params, X, Y, Z, T, S, F, Abs)

            target = np.array([X, Y, Z])

            #use previous angles for previous point as guess to achieve shortest change in angles            

            angles, tol, dirs = self.robot.sim_coords_to_angles(target, guess)
            speeds = [70] * self.robot.motors_count
            
            print("Angles", angles, "speeds", speeds, "dirs", dirs, "target", target, "\n\n")

            self.robot.goto_abs_single_loop_angles_speeds(angles, speeds, dirs)
            guess = angles

            #print("Res angles [angles], [coord tolerances]:", res)
            
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


async def handler(websocket, path):
    wsockets.append(websocket)

    try:
        while(True):
            name = await websocket.recv()
            print(f"RX {name}")
    except:     #on socket close
        wsockets.remove(websocket)



async def main():
    print("Starting main")
    GInt: GInterpreter = GInterpreter("ttyUSB0")

    m1: Motor = GInt.robot.add_motor(0x01, 0.1, "X",  False, [112.2, -45, 0],  "YZ")
    m2: Motor = GInt.robot.add_motor(0x02, 0.1, "Y1", False, [126, 53, 0], "XZ")
    m3: Motor = GInt.robot.add_motor(0x03, 0.1, "Y2", False, [105.7, -34, 0], "XZ")
    
    GInt.robot.goto_zero()
   

    GInt.code = """%
    ( File created:  2022-07-17  01:27:15  )

    N010 G00 X343 F70
    N020 G00 X340.2 Y-26 F70
    N030 G01 Z-1 F50
    N040 G01 Z1 F50
    N050 G01 X35 Y15
    N060 G01 X5 Y15
    N070 G00 Z0.5 F70
    N080 G00 X0 Y0 F70
    N090 M30
    %"""

    #TODO: Перевести на abs_single_loop_angle_speed и выбирать направление вращения исходя из кратчайшего угла по пути

    await asyncio.sleep(1)
    
    g = GInt.step()

    for s in wsockets:
        if s.open: await s.send(GInt.json_serialize())

    res = next(g)
    for s in wsockets:
        if s.open: await s.send(GInt.json_serialize())

    res = next(g)
    for s in wsockets:
        if s.open: await s.send(GInt.json_serialize())
    res = next(g)
    for s in wsockets:
        if s.open: await s.send(GInt.json_serialize())
    res = next(g)
    for s in wsockets:
        if s.open: await s.send(GInt.json_serialize())

    while(1):
        await asyncio.sleep(1)
        for s in wsockets:
            if s.open: await s.send(GInt.json_serialize())


if __name__ == '__main__':
    os.system('clear')    
    
    print("Starting server")
    serv = websockets.serve(handler, 'localhost', 8000)
    
    asyncio.get_event_loop().run_until_complete(serv)
    asyncio.get_event_loop().run_until_complete(main())
    asyncio.get_event_loop().run_forever()
    