#!/usr/bin/env pybricks-micropython

import time

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# Brick Initialization
ev3 = EV3Brick()

# Motor Initialization
l_motor = Motor(Port.A)
r_motor = Motor(Port.B)

# Sensor initialization
l_color = ColorSensor(Port.S2)
r_color = ColorSensor(Port.S1)

# Constants
Kp = 2
Kd = 1.2

BLACK = 15
WHITE = 75

# Vars
last_error = 0
timer = time.time()



def PD_regulator(Speed):
	""" PD regulator """

    global last_error, timer, Kp, Kd

    error = l_color.reflection() - r_color.reflection()

    l_motor.run(-1 * (Speed + (Kp * error + Kd * (error - last_error))))
    r_motor.run(Speed - (Kp * error + Kd * (error - last_error)))

    if (time.time() - timer) > 0.05:
        timer = time.time()
        last_error = error

def PD_time(Speed, Time):
	""" PD regulator that works for provided Time """

    timing = time.time()
    while time.time() - timing < Time:
        PD_regulator(Speed)


def PD_line_T(Speed):
	""" Movement on T like line """

    while l_color.reflection() >= BLACK and r_color.reflection() >= BLACK:
        PD_regulator(Speed)
    l_motor.brake()
    r_motor.brake()		


def Move(L_speed, R_speed, L_angle, R_angle):

    l_motor.reset_angle(0)
    r_motor.reset_angle(0)
    r_flag = 1
    l_flag = 1

    while abs(l_motor.angle()) < abs(L_angle) or abs(r_motor.angle()) < abs(R_angle):
        if abs(r_motor.angle()) > abs(R_angle):
            r_flag = 0
        if abs(l_motor.angle()) > abs(L_angle):
            l_flag = 0

        l_motor.run(-1 * L_speed * l_flag)
        r_motor.run(R_speed * r_flag)


# <========Start=========>
# 45 deg turn
# Move(400, 400, 270, 40)
# r_motor.stop()
# l_motor.stop()