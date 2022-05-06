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
l_color = ColorSensor(Port.S1)
r_color = ColorSensor(Port.S2)

# Constants
WHEEL_DIAMETER = 55.5
AXLE_TRACK = 160

BLACK = 20
WHITE = 70

Kp = 1.2
Kd = 5

# Vars
last_error = 0

robot = DriveBase(l_motor, r_motor, wheel_diameter=WHEEL_DIAMETER, axle_track=AXLE_TRACK)

def PD(Speed):
	""" PD regulator """

	global last_error

	error = l_color.reflection() - r_color.reflection()

	turn_rate = Kp * error + Kd * (error - last_error)

	last_error = error

	robot.drive(-1 * Speed, turn_rate)


def PD_time(Speed, Time):
	""" PD controller that works for provided Time """

	timing = time.time()
	while time.time() - timing < Time:
		PD(Speed)


def PD_line_T(Speed):
	""" Movement on T like line """

	while l_color.reflection() >= BLACK and r_color.reflection() >= BLACK:
		PD(Speed)
	l_motor.brake()
	r_motor.brake()


def Turn_90(Speed, Direction):
	""" Turn to left or right. left - 0, right - 1 """

	l_motor.reset_angle()
	r_motor.reset_angle()

	if Direction == 0:
		l_motor.run_angle(Speed, 300, then=Stop.HOLD, wait=False)
		r_motor.run_angle(-1 * Speed, -300, then=Stop.HOLD, wait=False)
	else:
		l_motor.run_angle(-1 * Speed, -300, then=Stop.HOLD, wait=False)
		r_motor.run_angle(Speed, 300, then=Stop.HOLD, wait=False)	



# <========Start=========>
# 45 deg turn
