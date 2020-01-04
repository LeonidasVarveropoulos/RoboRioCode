#!/usr/bin/env python3

from components.motor_output import MotorOutput
motor = MotorOutput(2,0,1,2,6)
print(motor.get_intersection().x)
print(motor.get_intersection().y)
print(motor.get_sum_tan())
print(motor.get_sum_sin())
print(motor.get_sum_cos())
print(motor.get_angle_bisector().m)
print(motor.get_angle_bisector().b)