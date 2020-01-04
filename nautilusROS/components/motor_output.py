#!/usr/bin/env python3
import math
from fractions import Fraction

class MotorOutput:
    """ This class is converts pid output to a motor output. Takes in two tangent lines and a point on the circle between them"""

    def __init__(self, line1_m, line1_b, line2_m, line2_b, point_on_circle_y):
        # Line 1 (Is Lower)
        self.line1 = Line(line1_m, line1_b)

        # Line 2 (Is Higher)
        self.line2 = Line(line2_m, line2_b)

        # Adjusts how smooth the transition is
        self.point_on_circle = Point((point_on_circle_y - self.line1.b) / (self.line1.m) , point_on_circle_y)

        # Start Calc
        self.tan_fraction = Fraction(self.get_sum_tan()).limit_denominator()
        self.sin_fraction = Fraction(self.get_sum_sin()).limit_denominator()
        self.cos_fraction = Fraction(self.get_sum_cos()).limit_denominator()

    def get_intersection(self):
        """ Follows the (mx + b) base equation. Returns a point (x,y) in a list """
        x = (self.line2.b - self.line1.b) / (self.line1.m - self.line2.m)

        # Using line 1
        y = self.line1.m * x + self.line1.b
        return Point(x,y)
    
    def get_sum_tan(self):
        return (self.line1.m + self.line2.m) / (1 - self.line1.m * self.line2.m)

    def get_sum_sin(self):
        return (-self.tan_fraction.numerator) / (math.sqrt(math.pow(self.tan_fraction.numerator, 2) + math.pow(self.tan_fraction.denominator, 2)))

    def get_sum_cos(self):
        return (-self.tan_fraction.denominator) / (math.sqrt(math.pow(self.tan_fraction.numerator, 2) + math.pow(self.tan_fraction.denominator, 2)))

    def get_angle_bisector(self):
        if self.get_sum_sin != 0:
            slope = (1 - self.get_sum_cos()) / self.get_sum_sin()
        else:
            print("Is undefined")
        
        b_term = (self.get_intersection().y) - (slope * self.get_intersection().x)

        return Line(slope, b_term)
    
    def get_circle(self):
        pass
  
class Point:
    """ Point (x,y) """
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Line:
    """ Follows the (mx + b) base equation"""
    def __init__(self, m, b):
        self.m = m
        self.b = b

class Circle:
    """ Gives a cicle equation """
    def __init__(self, h, k, radius):
        self.h = h
        self.k = k
        self.radius = radius

