#!/usr/bin/env python3
class MovingAverage:
    def __init__(self,window_length):
        self.window_length = window_length
        self.average_vel = 0
        self.window = [0]* self.window_length
    def add_velocity(self,velocity):
        del self.window[len(self.window) - 1] 
        self.window.insert(0,velocity)
    def get_average(self):
        self.average_vel = sum(self.window) / len(self.window)
        return self.average_vel