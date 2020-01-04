#!/usr/bin/env python3
class BaseDistances:
    """ This class is converts linear and angular velocity to the left and right wheel distance """

    def __init__(self,body_width,update_rate):
        self.body_width = body_width
        self.update_rate = update_rate
        self.left_distance = 0.0
        self.right_distance = 0.0

    def calc_distance(self,vx,vth):
        a_list = [[1,-1],[1,1]]

        #b_list = [[x],[y]]
        x = self.body_width * (vth/self.update_rate)
        y = 2*(vx/self.update_rate)
        
        #inv_a = [[ a , b ] , [ c , d]]
        determinant = ( (a_list[0][0]) * (a_list[1][1]) ) - ( (a_list[0][1]) * (a_list[1][0]) )
        a = (a_list[1][1])/determinant
        b = (-a_list[0][1])/determinant
        c = (-a_list[1][0])/determinant
        d = (a_list[0][0])/determinant

        #solutions = [[(a * x) + (b * y)],[(c * x) + (d * y)]]
        self.left_distance = (a * x) + (b * y)
        self.right_distance = (c * x) + (d * y)
        
    def get_left_distance(self):
        return self.left_distance

    def get_right_distance(self):
        return self.right_distance
