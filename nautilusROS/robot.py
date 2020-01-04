#!/usr/bin/env python3

# INSTALL NUMPY: python -m pip install numpy
import wpilib
import wpilib.drive
from networktables import NetworkTables
import time
import math
#https://robotpy.readthedocs.io/en/stable/install/ctre.html#install-ctre
import ctre
from components.moving_average import MovingAverage
from components.base_distances import BaseDistances
from components.pid import PID
import logging

class MyRobot(wpilib.TimedRobot):
    """Main robot class"""

    def robotInit(self):
        """Robot-wide initialization code should go here"""

        # Initialize network tables for communication with the dashboard in the driver station
        # Problem: Could be that the version on roboRio does not match or IP of roboRio is not right
        # set up loging
        NetworkTables.initialize()
        self.table = NetworkTables.getTable('Nautilus')

        # Setup pneumatics solenoids
        self.shifter = wpilib.Solenoid(0)
        self.center_wheel = wpilib.Solenoid(1)
        self.gear_pusher = wpilib.Solenoid(2)
        self.gear_side_flap = wpilib.Solenoid(4)
        self.gear_top_flap = wpilib.Solenoid(5)

        # Create LEDs (PCM1 solenoids)
        self.led_front_red = wpilib.Solenoid(1, 0)
        self.led_front_blue = wpilib.Solenoid(1, 1)
        self.led_front_green = wpilib.Solenoid(1, 2)
        self.led_back_blue = wpilib.Solenoid(1, 6)
        self.led_back_green = wpilib.Solenoid(1, 5)

        # Create the encoders
        self.rotations_per_meter = 0.00123287028
        self.encoder_left = wpilib.Encoder(0, 1, reverseDirection = True)
        self.encoder_right = wpilib.Encoder(2, 3, reverseDirection = True)
        self.encoder_left.setDistancePerPulse(self.rotations_per_meter)
        self.encoder_right.setDistancePerPulse(self.rotations_per_meter)
        
        self.left_motor1 = wpilib.Talon(0)
        self.left_motor2 = wpilib.Talon(1)

        self.right_motor1 = wpilib.Talon(2)
        self.right_motor2 = wpilib.Talon(3)

        # Group left and right motors
        self.left_motors = wpilib.SpeedControllerGroup(self.left_motor1, self.left_motor2)
        self.right_motors = wpilib.SpeedControllerGroup(self.right_motor1, self.right_motor2)

        # Create center drive motor
        self.centerdrive = wpilib.Talon(4)

        # Create drivetrain
        self.drive = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

        # Create PDP
        self.pdp = wpilib.PowerDistributionPanel()

        # Create (CAN) motors for shooting
        self.shooter1 = ctre.WPI_TalonSRX(0)
        self.shooter2 = ctre.WPI_TalonSRX(1)
        self.feeder = wpilib.Talon(6)

        # Create joystick
        self.stick1 = wpilib.Joystick(0)

    def autonomousInit(self):
        """Called only at the beginning of autonomous mode"""
        # Documented by encoders
        self.encoder_counts_per_rev = 256.0
        self.wheel_dia = 0.1524
        self.wheel_circum = self.wheel_dia * math.pi
        # Space between left and right wheels
        self.body_width = 0.6985

        # Gets rate for velocity
        self.update_rate = 50

        # Raw encoder ticks
        self.reset_tick_left = 0.0
        self.reset_tick_right = 0.0
        self.current_left_tick = self.encoder_left.get() - self.reset_tick_left
        self.current_right_tick = self.encoder_right.get() - self.reset_tick_right
        self.prev_left_tick = self.encoder_left.get() - self.reset_tick_left
        self.prev_right_tick = self.encoder_right.get() - self.reset_tick_right

        # Distance by encoders
        self.left_distance = 0.0
        self.right_distance = 0.0

        # Velocity feedback
        self.vth = 0.0
        self.vx = 0.0

        # PID Variables
        self.kp_right = 0.0046000000000000025
        self.ki_right = 0.0
        self.kd_right = 0.0

        self.kp_left = 0.0046000000000000025
        self.ki_left = 0.0
        self.kd_left = 0.0

        self.kp_gain = .0001
        self.ki_gain = .0000001
        self.kd_gain = .0001

        # PID Setpoint
        self.setpoint_angle = 0.0
        self.setpoint_linear = 0.0

        self.set_gain = 0.01

        # Output
        self.right_control = 0.0
        self.left_control = 0.0

        # Left And Right Distances
        self.feedback_distance = BaseDistances(self.body_width,self.update_rate)
        self.setpoint_distance = BaseDistances(self.body_width, self.update_rate)

        # PID stuff
        self.control_min = -0.75
        self.control_max = 0.75

        self.left_pid = PID(self.update_rate,self.control_max,self.control_min)
        self.right_pid = PID(self.update_rate,self.control_max,self.control_min)

        # Time
        self.time_for_next_loop = time.time()

        # Rolling Average
        self.window_length = 20
        self.lin_moving_average = MovingAverage(self.window_length)
        self.angle_moving_average = MovingAverage(self.window_length)

        # Change if doing sim
        self.sim = True

    def autonomousPeriodic(self):
        """Called every 20ms in autonomous mode"""

        # Slows down update rate (by changing the update rate var)
        """
        time_now = time.time()
        if time_now < self.time_for_next_loop:
            return
        else:
            self.time_for_next_loop = time_now + 1.0/self.update_rate
        """
        
        button_a = self.stick1.getRawButton(2)
        button_b = self.stick1.getRawButton(3)
        button_x = self.stick1.getRawButton(1)
        button_rt = self.stick1.getRawButton(8)
        button_lt = self.stick1.getRawButton(7)
        button_y = self.stick1.getRawButton(4)
        button_lb = self.stick1.getRawButton(5)
        button_rb = self.stick1.getRawButton(6)
        forward = -self.stick1.getY()
        turn = self.stick1.getZ()

        # Preform a reset
        if button_y == True and button_lb == True and button_rb == True:
            self.reset_tick_left = self.encoder_left.get()
            self.reset_tick_right = self.encoder_right.get()

            self.left_pid.reset()
            self.right_pid.reset()

            self.left_distance = 0.0
            self.right_distance = 0.0
            
            self.current_left_tick = 0.0
            self.current_right_tick = 0.0
            self.prev_left_tick = 0.0
            self.prev_right_tick = 0.0

            feed_left = 0.0
            feed_right = 0.0
            set_left = 0.0
            set_right = 0.0

            self.setpoint_linear = 0.0
            self.setpoint_angle = 0.0

        # Left PID tuning
        if button_rt == True:
            if button_a == True:
                if button_rb == True:
                    print("Subtracting Left kp")
                    self.kp_left -= self.kp_gain
                else:
                    print("Adding Left kp")
                    self.kp_left += self.kp_gain
                if self.kp_left < 0:
                    self.kp_left = 0
            if button_b == True:
                if button_rb == True:
                    print("Subtracting Left ki")
                    self.ki_left -= self.ki_gain
                else:
                    print("Adding Left ki")
                    self.ki_left += self.ki_gain
                if self.ki_left < 0:
                    self.ki_left = 0
            if button_x == True:
                if button_rb == True:
                    print("Subtracting Left kd")
                    self.kd_left -= self.kd_gain
                else:
                    print("Adding Left kd")
                    self.kd_left += self.kd_gain
                if self.kd_left < 0:
                    self.kd_left = 0
        
        # Right PID tuning
        if button_lt == True:
            if button_a == True:
                if button_rb == True:
                    print("Subtracting Right kp")
                    self.kp_right -= self.kp_gain
                else:
                    print("Adding Right kp")
                    self.kp_right += self.kp_gain
                if self.kp_right < 0:
                    self.kp_right = 0
            if button_b == True:
                if button_rb == True:
                    print("Subtracting Right ki")
                    self.ki_right -= self.ki_gain
                else:
                    print("Adding Right ki")
                    self.ki_right += self.ki_gain
                if self.ki_right < 0:
                    self.ki_right = 0
            if button_x == True:
                if button_rb == True:
                    print("Subtracting Right kd")
                    self.kd_right -= self.kd_gain
                else:
                    print("Adding Right kd")
                    self.kd_right += self.kd_gain
                if self.kd_right < 0:
                    self.kd_right = 0
                    
        # Set setpoint values if in sim
        if self.sim:
            if forward > 0.1:
                print("Adding Linear Setpoint")
                self.setpoint_linear+=self.set_gain
            elif forward < -0.1:
                self.setpoint_linear-=self.set_gain
                print("Sub Linear Setpoint")
            if turn > 0.1:
                self.setpoint_angle+= self.set_gain
                print("Add Angle Setpoint")
            elif turn < -0.1:
                self.setpoint_angle-= self.set_gain
                print("Sub Angle Setpoint")
            if button_y == True:
                self.setpoint_angle = 0.0
                self.setpoint_linear = 0.0
                print("Reset Setpoint")
        else:
            self.setpoint_linear = self.table.getNumber("lin_cmd_vel",self.setpoint_linear)
            self.setpoint_angle = self.table.getNumber("ang_cmd_vel",self.setpoint_angle)

        # Real bot simulation- Calc vth and vx
        if self.sim:
            self.current_left_tick = self.encoder_left.get() - self.reset_tick_left
            self.current_right_tick = self.encoder_right.get() - self.reset_tick_right

            self.left_distance = ((self.current_left_tick - self.prev_left_tick)/self.encoder_counts_per_rev) * self.wheel_circum
            self.right_distance = ((self.current_right_tick - self.prev_right_tick)/self.encoder_counts_per_rev)* self.wheel_circum

            self.vth = ((self.left_distance-self.right_distance)/ self.body_width) * self.update_rate
            self.vx = ((self.right_distance + self.left_distance)/2.0) * self.update_rate

        # Sim bot simulation (encoder values are wack) - Calc vth and vx
        else:
            self.current_left_tick = self.encoder_left.get() - self.reset_tick_left
            self.current_right_tick = self.encoder_right.get() - self.reset_tick_right

            self.left_distance = (self.current_left_tick - self.prev_left_tick)/10000.0
            self.right_distance = (self.current_right_tick - self.prev_right_tick)/10000.0

            self.vth = ((self.left_distance-self.right_distance)/ self.body_width) * self.update_rate
            self.vx = ((self.right_distance + self.left_distance)/2.0) * self.update_rate

        # Average velocities over a rolling window to account for too fast update rate
        self.lin_moving_average.add_velocity(self.vx)
        self.angle_moving_average.add_velocity(self.vth)
        new_vx = self.lin_moving_average.get_average() 
        new_vth = self.angle_moving_average.get_average()

        # Get velocities for each side of the robot (For both feedback and setpoint)
        self.feedback_distance.calc_distance(new_vx,new_vth)
        self.setpoint_distance.calc_distance(self.setpoint_linear, self.setpoint_angle)
        feed_left = self.feedback_distance.get_left_distance() * self.update_rate
        feed_right = self.feedback_distance.get_right_distance() * self.update_rate

        set_left = self.setpoint_distance.get_left_distance() * self.update_rate
        set_right = self.setpoint_distance.get_right_distance() * self.update_rate

        # Left Side PID
        self.left_pid.updatePID(set_left,feed_left,self.kp_left,self.ki_left,self.kd_left)
        self.left_control = self.left_pid.get_control()
        # Right Side PID
        self.right_pid.updatePID(set_right,feed_right,self.kp_right,self.ki_right,self.kd_right)
        self.right_control = self.right_pid.get_control()
        # Drive Bot
        self.left_motor1.set(self.left_control)
        self.left_motor2.set(self.left_control)

        self.right_motor1.set(-self.right_control)
        self.right_motor2.set(-self.right_control)


        # NetworkTables
        self.table.putNumber("EncVelLinear", new_vx)
        self.table.putNumber("EncVelAngle",new_vth)
        self.table.putNumber("SetpointVelLinear", self.setpoint_linear)
        self.table.putNumber("SetpointVelAngle",self.setpoint_angle)
        self.table.putNumber("LeftControl", self.left_control)
        self.table.putNumber("RightControl", self.right_control)
        self.table.putNumber("Test encoder", self.encoder_left.get() - self.reset_tick_left)

        self.table.putNumber("Leftkp", self.kp_left)
        self.table.putNumber("Leftki", self.ki_left)
        self.table.putNumber("Leftkd", self.kd_left)
        self.table.putNumber("LeftFeedback", feed_left)
        self.table.putNumber("LeftSetpoint", set_left)


        self.table.putNumber("Rightkp", self.kp_right)
        self.table.putNumber("Rightki", self.ki_right)
        self.table.putNumber("Rightkd", self.kd_right)
        self.table.putNumber("RightFeedback", feed_right)
        self.table.putNumber("RightSetpoint", set_right)



        self.prev_left_tick = self.current_left_tick
        self.prev_right_tick = self.current_right_tick

    def disabledInit(self):
        """Called only at the beginning of disabled mode"""
        pass

    def disabledPeriodic(self):
        """Called every 20ms in disabled mode"""
        pass

    def teleopInit(self):
        """Called only at the beginning of teleoperated mode"""
        self.power = 1.0

        self.shooter1_power = 0.8
        self.shooter2_power = 0.8
        self.feeder_power = 0.5
        self.side_power = 0.7

        self.shooter1.set(0)
        self.shooter2.set(0)
        self.feeder.set(0)
    def teleopPeriodic(self):
        """Called every 20ms in teleoperated mode"""

        # Read joystick buttons
        button_a = self.stick1.getRawButton(2)
        button_b = self.stick1.getRawButton(3)
        button_x = self.stick1.getRawButton(1)
        button_rt = self.stick1.getRawButton(8)
        button_lt = self.stick1.getRawButton(7)
        button_lb = self.stick1.getRawButton(5)
        button_rb = self.stick1.getRawButton(6)

        forward = -self.stick1.getY()
        turn = self.stick1.getZ()
        side = self.stick1.getX()


        if side > 0.2 or side < -0.2:
            self.center_wheel.set(True)
            self.centerdrive.set(side)
        else:
            self.center_wheel.set(False)
            self.centerdrive.set(side)

        
        if button_rt == True:
            self.shooter1.set(-self.shooter1_power)
            self.shooter2.set(-self.shooter2_power)
            if button_lt == True:
                self.feeder.set(-self.feeder_power)
            else:
                self.feeder.set(0)
        else:
            self.shooter1.set(0)
            self.shooter2.set(0)
            self.feeder.set(0)

        if button_rb == True:
            self.shifter.set(True)   
        else:
            self.shifter.set(False)
            
        if button_lb == True:
            self.drive.arcadeDrive(forward*0.5, turn*0.5)
        else:
            self.drive.arcadeDrive(forward*self.power, turn*self.power)
if __name__ == "__main__":
    wpilib.run(MyRobot)