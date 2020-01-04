#!/usr/bin/env python3

import wpilib
import wpilib.drive
from networktables import NetworkTables
import time
import math
#https://robotpy.readthedocs.io/en/stable/install/ctre.html#install-ctre
import ctre
from components.moving_average import MovingAverage
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
        self.rotationsPerMeters = 0.00123287028
        self.encoder_left = wpilib.Encoder(0, 1, reverseDirection = True)
        self.encoder_right = wpilib.Encoder(2, 3, reverseDirection = True)
        self.encoder_left.setDistancePerPulse(self.rotationsPerMeters)
        self.encoder_right.setDistancePerPulse(self.rotationsPerMeters)
        
        # Group left and right motors
        self.left_motors = wpilib.SpeedControllerGroup(wpilib.Talon(0), wpilib.Talon(1))
        self.right_motors = wpilib.SpeedControllerGroup(wpilib.Talon(2), wpilib.Talon(3))

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
        self.encoderCountsPerRev = 256.0
        self.wheelDia = 0.1524
        self.wheelCircum = self.wheelDia * math.pi
        # Space between left and right wheels
        self.bodyWidth = 0.6985

        # Gets rate for velocity
        self.updateRate = 50

        # Raw encoder ticks
        self.resetTickLeft = 0.0
        self.resetTickRight = 0.0
        self.currentTickL = self.encoder_left.get() - self.resetTickLeft
        self.currentTickR = self.encoder_right.get() - self.resetTickRight
        self.prevTickL = self.encoder_left.get() - self.resetTickLeft
        self.prevTickR = self.encoder_right.get() - self.resetTickRight

        # Distance by encoders
        self.leftDistance = 0.0
        self.rightDistance = 0.0

        # Velocity feedback
        self.vth = 0.0
        self.vx = 0.0

        # PID Variables
        self.kpAngleNorm = 0.8390000000000006 #0.44900000000000034
        self.kiAngleNorm = 0.07086000000000522 #0.030069999999998945
        self.kdAngleNorm = 0.09070000000000157 #0.039900000000000116

        self.kpAngle = 0.25100000000000017
        self.kiAngle = 0.009109999999999797
        self.kdAngle = 0.026099999999999898

        self.kpLinear = 0.5470000000000004 #2.0029999999998904 #0.8010000000000005
        self.kiLinear = 0.0892199999999981 #0.01964999999999937  #0.013059999999999636
        self.kdLinear = 0.05460000000000054 #0.3119999999999901 #0.22100000000000014

        self.kpGain = .001
        self.kiGain = .0001
        self.kdGain = .0001

        # PID Setpoint
        self.setpointAngle = 0.0
        self.setpointLinear = 0.0

        self.set_gain = 0.01

        # Output
        self.angleControl = 0.0
        self.linearControl = 0.0

        # PID stuff
        self.controlMin = -0.75
        self.controlMax = 0.75

        self.linearErrorSum = 0
        self.angleErrorSum = 0

        self.linear_error_prior = 0
        self.angle_error_prior = 0

        self.time_for_next_loop = time.time()

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
            self.time_for_next_loop = time_now + 1.0/self.updateRate
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
            self.resetTickLeft = self.encoder_left.get()
            self.resetTickRight = self.encoder_right.get()
            self.linearErrorSum = 0
            self.angleErrorSum = 0
            self.linear_error_prior = 0
            self.angle_error_prior = 0
            self.leftDistance = 0.0
            self.rightDistance = 0.0
            self.currentTickL = 0.0
            self.currentTickR = 0.0
            self.prevTickL = 0.0
            self.prevTickR = 0.0
            self.setpointLinear = 0.0
            self.setpointAngle = 0.0

        if button_rt == True:
            if button_a == True:
                if button_rb == True:
                    print("Subtracting Linear kp")
                    self.kpLinear -= self.kpGain
                else:
                    print("Adding Linear kp")
                    self.kpLinear += self.kpGain
                if self.kpLinear < 0:
                    self.kpLinear = 0
            if button_b == True:
                if button_rb == True:
                    print("Subtracting Linear ki")
                    self.kiLinear -= self.kiGain
                else:
                    print("Adding Linear ki")
                    self.kiLinear += self.kiGain
                if self.kiLinear < 0:
                    self.kiLinear = 0
            if button_x == True:
                if button_rb == True:
                    print("Subtracting Linear kd")
                    self.kdLinear -= self.kdGain
                else:
                    print("Adding Linear kd")
                    self.kdLinear += self.kdGain
                if self.kdLinear < 0:
                    self.kdLinear = 0
        
        if button_lt == True:
            if button_a == True:
                if button_rb == True:
                    print("Subtracting Angle kp")
                    self.kpAngleNorm -= self.kpGain
                else:
                    print("Adding Angle kp")
                    self.kpAngleNorm += self.kpGain
                if self.kpAngleNorm < 0:
                    self.kpAngleNorm = 0
            if button_b == True:
                if button_rb == True:
                    print("Subtracting Angle ki")
                    self.kiAngleNorm -= self.kiGain
                else:
                    print("Adding Angle ki")
                    self.kiAngleNorm += self.kiGain
                if self.kiAngleNorm < 0:
                    self.kiAngleNorm = 0
            if button_x == True:
                if button_rb == True:
                    print("Subtracting Angle kd")
                    self.kdAngleNorm -= self.kdGain
                else:
                    print("Adding Angle kd")
                    self.kdAngleNorm += self.kdGain
                if self.kdAngleNorm < 0:
                    self.kdAngleNorm = 0
                    
        # Set setpoint values
        if True:
            if forward > 0.1:
                print("Adding Linear Setpoint")
                self.setpointLinear= 0.16
            elif forward < -0.1:
                self.setpointLinear= -0.16
                print("Sub Linear Setpoint")
            if turn > 0.1:
                self.setpointAngle= .1
                print("Add Angle Setpoint")
            elif turn < -0.1:
                self.setpointAngle= -.1
                print("Sub Angle Setpoint")
            if button_y == True:
                self.setpointAngle = 0.0
                self.setpointLinear = 0.0
                print("Reset Setpoint")
        else:
            self.setpointLinear = self.table.getNumber("lin_cmd_vel",self.setpointLinear)
            self.setpointAngle = self.table.getNumber("ang_cmd_vel",self.setpointAngle)

        # Real bot simulation
        if self.sim == False:
            self.currentTickL = self.encoder_left.get() - self.resetTickLeft
            self.currentTickR = self.encoder_right.get() - self.resetTickRight

            self.leftDistance = ((self.currentTickL - self.prevTickL)/self.encoderCountsPerRev) * self.wheelCircum
            self.rightDistance = ((self.currentTickR - self.prevTickR)/self.encoderCountsPerRev)* self.wheelCircum

            self.vth = ((self.leftDistance-self.rightDistance)/ self.bodyWidth) * self.updateRate
            self.vx = ((self.rightDistance + self.leftDistance)/2.0) * self.updateRate

        # Sim bot simulation (encoder values are wack)
        elif self.sim == True:
            self.currentTickL = self.encoder_left.get() - self.resetTickLeft
            self.currentTickR = self.encoder_right.get() - self.resetTickRight

            self.leftDistance = (self.currentTickL - self.prevTickL)/10000.0
            self.rightDistance = (self.currentTickR - self.prevTickR)/10000.0

            self.vth = ((self.leftDistance-self.rightDistance)/ self.bodyWidth) * self.updateRate
            self.vx = ((self.rightDistance + self.leftDistance)/2.0) * self.updateRate

        # Average velocities over a rolling window to account for too fast update rate
        self.lin_moving_average.add_velocity(self.vx)
        self.angle_moving_average.add_velocity(self.vth)

        new_vx = self.lin_moving_average.get_average()
        new_vth = self.angle_moving_average.get_average()

        # LINEAR PID
        # Negate values for backward robot
        linearError = (-1 * self.setpointLinear) - new_vx
        self.linearErrorSum += (linearError / self.updateRate)

        if self.kiLinear != 0:
            linear_error_sum_max = self.controlMax / self.kiLinear
            linear_error_sum_min = self.controlMin / self.kiLinear
            if self.linearErrorSum > linear_error_sum_max:
                self.linearErrorSum = linear_error_sum_max
            elif self.linearErrorSum < linear_error_sum_min:
                self.linearErrorSum = linear_error_sum_min

        self.linearControl = (self.kpLinear*linearError) + (self.kiLinear*self.linearErrorSum) + (self.kdLinear*((linearError-self.linear_error_prior)* self.updateRate))

        # Angle control acts differently when there is a linear control
        if self.setpointLinear != 0:
            kpAngle = self.kpAngleNorm
            kiAngle = self.kiAngleNorm
            kdAngle = self.kdAngleNorm
            if self.linearControl > 0:
                self.linearControl += 0.3
            else:
                self.linearControl -= 0.3
        else:
            self.linearControl = 0
            kpAngle = self.kpAngle
            kiAngle = self.kiAngle
            kdAngle = self.kdAngle


        if self.linearControl > self.controlMax:
            self.linearControl = self.controlMax
        elif self.linearControl < self.controlMin:
            self.linearControl = self.controlMin

        self.linear_error_prior = linearError

        # ANGULAR PID
        # Negate values for backward robot
        angleError = (-1 * self.setpointAngle) - new_vth
        self.angleErrorSum += angleError / self.updateRate

        if kiAngle != 0:
            angle_error_sum_max = self.controlMax / kiAngle
            angle_error_sum_min = self.controlMin / kiAngle
            if self.angleErrorSum > angle_error_sum_max:
                self.angleErrorSum = angle_error_sum_max
            elif self.angleErrorSum < angle_error_sum_min:
                self.angleErrorSum = angle_error_sum_min

        self.angleControl = (kpAngle*angleError) + (kiAngle*self.angleErrorSum) + (kdAngle*((angleError-self.angle_error_prior)* self.updateRate))

        # Angle control acts differently when turning in place (only turning)
        if self.setpointLinear == 0:
            if self.angleControl > 0:
                self.angleControl += 0.3
            else:
                self.angleControl -= 0.3
            
        if self.setpointAngle == 0:
            self.angleControl = 0

        if self.angleControl > self.controlMax:
            self.angleControl = self.controlMax
        elif self.angleControl < self.controlMin:
            self.angleControl = self.controlMin


        self.angle_error_prior = angleError

        # NetworkTables
        self.table.putNumber("EncVelLinear", new_vx)
        self.table.putNumber("EncVelAngle",new_vth)
        self.table.putNumber("SetpointVelLinear",-1 * self.setpointLinear)
        self.table.putNumber("SetpointVelAngle",-1 * self.setpointAngle)
        self.table.putNumber("Linearkp", self.kpLinear)
        self.table.putNumber("Linearki", self.kiLinear)
        self.table.putNumber("Linearkd", self.kdLinear)
        self.table.putNumber("Anglekp", self.kpAngleNorm)
        self.table.putNumber("Angleki", self.kiAngleNorm)
        self.table.putNumber("Anglekd", self.kdAngleNorm)
        self.table.putNumber("LinearControl", self.linearControl)
        self.table.putNumber("AngleControl", self.angleControl)
        self.table.putNumber("Test encoder", self.encoder_left.get() - self.resetTickLeft)


        self.drive.arcadeDrive(self.linearControl, self.angleControl)

        self.prevTickL = self.currentTickL
        self.prevTickR = self.currentTickR

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