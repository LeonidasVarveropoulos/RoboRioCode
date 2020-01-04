class PID:
    def __init__(self, update_rate,control_max,control_min):
        self.control = 0.0
        self.error_sum = 0.0
        self.update_rate = update_rate
        self.control_max = control_max 
        self.control_min = control_min
        self.prev_error = 0.0
    
    def reset(self):
        self.control = 0.0
        self.error_sum = 0.0
        self.prev_error = 0.0

    def updatePID(self,setpoint,feedback,kp,ki,kd):
        # Calculate error
        error = (setpoint) - feedback
        
        # Calculate sum of error over time
        self.error_sum += (error/self.update_rate) # Not sure if this should be here

        if ki != 0:
            error_sum_max = self.control_max / ki
            error_sum_min = self.control_min / ki
            if self.error_sum > error_sum_max:
                self.error_sum = error_sum_max
            elif self.error_sum < error_sum_min:
                self.error_sum = error_sum_min
        
        self.control = (kp * error) + (ki *self.error_sum) + (kd * ((error - self.prev_error) * self.update_rate))

        if self.control > self.control_max:
            self.control = self.control_max
        elif self.control < self.control_min:
            self.control = self.control_min
        self.prev_error = error

    def get_control(self):
        return self.control