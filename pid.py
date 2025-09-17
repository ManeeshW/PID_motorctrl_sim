class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        deriv = (error - self.prev_error) / dt if dt > 0 else 0
        u = self.kp * error + self.ki * self.integral + self.kd * deriv
        self.prev_error = error
        return u