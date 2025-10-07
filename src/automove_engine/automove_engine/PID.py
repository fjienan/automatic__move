import numpy as np
class PID_Controller():
    def __init__(self,kp=0.0,ki=0.0,kd=0.0,dimention = 2,vx_max = 0.5,vy_max = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dimention = dimention
        self.vx_max = vx_max
        self.vy_max = vy_max
        self.prev_error = np.zeros(dimention)
        self.integral = np.zeros(dimention)

    def compute_vel(self, goal, position, dt):
        error = np.zeros(self.dimention)
        for i in range(self.dimention):
            error[i] = (goal[i] - position[i])
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        com_vel = self.kp * error + self.ki * self.integral + self.kd * derivative
        com_vel[0] = np.clip(com_vel[0], -self.vx_max, self.vx_max)
        com_vel[1] = np.clip(com_vel[1], -self.vy_max, self.vy_max)
        self.prev_error = error
        return com_vel