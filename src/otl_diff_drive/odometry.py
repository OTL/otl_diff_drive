import math

class Robot:

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta

    def update_by_velocity(self, vel_x, vel_theta, duration):
        diff_x = vel_x * duration
        delta_x = diff_x * math.cos(self.theta)
        delta_y = diff_x * math.sin(self.theta)
        delta_theta = vel_theta * duration

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
