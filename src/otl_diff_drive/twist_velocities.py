
#
#             wheel_offset         wheel_offset
# Left Wheel <============> center <===========> Right Wheel
#


class Converter:
    def __init__(self, wheel_radius, wheel_offset, left_direction=1, right_direction=1):
        self.wheel_radius = wheel_radius
        self.wheel_offset = wheel_offset
        assert wheel_radius > 0, "wheel_radius (%f) must be positive"%wheel_radius
        assert wheel_offset > 0, "wheel_offset (%f) must be positive"%wheel_offset
        self.left_direction = left_direction
        self.right_direction = right_direction

    def convert(self, vel_x, vel_theta):
        ts = vel_x
        tw = vel_theta * self.wheel_offset
        vel_l = (ts - tw) / self.wheel_radius
        vel_r = (ts + tw) / self.wheel_radius
        return (vel_l * self.left_direction, vel_r * self.right_direction)

def calc_filtered_velocity(target_vel, current_vel, limit_accel, duration):
    accel = (target_vel - current_vel) / duration
    if accel > limit_accel:
        filtered = current_vel + (limit_accel * duration)
    elif accel < -limit_accel:
        filtered = current_vel - (limit_accel * duration)
    else:
        filtered = target_vel
    return filtered

def limit_abs(target, limit):
    if target > limit:
        filtered = limit
    elif target < -limit:
        filtered = -limit
    else:
        filtered = target
    return filtered

class AccelFilter:
    def __init__(self, accel_x, accel_theta):
        self._accel_x = accel_x
        self._accel_theta = accel_theta
        self._current_vel_x = 0.0
        self._current_vel_theta = 0.0

    def set_velocity(self, vel_x, vel_theta):
        self._current_vel_x = vel_x
        self._current_vel_theta = vel_theta

    def filter(self, vel_x, vel_theta, time_from_last):
        assert time_from_last > 0.0
        filtered_x = calc_filtered_velocity(vel_x, self._current_vel_x, self._accel_x, time_from_last)
        filtered_theta = calc_filtered_velocity(vel_theta, self._current_vel_theta, self._accel_theta, time_from_last)
        self.set_velocity(filtered_x, filtered_theta)
        return (filtered_x, filtered_theta)

class VelocityFilter:
    def __init__(self, vel_x, vel_theta):
        self._vel_x = vel_x
        self._vel_theta = vel_theta

    def filter(self, vel_x, vel_theta):
        return (limit_abs(vel_x, self._vel_x), limit_abs(vel_theta, self._vel_theta))
