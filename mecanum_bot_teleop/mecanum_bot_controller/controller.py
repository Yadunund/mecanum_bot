import math
import numpy as np
   
class Robot:
    def __init__(self, wheel_base, track_width, wheel_radius):
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.wheel_radius = wheel_radius


def compute_motor_velocities(axes, robot, threshold=0.01, max_value=255):
    assert(len(axes) > 2)
    return np.zeros(4)