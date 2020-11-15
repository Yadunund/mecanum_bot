import math
import numpy as np
   
class Robot:
    def __init__(self, wheel_base, track_width, wheel_radius):
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.wheel_radius = wheel_radius


def compute_motor_velocities(axes, robot, threshold=0.01, max_value=255):
    assert(len(axes) > 2)
    
    motor_velocities = np.zeros(4)

    joy_angle = np.arctan(axes[1]/axes[0])
    joy_pos = np.sqrt(axes[0]**2 + axes[1]**2)
    x_vel = joy_pos*np.cos(joy_angle)
    y_vel = joy_pos*np.sin(joy_angle)
    if (abs(axes[2] > threshold):
        w_vel = axes[2]
    else:
        w_vel = 0.0
    robot_velocity = np.array([x_vel,y_vel,w_vel])

    wb = robot.wheel_base/2.0
    tw = robot.track_width/2.0
    r = robot.wheel_radius
    
    T = np.array([[1,-1,-(tw+wb)],
                  [1,1,(tw+wb)],
                  [1,1,-(tw+wb)],
                  [1,-1,(tw+wb)]])
    T=(1/r)*T

    motor_velocities = np.matmul(T,robot_velocity)
    # Normalize motor velocities value and multiply by max_value
    norm = np.linalg.norm(motor_velocities)
    motor_velocities = max_value* motor_velocities / norm
    return motor_velocities
