import math
import numpy as np
   
class Robot:
    def __init__(self, wheel_base, track_width, wheel_radius):
        # w1<--track width--> w2
        # ^                   |
        # |                   |
        # wb                  |
        # |                   |
        # v                   |
        # w3 ---------------- w4
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.wheel_radius = wheel_radius
        wb = self.wheel_base/2.0
        tw = self.track_width/2.0
        r = self.wheel_radius
        T = np.array([[1,-1,-(tw+wb)],
                      [1,1,(tw+wb)],
                      [1,1,-(tw+wb)],
                      [1,-1,(tw+wb)]])
        self.inverse_transform_matrix=(1/r)*T
        self.max_wheel_speed = max(np.matmul(self.inverse_transform_matrix, np.array([[1.0],[1.0],[1.0]])))

def compute_motor_velocities(input,robot,max_value=255):    
    motor_velocities = np.zeros(4)
    if (len(input)<3):
        return motor_velocities
    robot_velocity = np.array([[input[0]],[input[1]],[input[2]]])
    raw_velocities = np.matmul(robot.inverse_transform_matrix,robot_velocity)

    if (max(raw_velocities) == 0.0):
        return motor_velocities
    
    for i in range(len(raw_velocities)):
      motor_velocities[i] = raw_velocities[i]*max_value/robot.max_wheel_speed
  
    return motor_velocities
