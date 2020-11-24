import math
import numpy as np
   
class Robot:
    def __init__(self, wheel_base, track_width, wheel_radius):
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.wheel_radius = wheel_radius


def compute_motor_velocities(input,robot,max_value=255):    
    motor_velocities = np.zeros(4)
    robot_velocity = np.array([[input[0]],[input[1]],[input[2]]])

    wb = robot.wheel_base/2.0
    tw = robot.track_width/2.0
    r = robot.wheel_radius
    
    T = np.array([[1,-1,-(tw+wb)],
                  [1,1,(tw+wb)],
                  [1,1,-(tw+wb)],
                  [1,-1,(tw+wb)]])
    T=(1/r)*T

    raw_velocities = np.matmul(T,robot_velocity)
    print(f"Raw motor velocities: {raw_velocities}")
    # Normalize motor velocities value and multiply by max_value
    if (max(raw_velocities) == 0.0):
        return np.zeros(4)
    motor_velocities = np.zeros(4) 
    for i in range(len(raw_velocities)):
      motor_velocities[i] = raw_velocities[i]*max_value/abs(max(raw_velocities))
  
    return motor_velocities
