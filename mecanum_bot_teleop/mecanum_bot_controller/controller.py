import math

def compute_yaw_velocities(axes, max_value):
    return []

def compute_holonomic_velocities(axes, max_value):
    return []
    
def compute_motor_velocities(axes, threshold=0.01, max_value=255):
    assert(len(axes) > 2)

    if (abs(axes[3]) > threshold):
        return compute_yaw_velocities(axes, max_value)
    else:
        return compute_holonomic_velocities(axes, max_value)