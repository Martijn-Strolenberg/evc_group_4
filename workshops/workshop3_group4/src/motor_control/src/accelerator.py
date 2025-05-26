# This function reduces slipping by more slowly accelerating the vehicle
def accelerate(current_speed, target_speed, max_acceleration):
    """
    Accelerate the vehicle from current_speed to target_speed with a maximum acceleration limit.

    Parameters:
    current_speed (float): The current speed of the vehicle.        (current estimated speed)
    target_speed (float): The desired target speed of the vehicle.  (speed set point)
    max_acceleration (float): The maximum acceleration limit.       (step size how much we increment)

    Returns:
    float: The new speed of the vehicle after acceleration.
    """
    new_speed = current_speed # Initialize new_speed to current_speed
    
    if current_speed < target_speed:
        new_speed = min(current_speed + max_acceleration, target_speed)
    else:
        new_speed = max(current_speed - max_acceleration, target_speed)
    
    return new_speed