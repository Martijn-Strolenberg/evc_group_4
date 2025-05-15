
def PIDController_theta_to_velocity(v_0, theta_ref, theta_hat, prev_e, prev_int, delta_t): #add theta_ref as input
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given).
        theta_ref (:double:) reference heading pose
        theta_hat (:double:) the current estiamted theta.
        prev_e (:double:) tracking error at previous iteration.
        prev_int (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        Left_velocity (:double:) left wheel velocity
        Right_velocity (:double:) right wheel velocity  
        e (:double:) current tracking error (automatically becomes prev_e at next iteration).
        e_int (:double:) current integral error (automatically becomes prev_int at next iteration).
    """
    
   # Tracking error
    e = theta_ref - theta_hat

    # integral of the error
    e_int = prev_int + e*delta_t


    max_error = 2
    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,max_error),-max_error)

    # derivative of the error
    e_der = (e - prev_e)/delta_t

    # controller coefficients
    Kp = 5
    Ki = 0.2
    Kd = 0.1

    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der

    # Split up in left and right velocity
    Left_velocity = v_0 - omega/2
    Right_velocity = v_0 + omega/2
    
    return Left_velocity, Right_velocity, e, e_int

def PIDController_velocity_to_speed(v_ref, v_hat, prev_e, prev_int, delta_t):
    """
    Args:
        v_ref (:double:) reference velocity (can be negative for reverse)
        v_hat (:double:) the current estimated velocity.
        prev_e (:double:) tracking error at previous iteration.
        prev_int (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    Returns:
        speed (:double:) setpoint speed for the Duckiebot [-1, 1]
        e (:double:) current tracking error
        e_int (:double:) current integral error
    """

    # Tracking error
    e = v_ref - v_hat

    # Integral of the error
    e_int = prev_int + e * delta_t
    max_error = 2
    e_int = max(min(e_int, max_error), -max_error)  # Anti-windup

    # Derivative of the error
    e_der = (e - prev_e) / delta_t

    # PID gains
    Kp = 1.5
    Ki = 0.3
    Kd = 0.2

    # PID controller output (velocity command)
    velocity_cmd = Kp * e + Ki * e_int + Kd * e_der

    # Map to speed scale [-1, 1]
    max_vel = 0.5  # max magnitude of velocity in m/s
    speed = velocity_cmd / max_vel
    speed = max(min(speed, 1.0), -1.0)  # Clamp to [-1, 1]

    return speed, e, e_int
