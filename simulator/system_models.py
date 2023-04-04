def toy_system_model(prev_state, dt, pedal, steering=0):
    """Get previous state and return the next state"""
    # Motion model of the car's moving
    # The car's previous state: prev_state=[pos_x, pos_y, car_angle, velocity]
    # Two control inputs: pedal (pedal position), and steering (steering angle)
    pos_x = prev_state[0]
    v = prev_state[3]  # m/s

    pos_x_dot = v
    # Control input is the pedal position
    # Assume acceleration = pedal_position;
    v_dot = (-v + pedal*5.0) / 5.0  # Not optimal, just a toy model for the toy experiment


    pos_x += pos_x_dot * dt
    v += v_dot * dt

    # Return the new state
    return [pos_x, 0., 0., v]  # 1D case does not require pos_y and car_angle

def system_model_0(prev_state, dt, v_control, steering=0):
    """Get previous state and return the next state"""
    # Motion model of the car's moving
    # The car's previous state: prev_state=[pos_x, pos_y, car_angle, velocity]
    # Two control inputs: pedal (pedal position), and steering (steering angle)
    pos_x = prev_state[0]
    v = prev_state[3]  # m/s
    pos_x += v * dt  # v=x_dot

    # Assume we can control the velocity directly
    v = v_control
    # Return the new state
    return [pos_x, 0, 0, v]  # 1D case does not require pos_y and car_angle

def system_model_1(prev_state, dt, pedal, steering=0):
    """Get previous state and return the next state"""
    # Motion model of the car's moving
    # The car's previous state: prev_state=[pos_x, pos_y, car_angle, velocity]
    # Two control inputs: pedal (pedal position), and steering (steering angle)
    pos_x = prev_state[0]
    v = prev_state[3]  # m/s

    x_dot = v
    # Control input is the pedal position
    # Assume acceleration = pedal_position * 5;
    v_dot = pedal * 5

    pos_x += x_dot * dt
    v += v_dot * dt 

    # Return the new state
    return [pos_x, 0, 0, v]  # 1D case does not require pos_y and car_angle

def system_model_2(prev_state, dt, pedal, steering=0):
    """Get previous state and return the next state"""
    # Motion model of the car's moving
    # The car's previous state: prev_state=[pos_x, pos_y, car_angle, velocity]
    # Two control inputs: pedal (pedal position), and steering (steering angle)
    pos_x = prev_state[0]
    v = prev_state[3]  # m/s

    x_dot = v
    # Control input is the pedal position
    # Assume acceleration = pedal_position * 5;
    v_dot = pedal * 5

    pos_x += x_dot * dt
    # Assume a natural resistive force "-v/25" from the air friction
    v += v_dot * dt - v / 25 

    # Return the new state
    return [pos_x, 0, 0, v]  # 1D case does not require pos_y and car_angle

def system_model_3(prev_state, dt, pedal, steering=0):
    """Get previous state and return the next state"""
    # Motion model of the car's moving
    # The car's previous state: prev_state=[pos_x, pos_y, car_angle, velocity]
    # Two control inputs: pedal (pedal position), and steering (steering angle)
    pos_x = prev_state[0]
    v = prev_state[3]  # m/s

    x_dot = v
    # control input is the pedal position
    # Assume acceleration = pedal_position * 5;
    # Assume a constant resistive force F=m*0.5
    v_dot = pedal * 5 - 0.5

    pos_x += x_dot * dt
    # Assume a natural resistive force "-v/25" from the air friction
    v += v_dot * dt - v / 25

    # Return the new state
    return [pos_x, 0, 0, v]  # 1D case does not require pos_y and car_angle

