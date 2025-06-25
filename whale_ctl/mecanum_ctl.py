import numpy

def generate_motor_cmd(v_x, v_y, v_yaw):


    A = numpy.array([
        [ 1,  1,  1],   # motor_0: Front Left
        [-1,  1, -1],   # motor_1: Front Right
        [-1,  1,  1],   # motor_2: Rear Left
        [ 1,  1, -1]    # motor_3: Rear Right
    ])

    Vin = numpy.array([v_x, v_y, v_yaw])

    motors = A @ Vin

    max_val = max(numpy.max(numpy.abs(motors)), 1.0)
    motors = motors / max_val

    return motors.tolist() 
