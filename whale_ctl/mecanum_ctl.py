import numpy

def generate_motor_cmd(v_x, v_y, v_yaw):


    # A = numpy.array([
    #     [ 1.0,  1.0,  1.0],   # motor_0: Front Right
    #     [ 1.0, -1.0,  1.0],   # motor_1: Front Left
    #     [-1.0, -1.0,  1.0],   # motor_2: Rear Left
    #     [-1.0,  1.0,  1.0]    # motor_3: Rear Right
    # ])
    A = numpy.array([
        [ 1.0,  1.0,  1.0],   # motor_0: Front Right
        [-1.0,  1.0,  1.0],   # motor_1: Front Left
        [-1.0, -1.0,  1.0],   # motor_2: Rear Left
        [ 1.0, -1.0,  1.0]    # motor_3: Rear Right
    ])

    Vin = numpy.array([v_x, v_y, v_yaw])

    motors = A @ Vin

    max_val = max(numpy.max(numpy.abs(motors)), 1.0)
    if max_val > 100:
        motors = motors * 100 / max_val

    return motors.tolist() 
