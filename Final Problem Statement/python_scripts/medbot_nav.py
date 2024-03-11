motor_speed = 0

# FUNCTION TO COMPUTE VELOCITY IN Y DIRECTION
def goToY(curr,target):
    print("go to pos")
    global motor_speed
    error_y = target - curr
    # Kp = 0.35
    Kp = 0.15
    motor_speed = abs(error_y) * Kp
    return motor_speed

# FUNCTION TO COMPUTE VELOCITY IN Y DIRECTION
def goToX(curr,target):
    print("go to pos")
    global motor_speed
    error_x = target - curr
    # Kp = 0.35
    Kp = 0.15
    motor_speed = abs(error_x) * Kp
    return motor_speed

# FUNCTION TO COMPUTE ANGULAR VELOCITY
def goToOrientation(curr,target):
    global motor_speed
    Kp = 1
    error = target - curr
    motor_speed = Kp * abs(error)

    if target > curr:
        print("turning ACW")
        return motor_speed
    if target < curr:
        print("turning CW")
        return -motor_speed