###################
# Desired fuctions
###################
# -set speed
# -set acceleration
# -set listener
# -move number of steps
# -set limits
# -calculate endpoint with switch

import RPi.GPIO as GPIO

class steppermotor():
    """
    This is a class for controlling a steppermotor. It should have fuctions to
    jump a number of steps, set speed and made observable by a listener to get
    its position.
    """
    def __init__(motor_pin):
        self.motor_pin = motor_pin


    def set_speed():
        pass

    def step_num_steps():
        pass
