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
import threading
import serial
from time import sleep

class steppermotor():
    """
    This is a class for controlling a steppermotor. It should have fuctions to
    jump a number of steps, set speed and made observable by a listener to get
    its position.
    """
    def __init__(motor_pin, dir_pin):
        self.motor_pin = motor_pin
        self.position = 0
        self.dir_pin = dir_pin
        self.stop = False
        self.ser = serial.Serial(
               port='/dev/ttyUSB0',
               baudrate = 9600,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout=1
           )

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.motor_pin, self.dir_pin], GPIO.OUT)


    def set_speed(speed):
        """
        This method is called to set a contignous speed on the motor
        """
        if speed >= 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.dir_pit, GPIO.LOW)
        t = threading.Thread(target=__speed__, args=(speed,))
        t.start()

    def stop():
        self.stop = True

    def __speed__(speed):
        """
        Helper method to run a contignous speed
        """
        while True:
            if self.stop == True:
                self.stop = False
                break
            GPIO.output(self.motor_pin, GPIO.HIGH)
            sleep(1/speed)
            GPIO.output(self.motor_pin, GPIO.LOW)
            sleep(1/speed)
            __hasMoved__(speed)

    def __hasMoved__(speed):
        if speed >= 0:
            self.position += 1
        else:
            self.position -= 1
        #self.ser.write(self.position)
        print(self.position)

    def step_num_steps(steps, speed):
        if steps >= 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
            dif = -1
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
            dif = 1
        while steps != 0:
            GPIO.output(self.motor_pin, GPIO.HIGH)
            sleep(1/speed)
            GPIO.output(self.motor_pin, GPIO.LOW)
            sleep(1/speed)
            __hasMoved__(speed)


    def unconnet():
        GPIO.cleanup()
