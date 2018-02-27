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

class Steppermotor():
    """
    This is a class for controlling a steppermotor. It should have fuctions to
    jump a number of steps, set speed and made observable by a listener to get
    its position.
    """
    def __init__(self, motor_pin, dir_pin, listener=None):
        self.motor_pin = motor_pin
        self.position = 0
        self.dir_pin = dir_pin
        self.stop_flag = False
        # self.ser = serial.Serial(
        #        port='/dev/ttyUSB0',
        #        baudrate = 9600,
        #        parity=serial.PARITY_NONE,
        #        stopbits=serial.STOPBITS_ONE,
        #        bytesize=serial.EIGHTBITS,
        #        timeout=1
        #    )

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.motor_pin, self.dir_pin], GPIO.OUT)


    def set_speed(self, speed):
        """
        This method is called to set a contignous speed on the motor
        """
        if speed >= 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
        t = threading.Thread(target=self.__speed__, args=(speed,))
        t.start()

    def stop(self):
        self.stop_flag = True

    def __speed__(self, speed):
        """
        Helper method to run a contignous speed
        """
        while True:
            if self.stop_flag == True:
                self.stop_flag = False
                break
            GPIO.output(self.motor_pin, GPIO.HIGH)
            sleep(1/abs(speed))
            GPIO.output(self.motor_pin, GPIO.LOW)
            sleep(1/abs(speed))
            self.__hasMoved__(speed)

    def __hasMoved__(self, speed):
        if speed >= 0:
            self.position += 1
        else:
            self.position -= 1
        if self.listener!=None:
            self.listener.fire
        #self.ser.write(self.position)
        #print(self.position)

    def step_num_steps(self, steps, speed):
        if steps >= 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
            dif = -1
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
            dif = 1
        while steps != 0:
            GPIO.output(self.motor_pin, GPIO.HIGH)
            sleep(1/abs(speed))
            GPIO.output(self.motor_pin, GPIO.LOW)
            sleep(1/abs(speed))
            self.__hasMoved__(speed)
            steps += dif


    def unconnet(self):
        GPIO.cleanup()


def tb6612_test():
    GPIO.setmode(GPIO.BOARD)
    chan_list = [7, 11, 13, 15]
    GPIO.setup(chan_list, GPIO.OUT)
    pin_order = [[GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.LOW],
                [GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.LOW],
                [GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.LOW],
                [GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.HIGH]]
    i = 0
    while True:
        GPIO.output(chan_list, pin_order[i])
        i=(i+1)%4
        time.sleep(0.0005)
