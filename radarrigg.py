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
from time import sleep
import argparse
import socketserver, socket
import time
import pickle

# This varable is a hack to make the rig work. Don't judge
pos = {'rotation': 0, 'height': 0}

class Steppermotor():
    """
    This is a class for controlling a steppermotor. It should have fuctions to
    jump a number of steps, set speed and made observable by a listener to get
    its position.
    """
    def __init__(self, motor_pin, dir_pin, changeval):
        self.motor_pin = motor_pin
        self.position = 0
        self.dir_pin = dir_pin
        self.stop_flag = False
        self.changeval = changeval

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup([self.motor_pin, self.dir_pin], GPIO.OUT)


    def set_speed(self, speed):
        """
        This method is called to set a contignous speed on the motor
        """
        if speed >= 0: #To set the direction of the motor
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
            sleep(1/abs(2*speed))
            GPIO.output(self.motor_pin, GPIO.LOW)
            sleep(1/abs(2*speed))
            self.__hasMoved(speed)

    def __hasMoved(self, speed):
        global pos
        if speed >= 0:
            self.position += 1
            pos[self.changeval] = pos[self.changeval] + 1
        else:
            self.position -= 1
            pos[self.changeval] = pos[self.changeval] - 1


    def step_num_steps(self, steps, speed):
        if steps >= 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
            dif = -1
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
            dif = 1
        while steps != 0:
            GPIO.output(self.motor_pin, GPIO.HIGH)
            sleep(1/abs(2*speed))
            GPIO.output(self.motor_pin, GPIO.LOW)
            sleep(1/abs(2*speed))
            self.__hasMoved(speed)
            steps += dif


    def unconnet(self):
        GPIO.cleanup()

class RadarTCPServer():
    """
    This class will start a socketserver to serve the position of the radarrigg
    to another RPI.
    """
    def __init__(self, host, port):
        """
        initializes the socket server
        """
        self.HOST = host
        self.PORT = port
        self.server = socketserver.TCPServer((self.HOST, self.PORT), RadarTCPHandler)
        self.server.serve_forever()

class RadarTCPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        """
        This method will respond to the tcp request
        """

        close = 0
        while not close:
            _data = self.request.recv(1024)
            if not _data:
                # EOF, client closed, just return
                return
            # kode for haandtering av request

            self.request.send(pickle.dumps(pos))

class RadarTCPClient():
    """
    This class acts as a client to ask the radarrigg for the posittion
    """
    def __init__(self, ip, port):
        """
        initializes the radar client
        """
        self.TCP_IP = ip
        self.TCP_PORT = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.TCP_IP, self.TCP_PORT))


    def getPosition(self):
        self.s.send("get")
        data = self.s.recv(1024)
        return data

    def test(self):
        # send get request
        start = time.time()
        data = pickle.dumps('get')
        self.s.sendall(data)

        # receive position
        data = self.s.recv(4096)
        end = time.time()
        try:
            print("length of data {}".format(data))
            print(pickle.loads(data))
        except EOFError:
            print('something went wrong')

        print("delay: {}".format(end-start))


def tb6612_test(speed):
    GPIO.setmode(GPIO.BOARD)
    chan_list = [7, 11, 13, 15]
    GPIO.setup(chan_list, GPIO.OUT)
    pin_order = [[1, 0, 0, 0],
                [1, 0, 1, 0],
                [0, 0, 1, 0],
                [0, 0, 1, 0],
                [0, 1, 0, 0],
                [0, 1, 0, 1],
                [0, 0, 0, 1],
                [1, 0, 0, 1]
                ]
    i = 0
    while True:
        GPIO.output(chan_list, pin_order[i])
        print('{} pins, {}-value'.format(pin_order[i], i))
        i=(i+1)%8
        sleep(1/abs(speed))

def main():
    # Initialize server
    t = threading.Thread(target=RadarTCPServer, args=('', 2323))
    t.daemon = True
    t.start()

    # Defines the steppmotores to be used
    buttonpin = 11
    table_stepper = Steppermotor(18, 22, 'rotation')
    rail_stepper = Steppermotor(12,16, 'height')

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(32, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


    #First we have to detect the endpoints for the rail
    rail_stepper.set_speed(-200)

    # Wait on switch
    #while GPIO.input(buttonpin) == 1:
    #   pass

    # Stop motor at bottom
    rail_stepper.stop()
    pos['rotation'] = 0
    pos['height'] = 0

    _ = input('tap any button to start scann')

    # How many levels to scan
    levels = 10

    # round is the amount of steps to get one revolution.
    round = 32000

    # height is the number of steps for one length of the slider
    height = 29600

    # Levels is the amount of layers to divide 50 cm stepping in
    levels = 10

    # rotation_speed is the speed which the table rotates
    rotation_speed = 20000

    # rail_speed is the speed which the rail climbs
    rail_speed = 20000

    # Time is the amount of time it takes for one round with the given speed
    rotation_time = round/(rotation_speed)

    # Sets the speed for the scan
    table_stepper.set_speed(rotation_speed)

    for i in range(levels):
        print("Scanning level {}".format(i+1))
        # Here we divide the number of height steps on the levels to know how many levels to scan and the distance between them
        time.sleep(3*rotation_time)
        rail_stepper.step_num_steps(height/(levels-1), rail_speed)



if __name__ == "__main__":
    main()
