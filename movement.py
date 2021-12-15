#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time


class robotState:
    # List of commmands. syntax is [command, [left_speed, right_speed, time]]
    ADVANCE_TIME = 5
    FAR_DISTANCE = 50
    MID_DISTANCE = 30

    #Definition of  pin
    IN1 = 20
    IN2 = 21
    IN3 = 19
    IN4 = 26
    ENA = 16
    ENB = 13

    #Definition of button
    key = 8

    #Definition of ultrasonic module pin
    EchoPin = 0
    TrigPin = 1

    #Definition of RGB module pins
    LED_R = 22
    LED_G = 27
    LED_B = 24

    #Definition of servo pin
    ServoPin = 23

    # Sets up GPIO values
    def __init__(self):

        #Set the GPIO port to BCM encoding mode.
        GPIO.setmode(GPIO.BCM)

        #Ignore warning information
        GPIO.setwarnings(False)

        #Motor pins are initialized into output mode
        #Key pin is initialized into input mode
        #Ultrasonic pin initialization
        GPIO.setup(self.ENA, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.ENB, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN4, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.key, GPIO.IN)
        GPIO.setup(self.EchoPin, GPIO.IN)
        GPIO.setup(self.TrigPin, GPIO.OUT)
        GPIO.setup(self.LED_R, GPIO.OUT)
        GPIO.setup(self.LED_G, GPIO.OUT)
        GPIO.setup(self.LED_B, GPIO.OUT)
        GPIO.setup(self.ServoPin, GPIO.OUT)
        #Set the PWM pin and frequency is 2000hz
        self.pwm_ENA = GPIO.PWM(self.ENA, 2000)
        self.pwm_ENB = GPIO.PWM(self.ENB, 2000)
        self.pwm_ENA.start(0)
        self.pwm_ENB.start(0)

        self.pwm_servo = GPIO.PWM(self.ServoPin, 50)
        self.pwm_servo.start(0)

        self.infra_servo_deg = 0
        self.Commands = []

    #Button detection
    def key_scan(self):
        while GPIO.input(self.key):
            pass
        while not GPIO.input(self.key):
            time.sleep(0.01)
            if not GPIO.input(self.key):
                time.sleep(0.01)
                while not GPIO.input(self.key):
                    pass

    # runs through the list of commands backwards
    def workBackwards(self):
        print("working backwards")
        if (len(self.Commands) < 1):
            return
        for command in self.Commands[::-1]:
            print(command)
            leftspeed = command[1][0]
            rightspeed = command[1][1]
            times = command[1][2]
            command = command[0]
            command(leftspeed, rightspeed, times, True)
            self.brake()
            time.sleep(0.1)
        self.Commands[:] = []

    # Go straight forward for times seconds with the wheels moving at leftspeed and rightspeed respectively (those are frequently the values)
    # recover value says whether to add the function to our list of commands for backtracking
    def run(self, leftspeed, rightspeed, times, recover=False):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(times)
        if not recover:
            self.Commands.append([self.back, [leftspeed, rightspeed, times]])

    # Go straight backward, same rules as `run` function apply
    def back(self, leftspeed, rightspeed, times, recover=False):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(times)
        if not recover:
            self.Commands.append([self.run, [leftspeed, rightspeed, times]])

    # Turn left, left and right speeds will vary in this function
    # recover value does the same as previous functions
    def left(self, leftspeed, rightspeed, times, recover=False):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(times)
        if not recover:
            self.Commands.append([self.right, [leftspeed, rightspeed, times]])

    # Turn right, left and right speeds will vary in this function
    # recover value does the same as previous functions
    def right(self, leftspeed, rightspeed, times, recover=False):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(times)
        if not recover:
            self.Commands.append([self.left, [leftspeed, rightspeed, times]])

    # Turn left in place, left and right speeds may vary, but won't necessarily
    # recover value does the same as previous functions
    def spin_left(self, leftspeed, rightspeed, times, recover=False):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(times)
        if not recover:
            self.Commands.append(
                [self.spin_right, [leftspeed, rightspeed, times]])

    # Turn right in place, left and right speeds may vary, but won't necessarily
    # recover value does the same as previous functions
    def spin_right(self, leftspeed, rightspeed, times, recover=False):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_ENA.ChangeDutyCycle(leftspeed)
        self.pwm_ENB.ChangeDutyCycle(rightspeed)
        time.sleep(times)
        if not recover:
            self.Commands.append(
                [self.spin_left, [leftspeed, rightspeed, times]])

    # Stops all wheel movements
    def brake(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)

    # The ultrasonic sensor activates and returns the distance between it and the nearest object directly ahead of it
    def Distance(self):
        GPIO.output(self.TrigPin, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(self.TrigPin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(self.TrigPin, GPIO.LOW)

        t3 = time.time()

        while not GPIO.input(self.EchoPin):
            t4 = time.time()
            if (t4 - t3) > 0.03:
                return -1

        t1 = time.time()

        while GPIO.input(self.EchoPin):
            t5 = time.time()
            if (t5 - t1) > 0.03:
                return -1

        t2 = time.time()
        time.sleep(0.01)
        #    print "distance is %d " % (((t2 - t1)* 340 / 2) * 100)
        return ((t2 - t1) * 340 / 2) * 100

    #The ultrasonic sensor rotates to the specified angle
    def servo_appointed_detection(self, pos):
        for _ in range(18):
            self.pwm_servo.ChangeDutyCycle(2.5 + 10 * pos / 180)
        self.infra_servo_deg = pos

    # Back up a little bit
    # Fire the ultrasonic sensor straight ahead, 90 degrees to the right, then 90 degrees to the left, recording the distances
    # if the distances are all small, spin to the right
    # otherwise, if the left side is further away, spin to the left
    # otherwise, if the right side is further away, spin to the right
    # The lights above the ultrasonic sensor also flash colors depending on which way the robot turns
            # magenta for right and blue for left
    def change_direction(self):
        #red
        GPIO.output(self.LED_R, GPIO.HIGH)
        GPIO.output(self.LED_G, GPIO.LOW)
        GPIO.output(self.LED_B, GPIO.LOW)
        self.back(20, 20, 0.08)
        self.brake()

        self.servo_appointed_detection(0)
        time.sleep(0.8)
        rightdistance = Distance_test(self)

        self.servo_appointed_detection(180)
        time.sleep(0.8)
        leftdistance = Distance_test(self)

        self.servo_appointed_detection(90)
        time.sleep(0.8)
        frontdistance = Distance_test(self)

        if leftdistance < 30 and rightdistance < 30 and frontdistance < 30:
            #Magenta
            GPIO.output(self.LED_R, GPIO.HIGH)
            GPIO.output(self.LED_G, GPIO.LOW)
            GPIO.output(self.LED_B, GPIO.HIGH)
            self.spin_right(85, 85, 0.58)
        elif leftdistance >= rightdistance:
            #Blue
            GPIO.output(self.LED_R, GPIO.LOW)
            GPIO.output(self.LED_G, GPIO.LOW)
            GPIO.output(self.LED_B, GPIO.HIGH)
            self.spin_left(85, 85, 0.28)
        elif leftdistance <= rightdistance:
            #Magenta
            GPIO.output(self.LED_R, GPIO.HIGH)
            GPIO.output(self.LED_G, GPIO.LOW)
            GPIO.output(self.LED_B, GPIO.HIGH)
            self.spin_right(85, 85, 0.28)

    # The robot turns the number of degrees the the ultrasonic sensor is turned
    # Then it moves the sensor back to zero degrees 
    def turn_degrees(self, degrees = None):
        deg = self.infra_servo_deg if degrees is None else degrees
        deg = deg % 360
        print(deg)
        turn_time = abs(deg) / 90.0
        if (deg > 180 or (deg < 0 and deg > -180)):
            self.spin_right(50, 50, turn_time)
        else:
            self.spin_left(50, 50, turn_time)
        self.servo_appointed_detection(90)
        self.brake()

    def cleanup(self):
        self.pwm_ENA.stop()
        self.pwm_ENB.stop()
        self.pwm_servo.stop()


# Check how far away the nearest object is from a robot
def Distance_test(robot):
    num = 0
    ultrasonic = []
    while num < 5:
        distance = robot.Distance()
        while int(distance) == -1:
            distance = robot.Distance()
            print("Tdistance is %f" % (distance))
        while (int(distance) >= 500 or int(distance) == 0):
            distance = robot.Distance()
            print("Edistance is %f" % (distance))
        ultrasonic.append(distance)
        num = num + 1
        time.sleep(0.01)
    print(ultrasonic)
    distance = (ultrasonic[1] + ultrasonic[2] + ultrasonic[3]) / 3
    print("distance is %f" % (distance))
    return distance

# function for getting data from the image processor stack
def fromImageProcessor(deg, robot):
    robot.servo_appointed_detection(deg)

# Basic "runner" function
# it checks how far away an object is
# if the object is over a certain, far distance away it goes forward a good amount
# if it is under the far distance, but over a medium distance, it goes forward at a slower pace
# if the nearest object is very close it, look around and move accordingly
def goDistance(robot):
    distance = Distance_test(robot)
    if distance > robot.FAR_DISTANCE:
        robot.run(50, 50, 0.28)
    elif robot.MID_DISTANCE <= distance <= robot.FAR_DISTANCE:
        robot.run(robot.MID_DISTANCE + 5, robot.MID_DISTANCE + 5, 0.28)
    elif distance < robot.MID_DISTANCE:
        robot.change_direction()
    print(distance)


# time.sleep(2)
# try:
#     robot = robotState()

#     robot.key_scan()
#     # servo_appointed_detection(90)
#     # while True:
#     for j in range(2):
#         init_time = time.time()
#         while (time.time() - init_time < robot.ADVANCE_TIME):
#             # for i in range(1):
#             degs = 45
#             robot.servo_appointed_detection(degs)
#             robot.turn_degrees()
#             # goDistance(robot)
#         time.sleep(2)
#         robot.workBackwards()

# except KeyboardInterrupt:
#     pass
