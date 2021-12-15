#-*- coding:UTF-8 -*-
#This code uses its own defined pulse function to generate pwm waveform
import RPi.GPIO as GPIO
from time import sleep, time
import cv2
from movement import robotState

CAMERA_FOV = 45

class CameraController:
    
    def __init__(self, yawPin = 11, pitchPin = 9):
        self.yawPin = yawPin
        self.pitchPin = pitchPin
        self.currentYaw = 0
        self.currentPitch = 0

        #Set the GPIO port to BCM encoding mode.
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(yawPin, GPIO.OUT)
        GPIO.setup(pitchPin, GPIO.OUT)

        self.pitch_to_angle(0)
        self.yaw_to_angle(0)
        print("CameraController Initialized")


    #Define a pulse function to generate the PWM value in the analog mode. 
    #The base pulse is 20ms, and the high level of the pulse is controlled at 0-180 degrees in 0.5-2.5ms.
    def __servo_pulse(self, pin, angle):
        pulsewidth = (angle * 11) + 500
        GPIO.output(pin, GPIO.HIGH)
        sleep(pulsewidth/1000000.0)
        GPIO.output(pin, GPIO.LOW)
        sleep(20.0/1000-pulsewidth/1000000.0)

    def __pin_to_angle(self, pin, current, dest):
        if current == dest:
            return
        elif current < dest:
            for pos in range(current, dest):
                self.__servo_pulse(pin, pos)
        else:
            for pos in reversed(range(dest, current)):
                self.__servo_pulse(pin, pos)

    def yaw_to_angle(self, angle):
        print("Yaw to %s" % angle)        
        self.__pin_to_angle(self.yawPin, self.currentYaw, angle)
        self.currentYaw = angle

    def pitch_to_angle(self, angle):
        print("Pitch to %s" % angle)
        self.__pin_to_angle(self.pitchPin, self.currentPitch, angle)
        self.currentPitch = angle

CAT_CASCADE = cv2.CascadeClassifier('/usr/local/share/opencv4/haarcascades/haarcascade_frontalcatface.xml')

def detect_cat(cap, count):
    ret, img = cap.read()

    if img is None:
        print("Error reading camera: read returned None frame.")
        return None

    cv2.imwrite("caps/cap%s.png" % count, img)
    
    rects = CAT_CASCADE.detectMultiScale(img, scaleFactor=1.3,
	minNeighbors=10, minSize=(75, 75))

    if len(rects) == 0:
        return None
    else:
        print("Found cat!")
        indexOfBiggest = 0
        areaOfBiggest = 0
        for (i, (x, y , w, h)) in enumerate(rects):
            area = w * h
            if area > areaOfBiggest:
                indexOfBiggest = i
                areaOfBiggest = area
        
        # Get the horizontal position of the cat face in the image
        catRect = rects[indexOfBiggest]
        imgX = catRect[0] + catRect[2] / 2
        imgWidth = len(img[0])
        perc = float(imgX) / float(imgWidth)
        yaw = int(-(CAMERA_FOV / 2.0) + CAMERA_FOV * perc)
        return yaw


CAMERA_NUMBER = 1

def scan_for_cat():
    yawToCat = None
    controller = CameraController()

    # Initialize the movement and state controller
    robot = robotState()

    startTime = time()

    # Initialize the video stream
    cap = cv2.VideoCapture(CAMERA_NUMBER)

    fps = cap.get(cv2.CAP_PROP_FPS)

    yawIncrement = int(CAMERA_FOV / 1.5)
    yaw = 90
    capCount = 0

    # Set what properties of the camera it supports, to try to get clean images
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280);  # Width
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024); # Height
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 0);      # Brightness 0
    cap.set(cv2.CAP_PROP_CONTRAST, 10);       # Contrast 10
    cap.set(cv2.CAP_PROP_SATURATION, 50);     # Saturation 50
    cap.set(cv2.CAP_PROP_HUE, 25);            # Hue 25

    while True: # This is broken by keyboard interrupt
        try:
            while yawToCat is None:

                # Move to the target angle
                controller.yaw_to_angle(yaw)
                
                # Measure time since the last known capture. This is an estimate of how long
                # it took to move to the target yaw, run detection, and send a signal to the motion stack
                endTime = time()

                # Calculate how long to wait to sync the camera capture with the framerate.
                # This is assuming the first frame was taken very close to when the VideoCapture
                # was instantiated.
                wait = (1.0 / fps) - (endTime - startTime)
                if wait < 0:
                    wait = (1.0 / fps) + wait
                wait = wait % (1.0 / fps)
                sleep(wait)

                # Take multiple captures at the same position to account for the G1
                # camera's blur. 10 works well but this takes time so a lower number should
                # be used if it works.
                for i in range(10):
                    yawToCat = detect_cat(cap, capCount)
                    if not (yawToCat is None):
                        break

                # Record the time very close to when the last capture was taken, to try to\
                # synchronize captures with the camera framerate. This reduces blur because
                # the G1's camera interpolates between frames when polled instead of waiting for
                # the next frame.
                startTime = time()

                # Save a rotating log of captures for debugging.
                capCount = capCount + 1
                if capCount >= 40:
                    print("Full camera cycle")
                    capCount = 0

                if yawToCat is None:
                    # Increment the yaw and reverse directions if it goes past 0 or 180.
                    # This is only done if no cat was found because otherwise the target yaw
                    # is needed to calculate motion rotation.
                    yaw = yaw + yawIncrement
                    if yaw >= 180:
                        sleep(0.009)
                        yawIncrement = yawIncrement * -1
                    elif yaw <= 0:
                        sleep(0.009)
                        yawIncrement = yawIncrement * -1
            
            # A cat was found, so calculate the yaw to face it
            print("Yaw to cat: %s" % yawToCat)
            yaw = yaw + yawToCat
            # # Yaw scan in the same direction the cat was seen I guess
            # if yawIncrement < 0 != yawToCat < 0:
            #     yawIncrement = yawIncrement * -1
            
            motionYawHeading = 90 + yaw

            robot.turn_degrees(yaw)

            yaw = 90

            yawToCat = None # Reset so the inner while loop re-enters.
            
        except KeyboardInterrupt:
            break

    cap.release()
    robot.cleanup()

sleep(2)

scan_for_cat()

GPIO.cleanup()
