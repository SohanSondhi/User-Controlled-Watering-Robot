# Set Up and Import Packages
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray
import RPi.GPIO as gpio
import time
""""
gpio.setwarnings(False)

gpio.setmode(gpio.BOARD)
gpio.setup(11, gpio.OUT)
gpio.output(11, True)
time.sleep(5)

gpio.cleanup()

"""
#Set GPIO mode to GPIO.BCM
#GPIO.setmode(GPIO.BCM)
#et servoPin as GPIO.OUT
#GPIO.setup(servoPin,GPIO.OUT)
#pwm = GPIO.PWM(servoPin, 100) 
count = 0
#Pi Camera Setup

while(True):
  #Camera object
  camera = PiCamera()
  #Camera resolution
  camera.resolution = (640,480)
  #Camera frame rate
  camera.framerate = 30
  
  
  rawCapture = PiRGBArray(camera, size=(640,480))
  pixels = 640*480
    
  for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame = frame.array
    #Color Detection conversion BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Red Color Detection:
    red_lower = np.array([160,70,50])
    red_upper = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv, red_lower, red_upper)
    result_red = cv2.bitwise_and(frame, frame, mask=red_mask)

    #Yellow Color Detection:   
    #Yellow lower boundary
    yellow_lower = np.array([15,20,100])
    #Yellow upper boundary
    yellow_upper = np.array([30,255,255])
    #Yellow mask
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    #isolate yellow color from the video
    result_yellow = cv2.bitwise_and(frame, frame, mask=yellow_mask)
    
    #20,100,100
    #30,255,255

    #Blue Color Detection
    blue_lower = np.array([100,150,0])
    blue_upper = np.array([140,255,255])
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    result_blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

    #Combine the results so both red,yellow, and blue
    redyellow = cv2.bitwise_or(result_red,result_yellow)
    result = cv2.bitwise_or(redyellow,result_blue)
    
    #Display the final result
    
    cv2.imshow("Red and Yellow Detection in Real Time", result)
    
    #Stop rawCapture to clear the stream in preparation for the next frame

    rawCapture.truncate(0)
    


    #Color Detection for Blue
    #Checks if the frame of the camera sees 25% or more blue
    if(cv2.countNonZero(blue_mask)>(0.25*pixels)):
        count+=1
        print(count)
        #Counts to 10 seconds before watering
        if (count > 9):
            print ("Blue")
            count = 0
            gpio.setwarnings(False)

            gpio.setmode(gpio.BOARD)
            gpio.setup(11, gpio.OUT)
            gpio.output(11, True)
            time.sleep(2)

            gpio.cleanup()

    #Color Detection for Red
    #Checks if the frame of the camera sees 25% or more red
    elif cv2.countNonZero(red_mask)>(0.25*pixels): 
        count+=1
        print(count)
        #Counts to 10 seconds before watering
        if (count > 9):
            print ("Red")
            count = 0
            gpio.setwarnings(False)

            gpio.setmode(gpio.BOARD)
            gpio.setup(11, gpio.OUT)
            gpio.output(11, True)
            time.sleep(5)

            gpio.cleanup()

    #Color Detection for Yellow
    #Checks if the frame of the camera sees 25% or more Yellow
    elif cv2.countNonZero(yellow_mask)>(0.25*pixels):
        count+=1
        print(count)
        #Counts to 10 seconds before watering
        if (count > 9):
            print ("Yellow")
            count = 0
            gpio.setwarnings(False)

            gpio.setmode(gpio.BOARD)
            gpio.setup(11, gpio.OUT)
            gpio.output(11, True)
            time.sleep(10)

            gpio.cleanup()
        #camera.close()
