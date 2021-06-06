import os
import cv2
import numpy as np
import RPi.GPIO as GPIO
from time import sleep


GPIO.setmode(GPIO.BOARD)

motorspeed_pin = 8
DIRA = 10
DIRB = 22

def turnOff():
	pwmPIN.ChangeDutyCycle(0)
	GPIO.output(DIRA, GPIO.LOW)
	GPIO.output(DIRB, GPIO.LOW)

GPIO.setup(motorspeed_pin, GPIO.OUT)
GPIO.setup(DIRA, GPIO.OUT)
GPIO.setup(DIRB, GPIO.OUT)

from picamera import PiCamera

from time import sleep

camera = PiCamera()
camera.start_preview(alpha=192)
sleep(1)
camera.capture("{}/a.jpg".format(os.getcwd()))
camera.stop_preview()

faceCascade = cv2.CascadeClassifier('regnumber.xml')

img = cv2.imread('car4.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

faces = faceCascade.detectMultiScale(gray,scaleFactor=1.2,
    minNeighbors = 5, minSize=(25,25))
pwmPIN = GPIO.PWM(motorspeed_pin, 100)
pwmPIN.start(0)
turnOff()
for (x,y,w,h) in faces:
    pwmPIN.ChangeDutyCycle(100)
    GPIO.output(DIRA, GPIO.HIGH)
    GPIO.output(DIRB, GPIO.LOW)
    #cv2.rectangle(gray,(x,y),(x+w,y+h),(255,0,0),2)
    #plate = gray[y: y+h, x:x+w]
    #plate = cv2.blur(plate,ksize=(20,20))
    # put the blurred plate into the original image
    #gray[y: y+h, x:x+w] = plate

cv2.imshow('plates',gray)
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows()
