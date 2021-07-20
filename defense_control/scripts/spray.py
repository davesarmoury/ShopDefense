#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
import RPi.GPIO as GPIO

servoPIN = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50)
p.start(2.5)

MAX = 6.27450980392
MIN = 1.96078431373

def callback(msg):
    p.ChangeDutyCycle(MIN + MAX * abs( msg.data - 255.0 ) / 255.0)

def servo_driver():
    rospy.init_node('sprayer', anonymous=True)

    rospy.Subscriber("spray", UInt8, callback)

    p.ChangeDutyCycle(MAX)
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_driver()
    except KeyboardInterrupt:
        p.stop()
        GPIO.cleanup()

    p.stop()
    GPIO.cleanup()

