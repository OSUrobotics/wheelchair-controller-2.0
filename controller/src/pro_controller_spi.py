#!/usr/bin/env python

from Adafruit_MCP4725 import MCP4725
from Adafruit_ADS1x15 import ADS1x15
import rospy
import RPi.GPIO as GPIO
import spidev
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time

class Controller(object):
    def __init__(self):
        rospy.init_node('controller')

        rospy.Subscriber("cmd_vel", Twist, self.twist_callback)
	rospy.Subscriber("joy", Joy, self.joy_callback)

	GPIO.setmode(GPIO.BCM)
	GPIO.setup(7, GPIO.OUT)
	GPIO.setup(8, GPIO.OUT)

	self.volt_it = 0.009375
	self.volt_min = 4.8
	self.volt_max = 7.2

	self.high = 256
	self.low = 0

	self.spi_spd = spidev.SpiDev()
	self.spi_spd.open(0,1)

	self.spi_dir = spidev.SpiDev()
	self.spi_dir.open(0,0)
	
	self.safe = False
	self.num_unsafe = 0
	self.N = 5

	self.old_axes = -99;
	#rospy.Timer(rospy.Duration(0.1), self.runstop_callback)
	rospy.spin()

    def runstop_callback(self,event):
	#self.num_safe += 1
	if self.num_unsafe > self.N and self.safe:
            GPIO.output(7, GPIO.LOW)
	    self.spi_spd.xfer([int(128)])
            GPIO.output(7, GPIO.HIGH)
            GPIO.output(8, GPIO.LOW)
	    self.spi_dir.xfer([int(128)])
            GPIO.output(8, GPIO.HIGH)
            print("TIMER RUNSTOP WORKED!")
	    self.safe = False
	    self.num_unsafe = 0
    
    def joy_callback(self, joy_msg):
        if joy_msg.buttons[5] == self.old_axes:
	    self.num_unsafe += 1
	if joy_msg.buttons[5] != self.old_axes:
	    print("SAFE")
	    self.safe = True
	    self.num_unsafe = 0

	self.old_axes = joy_msg.buttons[5]
    

    def twist_callback(self, twist_msg):
	print("got twist")
	#If the killswitch (A) is not pressed, or self.N twist commands are executed without an update, unsafe.
	#if self.safe == False or self.num_unsafe > self.N:
	    #return

        xvel = twist_msg.linear.x
        zvel = twist_msg.angular.z * -1
   
        #x velocity
	if xvel > 1:
            xvel = 1

        if xvel < -1:  
            xvel = -1

        
	
	diff = (self.high-self.low)
	scale = (xvel+1)/2 * diff + self.low
	if scale == 256:
	    scale = 255

	print("Spd:")
	print([int(scale)])
	
	GPIO.output(7, GPIO.LOW)
	self.spi_spd.xfer2([int(scale)])
	time.sleep(0.1)
	GPIO.output(7, GPIO.HIGH)
	#direction
        if zvel > 1:
            zvel = 1

        if zvel < -1:
            zvel = -1

        scale = (zvel+1)/2 * diff + self.low
	if scale == 256:
	    scale = 255
        print("Dir:")
	print([int(scale)])

	GPIO.output(8, GPIO.LOW)
	self.spi_dir.xfer2([int(scale)])
	time.sleep(0.1)
	GPIO.output(8, GPIO.HIGH)

if __name__ == '__main__':
    Controller()
