#!/usr/bin/env python

from Adafruit_MCP4725 import MCP4725
from Adafruit_ADS1x15 import ADS1x15
import rospy
import RPi.GPIO as GPIO
import spidev
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Controller(object):
    def __init__(self):
        rospy.init_node('controller')

        rospy.Subscriber("cmd_vel", Twist, self.twist_callback)
	rospy.Subscriber("joy", Joy, self.joy_callback)

	self.volt_it = 0.009375
	self.volt_min = 4.8
	self.volt_max = 7.2

	self.adc = ADS1x15(ic=0x01)

	self.set_reference()

	self.dac_spd = MCP4725(0x062)
	self.dac_dir = MCP4725(0x063)
	
	self.dac_spd.setVoltage(int(4095*self.middle))
	self.dac_dir.setVoltage(int(4095*self.middle))

	self.safe = False
	self.num_safe = 0
	self.N = 5
	rospy.spin()

    #Sets the reference pin based on the adc. Called every twist command
    def set_reference(self):
	self.ref = self.adc.readADCSingleEnded(0, 4096, 250) / 1000
	self.ref = self.ref * 3

	self.middle = (self.ref)/9.9 * 4095
        self.low = (self.ref-1.2)/9.9 * 4095
        self.high = (self.ref+1.2)/9.9 * 4095

    #killswitch. Hold A on the XBOX controller. If self.N twist commands are ran without a killswitch update, unsafe.
    def joy_callback(self, joy_msg):
	if joy_msg.buttons[0] == 1:
	    self.safe = True
	    self.num_safe = 0
	else
	    self.safe = False

    def twist_callback(self, twist_msg):
	print("got twist")
	self.set_reference()

	#If the killswitch (A) is not pressed, or self.N twist commands are executed without an update, unsafe.
	if self.safe == False or self.num_safe > self.N:
	    return

	self.num_safe += 1

        xvel = twist_msg.linear.x
        zvel = twist_msg.angular.z * -1
   
        #x velocity
	if xvel > 1:
            xvel = 1

        if xvel < -1:  
            xvel = -1

        
	
	diff = (self.high-self.low)
	scale = (xvel+1)/2 * diff + self.low
	if scale == 4095:
	    scale = 4094

	

	print("Spd:")
	print(scale)
	self.dac_spd.setVoltage(int(scale))
	
	#direction
        if zvel > 1:
            zvel = 1

        if zvel < -1:
            zvel = -1

        scale = (zvel+1)/2 * diff + self.low
	if scale == 4095:
	    scale = 4094
        print("Dir:")
	print(scale)

	self.dac_dir.setVoltage(int(scale))

if __name__ == '__main__':
    Controller()
