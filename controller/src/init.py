#!/usr/bin/env python

import RPi.GPIO as GPIO
import spidev
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	spi_spd = spidev.SpiDev()
	spi_spd.open(0,0)

	spi_dir = spidev.SpiDev()
	spi_dir.open(0,1)

        GPIO.setmode(GPIO.BCM)
	GPIO.setup(8, GPIO.OUT)
	GPIO.setup(7, GPIO.OUT)

	GPIO.output(7, GPIO.LOW)
	spi_spd.xfer([int(128)])
	GPIO.output(7, GPIO.HIGH)
	GPIO.output(8, GPIO.LOW)
	spi_dir.xfer([int(128)])
	GPIO.output(8, GPIO.HIGH)

