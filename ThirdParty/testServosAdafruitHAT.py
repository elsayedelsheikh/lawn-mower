import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

## Backleft 
kit.servo[2].actuation_range = 160

## Backright
kit.servo[6].actuation_range = 160

## Frontleft
kit.servo[10].actuation_range = 160

## Frontright
kit.servo[14].actuation_range = 160

while True:
	kit.servo[2] = 0
	kit.servo[6] = 0
	kit.servo[10] = 0
	kit.servo[14] = 0
	time.sleep(3)

	kit.servo[2] = 90
	kit.servo[6] = 90
	kit.servo[10] = 90
	kit.servo[14] = 90
	time.sleep(3)

	kit.servo[2] = 160
	kit.servo[6] = 160
	kit.servo[10] = 160
	kit.servo[14] = 160
	time.sleep(3)
	
