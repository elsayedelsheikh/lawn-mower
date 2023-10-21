import time
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

kit.servo[0].actuation_range = 160

while True:
	kit.servo[0] = 0
	time.sleep(3)
	kit.servo[0] = 90
	time.sleep(3)
	kit.servo[0] = 160
	
