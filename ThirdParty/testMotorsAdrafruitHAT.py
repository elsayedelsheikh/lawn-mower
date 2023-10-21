import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)
hat.frequency = 60

## Backleft
motor_bl = hat.channels[0]

## Backright
motor_br = hat.channels[4]

## Frontleft
motor_fl = hat.channels[8]

## Frontright
motor_fr = hat.channels[12]

## Set the duty cycle to 0xffff
motor_bl.duty_cycle = 0xffff
motor_br.duty_cycle = 0xffff
motor_fl.duty_cycle = 0xffff
motor_fr.duty_cycle = 0xffff
