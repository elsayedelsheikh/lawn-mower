import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)
hat.frequency = 60

## Backleft
motor_bl = hat.channels[0]
motor_bl_dir = hat.channels[1]

## Backright
motor_br = hat.channels[4]
motor_br_dir = hat.channels[5]

## Frontleft
motor_fl = hat.channels[8]
motor_fl_dir = hat.channels[9]

## Frontright
motor_fr = hat.channels[12]
motor_fr_dir = hat.channels[13]

## Set the duty cycle to 0xffff
motor_bl.duty_cycle = 0xffff
motor_bl_dir.duty_cycle = 0xffff

motor_br.duty_cycle = 0xffff
motor_br_dir.duty_cycle = 0xffff

motor_fl.duty_cycle = 0xffff
motor_fl_dir.duty_cycle = 0xffff

motor_fr.duty_cycle = 0xffff
motor_fr_dir.duty_cycle = 0xffff
