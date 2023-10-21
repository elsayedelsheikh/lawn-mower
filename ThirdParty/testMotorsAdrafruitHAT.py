import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

hat.frequency = 60
motor_ch = hat.channels[0]
motor_ch.duty_cycle = 0xffff