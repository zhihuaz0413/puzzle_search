from robot import *
from machine import LED
from servos import *

led = LED("LED_BLUE")
led.on()

thresholds = [
              (30, 42, -27, -2, -34, -6), # blue
              (12, 24, 25, 44, 15, 33), # red
              (32, 44, -49, -20, -4, 44), # green
             #  (90, 100, -128, 127, -128, 76)  # compass
             ]

robot = Robot(thresholds, gain = 8)
servo = Servo()

robot.puzzle_search(0.2, 0.25)

servo.soft_reset()
