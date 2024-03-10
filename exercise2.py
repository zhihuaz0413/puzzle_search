from tuning import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [
      (12, 49, 13, 76, -2, 69), # Red
      (11, 51, -10, 80, -94, -29), # Blue
]

tuning = PanTuning(thresholds, gain = 0, p=0.2, i=0, d=0.005)

tuning.measure(0.1)
