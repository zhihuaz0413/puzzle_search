from servos import *
from camera import *
from machine import LED
import math

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

dg = [19, -21, 19]
b = [23, -5, -19]
y = [51, -11, 47]
r = [23, 30, 23]
p = [18, 9, 4]
o = [36, 10, 38]
lg = [47, -21, 32]

thresholds = []
colours = [dg, b, y, r, p, o, lg]
for colour in colours:
    thresholds.append((colour[0]-7, colour[0]+7, colour[1]-7, colour[1]+7, colour[2]-7, colour[2]+7))

colour_names = ['dark_green', 'blue', 'yellow', 'red', 'purple', 'orange', 'light green']
camera = Cam(thresholds, 20)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:

####################################################################################################
### The Exercise
start = time.time()
duration = 120
direction_threshold = 0.1
try:
    for idx, color_threshold in enumerate(thresholds):
        print('Looking for blob:', idx)
        searching = True

        # Search for colour
        while searching:
            if time.time() - start > duration:
                break
            blobs, img = camera.get_blobs_bottom()
            if blobs:
                # Draw rectangle around blob of interest
                for blob in blobs:
                    img.draw_rectangle(blob.rect())
                    if blob.elongation() > 0.5:
                         img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                         img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                         img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))

                    img.draw_cross(blob.cx(), blob.cy())
                    img.draw_keypoints(
                        [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                    )
                found_idx = camera.find_blob(blobs, idx)
                # If blob found, stop spinning
                if found_idx is not None:
                    print('Found this colour:', colour_names[idx])
                    direction = blobs[found_idx].cx() / img.width()
                    print('direction: ', direction)
                    if direction < 0.5 + direction_threshold and direction > 0.5 - direction_threshold:
                        print('direction good')
                        servo.set_differential_drive(0.5, -0.2)
                        searching = False
                        continue
            # If blob not found or not central, spin
            servo.set_differential_drive(0.1, 0.8)
            time.sleep_ms(150)
            servo.set_differential_drive(0, 0)

        frames_unseen_count = 0
        while True:
            # Move to found colour
            if time.time() - start > duration:
                break
            blobs, img = camera.get_blobs_bottom()
            if blobs:
                found_idx = camera.find_blob(blobs, idx)
                if found_idx is not None:
                    continue
            # Stop and start looking for next colour when blob isnt seen for 10 frames
            frames_unseen_count += 1
            if frames_unseen_count > 7:
                servo.set_differential_drive(0, 0)
                time.sleep_ms(1000)
                if idx == 4 or idx == 5:
                    servo.set_differential_drive(0.3, -0.2)
                    time.sleep_ms(1000)
                    servo.set_differential_drive(0, 0)
                break

        if time.time() - start > duration:
            print('Taking too long!!')
            error('TOO LONG')
            servo.set_differential_drive(0, 0)
            break

except Exception as e:
    print('An error occurred:', e)
    # Set speed to 0 and perform soft reset
    servo.set_differential_drive(0, 0)
    servo.soft_reset()
    print('Robot reset')
    print('Robot failed at colour threshold: ', idx)


####################################################################################################
#### Test 1
#print('START')
#servo.set_differential_drive(0.5, 0)
#time.sleep_ms(5000)
#servo.set_differential_drive(0, 0)
#servo.soft_reset()
#print('DONE')
####################################################################################################
#### TEST 2
#print('START')
#servo.set_differential_drive(-0.5, 0)
#time.sleep_ms(1000)
#servo.set_differential_drive(1, 0)
#time.sleep_ms(1000)
#servo.set_differential_drive(-0.5, 1)
#time.sleep_ms(1000)
#servo.set_differential_drive(-0.5, -1)
#time.sleep_ms(1000)
#servo.set_differential_drive(0.5, 1)
#time.sleep_ms(1000)
#servo.set_differential_drive(0.5, -1)
#time.sleep_ms(1000)
#servo.set_differential_drive(0, 0)
#servo.soft_reset()
#print('DONE')
