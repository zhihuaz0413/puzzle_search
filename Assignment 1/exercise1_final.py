from servos import *
from camera import *
from machine import LED

led = LED("LED_BLUE")
led.on()


servo = Servo()
servo.soft_reset()

thresholds = [
              (20, 36, -28, -14, 9, 26),
              (17, 45, -23, 1, -28, -8),
              (62, 79, -21, 1, 27, 53),
              (40, 48, 18, 40, 12, 47),
              (28, 43, 8, 22, -1, 24),
              (57, 72, -5, 15, 31, 51),
              (63, 72, -25, -10, 19, 42),
              ]
camera = Cam(thresholds)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:
# camera.get_blobs_bottom()
# camera.find_blobs()
# servos.set_differential_drive()

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.VGA)
sensor.skip_frames(time=2000)
sensor.set_auto_whitebal(False)  # must be turned off for colour tracking
sensor.set_auto_gain(False, gain_db = 24)
servo.set_differential_drive(0,0)


# for each threshold
# while no blob is detected, rotate to left/ right
#   check camera frame
# once it is detected, stop.
# if on the right, turn to right until no longer on the right
# if on the left, turn to left until no longer on left
# go towards it until you can't see it anymore for more than # no of frames
# repeat for next blob


for colour_index in range(len(thresholds)):
    blobs, _ = camera.get_blobs_bottom()
    blob_index = camera.find_blob(blobs, colour_index)

    print ("LOOKING FOR COLOUR" + str(colour_index))
    if blob_index==None:#if there is no blob of the colour, search until found
        print("no blob detected")
        servo.set_differential_drive(0.05,-0.5)

        if colour_index == 5:
            servo.set_differential_drive (0.2, 0)
            time.sleep_ms(300)
            servo.set_differential_drive (0.05,-0.5)

        while blob_index==None:
            time.sleep_ms(100)
            blobs, _ = camera.get_blobs_bottom()
            blob_index = camera.find_blob(blobs, colour_index)

    print("blob detected")
    servo.set_differential_drive(0,0)
    nextblob = blobs [blob_index]

    while camera.w_centre > nextblob[5]:
        print(" blob on the left")
        # rotate a bit and stop
        servo.set_differential_drive(0.05,0.8)
        time.sleep_ms(50)
        servo.set_differential_drive(0,0)
#        time.sleep_ms(50)
        #get the new position of blob
        blobs, _ = camera.get_blobs_bottom()
        blob_index = camera.find_blob(blobs, colour_index)
        #assing currect blob to be next blob
        if blob_index is not None:
            nextblob = blobs[blob_index]

    # repeat the same to the other side
    while camera.w_centre < nextblob[5]:
        print(" blob on the right")
        servo.set_differential_drive(0.05,-0.8)
        time.sleep_ms(50)
        servo.set_differential_drive(0,0)
#        time.sleep_ms(50)
        blobs, _ = camera.get_blobs_bottom()
        blob_index = camera.find_blob(blobs, colour_index)
        if blob_index is not None:
            nextblob = blobs[blob_index]

    print("going forward")
    frames_with_no_blob = 0
    servo.set_differential_drive(0.2,0)

    while(frames_with_no_blob<4):
        time.sleep_ms(80)
        blobs, _ = camera.get_blobs_bottom()
        blob_index = camera.find_blob(blobs, colour_index)

        if blob_index is None:
            frames_with_no_blob+=1
            servo.set_differential_drive(0,0)

        else:
            frames_with_no_blob =0

            nextblob = blobs[blob_index]
            blob_bias = (camera.w_centre - nextblob[5])/camera.w_centre
            # go a bit
            print ("recalibrate")
            servo.set_differential_drive(0.8,blob_bias)
            time.sleep_ms(50)
            servo.set_differential_drive(0.2,0)

        print(frames_with_no_blob)

    servo.set_differential_drive(0,0)
