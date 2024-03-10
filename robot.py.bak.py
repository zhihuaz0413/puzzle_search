from servos import *
from camera import *
from pid_control import PID
import time
import math
import sensor
#import numpy as np

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 10, p=0.18, i=0, d=0, imax=0.01):
        """
        Initializes the Robot object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds
            gain (float): Camera gain
            p (float): Proportional gain for the PID.
            i (float): Integral gain for the PID.
            d (float): Derivative gain for the PID.
            imax (float): Maximum Integral error for the PID.
        """
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)
        self.PID = PID(p, i, d, imax)
        self.thresholds = thresholds

        # Blob IDs
        self.mid_line_id = 0
        self.obstacle_id = 1
        self.l_line_id = 2
        self.r_line_id = 3
        self.scan_direction = 1


    def stage1(self, speed: float, bias: float) -> None:
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        time.sleep_ms(1000)
        print('starting stage 1')
        frames = 0

        # Find the initial lane to follow
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break
            elif biggest_blob_color == pow(2, 2):
                color_id = self.l_line_id
                break
            elif biggest_blob_color == pow(2, 3):
                color_id = self.r_line_id
                break

        print('color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            for blob in img.find_blobs(self.thresholds, pixels_threshold=150, area_threshold=150,
                                       roi=(1,int(sensor.height()/3),int(sensor.width()),int(2*sensor.height()/3))):
                img.draw_rectangle(blob.rect())

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), blob.cy())
                img.draw_keypoints(
                    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
                )
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)

            if found_mid is not None:

                # Level 1
                print('found mid')
                self.servo.set_angle(0)
                pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                steering = pixel_error / self.cam.w_centre
                drive = steering / 3
                print('steering', steering, 'drive:', bias - drive)
                if steering < 0.1 and steering > -0.1:
                    self.drive(speed, bias)
                else:
                    self.drive(speed, bias - drive)
            else:
                frames += 1
                if frames > 5:
                    self.drive(0, 0)
                    break

        self.servo.soft_reset()
        return


    def stage2(self, speed: float, bias: float) -> None:
        """
        Obstacle detection algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """

        time.sleep_ms(1000)
        print('starting stage 2')

        # Find the initial lane to follow
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break
            elif biggest_blob_color == pow(2, 2):
                color_id = self.l_line_id
                break
            elif biggest_blob_color == pow(2, 3):
                color_id = self.r_line_id
                break

        print('lane color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)
            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)

            # If obstacle not there, follow lane
            if obstacle_detected is None:
                if found_mid is not None:
                    print('found mid')
                    self.servo.set_angle(0)
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    steering = steering / 2
                    print('steering', steering, 'drive:', bias - steering)
                    if steering < 0.05 and steering > -0.05:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - steering)
            # If obstacle, stop
            else:
                print('OBSTACLE!!')
                self.drive(0, 0)
                break
        return


    def stage3(self, speed: float, bias: float, distance_threshold: float) -> None:
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
            obstacle_size (float): height of object (cm)
            distance_threshold (float): how far from the object the robot should stop
        """

#        diagonal_fov = math.sqrt((cam.h_fov ** 2) + (cam.v_fov ** 2))
#        focal_length = (sensor.width()) / 2) / np.tan(diagonal_fov / 2)

        time.sleep_ms(1000)
        print('starting stage 3')

        # Find the initial lane to follow
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            biggest_blob = self.cam.get_biggest_blob(blobs)
            biggest_blob_color = biggest_blob[8]

            print('check color')

            if biggest_blob_color == pow(2, 0):
                color_id = self.mid_line_id
                break
            elif biggest_blob_color == pow(2, 2):
                color_id = self.l_line_id
                break
            elif biggest_blob_color == pow(2, 3):
                color_id = self.r_line_id
                break

        print('lane color detected', color_id)

        # Following the lane
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            print('finding blobs of color', color_id)
            found_mid = self.cam.find_blob(blobs, color_id)
            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)

            # If obstacle not there, follow lane
            if obstacle_detected is None:
                if found_mid is not None:
                    print('found mid')
                    self.servo.set_angle(0)
                    pixel_error = blobs[found_mid].cx() - self.cam.w_centre
                    steering = pixel_error / self.cam.w_centre
                    steering = steering / 2
                    print('steering', steering, 'drive:', bias - steering)
                    if steering < 0.05 and steering > -0.05:
                        self.drive(speed, bias)
                    else:
                        self.drive(speed, bias - steering)
            # If obstacle, stop
            else:
                print('obstacle detected')
                distance_pixel = blobs[obstacle_detected].y() + blobs[obstacle_detected].h()
                distance_to_obstacle = 1000000 * (distance_pixel ** (-1.921))



#                apparent_height = blobs[obstacle_detected].height()
#                vertical_displacement = apparent_height / math.tan(math.radians(self.cam.camera_elevation_angle))
#                distance_pixel = obstacle_size * focal_length / apparent_height + vertical_displacement   # in pixel
#                distance_to_obstacle = 1000000 * (distance_pixel ** (-1.921))
                if distance_to_obstacle <= distance_threshold:
                    print('STOP')
                    self.drive(0, 0)  # Stop the robot
                    break
                else:
                    print('closerrrr')
                    self.drive(speed / 2, bias)
        return


    def stage4(self, speed: float, bias: float) -> None:
        """
        Obstacle distance + orientation algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.soft_reset()
        return


    def stage5(self, speed: float, bias: float) -> None:
        """
        Obstacle avoidance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.soft_reset()
        time.sleep_ms(1000)
        print('starting stage 2')

        # Find the initial lane to follow
        while True:
            blobs, img = self.cam.get_blobs_bottom()
            obstacle_detected = self.cam.find_blob(blobs, self.obstacle_id)
            img.draw_cross(int(self.cam.w_centre), int(self.cam.h_centre))

            # print(obstacle_detected)

            if obstacle_detected is not None:
                blob = blobs[obstacle_detected]
                img.draw_rectangle(blob.rect())
                delta_y = (blob.cy() + blob.h()/2) - self.cam.h_centre ## change blob bottom coordinates
                beta = delta_y/self.cam.h_centre / 2 * self.cam.h_fov
                distance =abs(6.3 /math.tan((self.cam.camera_elevation_angle + beta)*math.pi/180))
                print(distance)

                if blob.elongation() > 0.5:
                    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
                    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
                    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
                img.draw_cross(blob.cx(), int(blob.cy()-self.cam.h_centre/2))

        return




    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
        self.servo.set_differential_drive(drive, steering)


    def track_blob(self, blob) -> None:
        """
        Adjust the camera pan angle to track a specified blob based on its ID.

        Args:
            blob: The blob object to be tracked
        """
        # Error between camera angle and target in pixels
        pixel_error = blob.cx() - self.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/sensor.width()*self.cam.h_fov)

        pid_error = self.PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        # Move pan servo to track block
        self.servo.set_angle(pan_angle)


    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 20) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)
            print(new_pan_angle, self.servo.pan_pos, limit)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            found_idx = self.cam.find_blob(blobs, threshold_idx)
            if found_idx:
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit:
                print('reverse')
                self.scan_direction = -1
            if self.servo.pan_pos <= -limit:
                print('reverse')
                self.scan_direction = 1


    def debug(self, threshold_idx: int) -> None:
        """
        A debug function for the Robots vision.
        If no block ID is specified, all blocks are detected and debugged.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
        """
        while True:
            blobs, img = self.cam.get_blobs()
            if threshold_idx is not None:
                found_idx = self.cam.find_blob(blobs, threshold_idx)
            else:
                found_idx = range(len(blobs))

            if found_idx:
                for blob in [blobs[i] for i in found_idx]:
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(),blob.cy(),str(blob.code()))

                    angle_err = blob.cx() - self.cam.w_centre

                    print('\n' * 2)
                    print('Code:       ', blob.code())
                    print('X-pos:      ',blob.cx())
                    print('Pan angle:  ', self.servo.pan_pos)
                    print('Angle err:  ', angle_err)
                    print('Angle corr: ', (angle_err-self.servo.pan_pos)/self.servo.max_deg)
                    print('Block size: ', blob.pixels())

                    time.sleep(1)


    def reset(self) -> None:
        """
        Resets the servo positions to their default states and waits.
        """
        self.servo.soft_reset()


    def release(self) -> None:
        """
        Release all servos (no wait).
        """
        self.servo.release_all()
