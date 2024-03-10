from servos import *
from localization import *
from detector import *
from pid_control import PID
import time
import math

class Puzzle(object):
    def __init__(self, thresholds, gain = 10, p=0.15, i=0, d=0.005, imax=0.01):
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
        self.PID = PID(p, i, d, imax)
        self.detector = Detector(thresholds, gain)
        self.loc = Localization(self.detector.cam.w_centre, self.servo.pan_pos)
        
        self.scan_direction = 1
        self.servo.set_angle(0)

    def step_forward(self, speed: float, bias: float):
        print('Hi, Im going forward for 1 step!')
        # consider using observation as feedbacks
        num = 0
        self.servo.set_angle(0)
        while True:
            blobs, img = self.cam.get_blobs_bottom(self.mid_line_id)
            found_mid = self.cam.get_biggest_blob(blobs)
            num = num + 1
            # print('found_mid: ', found_mid)
            if found_mid is not None:
                img.draw_rectangle(found_mid.rect())
                self.servo.set_differential_drive(speed, bias)
                time.sleep_ms(900)
                break;
            else:
                print('Not found')
                if num > 15:
                    break
        self.servo.soft_reset()

    # turn a specific angle to move diag direction, in most cases 45 degree, 
    # considering using sun direction as feedbacks
    def turn_angle(self, speed, bias, angle):
        steering = bias + angle * 0.001
        self.servo.set_differential_drive(speed, steering)

    def search_nearby(self, left = True):
        if left:
            self.servo.set_angle(50)
            print('left: (1) -> ', self.servo.pan_pos)

        else:
            self.servo.set_angle(-40)
            print('right: (1) -> ', self.servo.pan_pos)
        return self.detect_objects()


    def debug_obs(self):
        for i in range(300):
            print('current #_# pan_pos:', self.servo.pan_pos)
            lane_mark_l, obstacle_l = self.search_nearby(True) # left
            self.loc.update_local_map(lane_mark_l, obstacle_l, 2)
            print("*_*: ", self.next_grid)
            time.sleep_ms(1000)
        self.servo.soft_reset()

    def puzzle_search(self, speed, bias):
        # self.step_forward(speed, bias)
        # angle = 45 # degree
        # self.turn_angle(self, speed, )

        time.sleep_ms(1000)
        for i in range(500):
            self.detector.detect_objects()
            self.loc.update_local_map(self.detector.lane_mark, self.detector.obstacle, 1)
            if self.loc.next_grid[1] == 1:
                print('current #_# pan_pos:', self.servo.pan_pos)
                lane_mark_l, obstacle_l = self.search_nearby(True) # left
                self.loc.update_local_map(lane_mark_l, obstacle_l, 0)
                time.sleep_ms(1000)
                print("**********: ", self.loc.next_grid)
                self.servo.set_angle(0)
                time.sleep_ms(1000)
                print('current #_# pan_pos:', self.servo.pan_pos)
                lane_mark_r, obstacle_r = self.search_nearby(False) # right
                self.loc.update_local_map(lane_mark_r, obstacle_r, 2)
                time.sleep_ms(1000)
                self.servo.set_angle(0)
                time.sleep_ms(1000)
                print("-----------: ", self.loc.next_grid)
                # break
            else:
                self.step_forward(speed, bias);


        self.servo.soft_reset()
    
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
        pixel_error = blob.cx() - self.detector.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/self.detector.cam.w_centre / 2 *self.detector.cam.h_fov)

        pid_error = self.PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        # Move pan servo to track block
        self.servo.set_angle(pan_angle)

        return pan_angle


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


            # Check blobs to see if the line is found
            blobs, _ = self.detector.cam.get_blobs_bottom(threshold_idx)
            found_idx = self.detector.cam.find_blob(blobs, threshold_idx)

            if found_idx:
                print("found")
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                print("Not found")
                self.scan_direction *= -1



    def debug(self, threshold_idx: int) -> None:
        """
        A debug function for the Robots vision.
        If no block ID is specified, all blocks are detected and debugged.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
        """
        while True:
            blobs, img = self.detector.cam.get_blobs()
            if threshold_idx is not None:
                found_idx = self.detector.cam.find_blob(blobs, threshold_idx)
            else:
                found_idx = range(len(blobs))

            if found_idx:
                for blob in [blobs[i] for i in found_idx]:
                    img.draw_rectangle(blob.rect(), color=(255,0,0))
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
