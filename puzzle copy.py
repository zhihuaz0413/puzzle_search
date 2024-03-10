from servos import *
from camera import *
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
        self.cam = Cam(thresholds, gain)
        self.PID = PID(p, i, d, imax)

        # Blob IDs
        self.mid_line_id = 0
        self.obstacle_id = 1
        self.destination_id = 2
        self.sun_id = 3
        self.destination = [0, 5]
        self.pos = [0, 0]
        self.my_dir = 0.
        self.cur_grid = [0, 0, 0]
        self.next_grid = [0, 0, 0]
        self.last_grid = [0, 0, 0]
        self.map = [[0, 0, 0, 0], 
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0 ,0, 0]]
        # 5 - - -  
        # 4 - - -  y
        # 3 - - -  ^
        # 2 - - -  |
        # 1 - - -  |
        # 0 1 2 3  ----> x 
        self.scan_direction = 1

    def get_next_grid(self, level):
        return map(level)

    def step_forward(self, speed: float, bias: float):
        print('Hi, Im going forward for 1 step!')
        num = 0
        while True:
            blobs, img = self.cam.get_blobs_bottom(self.mid_line_id)
            found_mid = self.cam.get_biggest_blob(blobs)
            num = num + 1
            print('found_mid: ', found_mid)
            if found_mid is not None:
                img.draw_rectangle(found_mid.rect())
                # steering = round((self.cam.w_centre-found_mid.cx())/(2*self.cam.w_centre),3) # camera centre x2 or img width
                self.servo.set_differential_drive(speed, bias)
                time.sleep_ms(1000)
                break;
            else:
                print('Not found')
                # self.servo.soft_reset()
                if num > 20:
                    break
        self.servo.soft_reset()
        return

    def turn_angle(self, speed, bias, angle):
        steering = bias + 0.01
        self.servo.set_differential_drive(speed, steering)

    def calculate_angle(self, blob, center):
        return 0.

    def update_my_map(self, lane_mark, obstacle):



    def detect_objects(self):
        blobs, img = self.cam.get_blobs()
        lane_marks = self.cam.find_blobs(blobs, self.mid_line_id)
        obstacles = self.cam.find_blob(blobs, self.obstacle_id)
        sun = self.cam.find_blob(blobs, self.sun_id)
        destination = self.cam.find_destination(blobs, self.destination_id)
        print('sun: ', len(sun))
        print('lane_marks: ', len(lane_marks))
        print('obstacles: ', len(obstacles))

        # update my state and local map
        if len(sun) > 0:
            sun_blob = self.cam.get_biggest_blob(sun)
            self.my_dir = self.calculate_angle(sun_blob, self.cam.w_centre)
        if len(destination) > 0:
            des_blob = self.cam.get_biggest_blob(destination)
            # do sth to update the destination position
        if len(lane_marks) > 0 or len(obstacles) > 0:
            lane_mark = self.cam.get_biggest_blob(lane_marks)
            obstacle = self.cam.get_biggest_blob(obstacles)

     
    # def find_next_block(self):



    def puzzle_search(self, speed, bias):
        # self.step_forward(speed, bias)
        angle = 45 # degree
        self.turn_angle(self, speed, )




    def stage1(self, speed: float, bias: float) -> None:
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """

        # changed get_blobs_bottom to take threshold idx input so blobs only contains blobs of that colour
        blobs, img = self.cam.get_blobs_bottom(self.mid_line_id)
        found_mid = self.cam.get_biggest_blob(blobs)

        while True:
            if found_mid:
                """
                ###Level 1### Please insert code here to compute the center line angular error as derived from the pixel error, then use this value
                to come up with a steering command to send to self.drive(speed, steering) function. Remember the steering takes values between -1 and 1.

                ###Level 2### Please insert code here to follow the lane when the red line is obstructed. How would you make sure the pixyBot still stays on the road?
                Come up with a steering command to send to self.drive(speed, steering) function
                """
                steering = round((self.cam.w_centre-found_mid.cx())/(2*self.cam.w_centre),3) # camera centre x2 or img width
                self.drive(speed, steering)
                time.sleep_ms(100)
                blobs, img = self.cam.get_blobs_bottom(self.mid_line_id)
                found_mid = self.cam.get_biggest_blob(blobs)

            else:
                print('Not found')
               # self.drive(0, 0) # might cause issues if servos are not calibrated so I put soft reset instead
                self.servo.soft_reset()
                break # this is for now but it might need to look around for it if it doesn't see it

        self.servo.soft_reset()
        return


    def stage2(self, speed: float, bias: float) -> None:
        """
        Obstacle detection algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        counter = 0
        while True:
            print("Start")
            blobs, img = self.cam.get_blobs_bottom(self.mid_line_id)
            found_mid = self.cam.get_biggest_blob(blobs)
            blobs_obs, img_obs = self.cam.get_blobs_bottom(self.obstacle_id)
            found_obstacle = self.cam.get_biggest_blob(blobs_obs)

            print('obstacle: ', found_obstacle)
            if found_obstacle != None:
                print("Obstacle")
                #print(found_obstacle)
                self.drive(0,0)
                time.sleep_ms(100)
                self.track_blob(found_obstacle)
                counter += 1
                self.servo.soft_reset()
                if counter > 50:
                    break

            elif found_mid != None:
                print("Drive")
                steering = round((self.cam.w_centre-found_mid.cx())/(2*self.cam.w_centre),3) # camera centre x2 or img width
                self.drive(speed, steering)
                time.sleep_ms(100)

            else:
                print('Not found')
               # self.drive(0, 0) # might cause issues if servos are not calibrated so I put soft reset instead
                self.servo.soft_reset()
                break

        #self.servo.soft_reset()
        return


    def stage3(self, speed: float, bias: float, distance_threshold = 2.5) -> None:
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        while True:
            print("Start")
            blobs, img = self.cam.get_blobs_bottom(self.mid_line_id)
            found_mid = self.cam.get_biggest_blob(blobs)
            blobs_obs, img_obs = self.cam.get_blobs_bottom(self.obstacle_id)
            found_obstacle = self.cam.get_biggest_blob(blobs_obs)

            if found_obstacle != None:

                distance = 7.1787*math.exp(0.0077* (sensor.height()-(found_obstacle.cy() + found_obstacle.h()/2)))
                distance -= 3 # offset in cm between camera view and robot actual position
                print(distance)

#                # Set an arbitrary distance threshold to stop the robot
#                distance_threshold = 2.5  # Adjust this based on your specific requirements and calibration

                # Check if the obstacle is within the specified distance
                if distance < distance_threshold:
                    print(f"Obstacle within {distance} units. Stopping.")
                    self.drive(0, 0)
                    self.track_blob(found_obstacle)
                    break # TODO REMOVE THIS AFTER DEV

                else:
                    # Continue moving according to lane-following logic
                    print("Drive")
                    steering = round((self.cam.w_centre-found_obstacle.cx()) / (2 * self.cam.w_centre), 3)
                    self.drive(speed, steering)
                    time.sleep_ms(100)

            elif found_mid != None:
                print("Drive")
                steering = round((self.cam.w_centre-found_mid.cx())/(2*self.cam.w_centre),3) # camera centre x2 or img width
                self.drive(speed, steering)
                time.sleep_ms(100)

            else:
                print('Not found')
               # self.drive(0, 0) # might cause issues if servos are not calibrated so I put soft reset instead
                self.servo.soft_reset()
                break

        self.servo.soft_reset()
        return


    def stage3_v1(self, speed: float, bias: float,distance_threshold) -> None:
        """
        Obstacle distance + orientation algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        while True:
            print("Start")
            blobs, img = self.cam.get_blobs_bottom(self.mid_line_id)
            found_mid = self.cam.get_biggest_blob(blobs)
            blobs_obs, img_obs = self.cam.get_blobs_bottom(self.obstacle_id)
            found_obstacle = self.cam.get_biggest_blob(blobs_obs)
            print("Drive")
            if found_mid != None:
                print("Drive")
                steering = round((self.cam.w_centre-found_mid.cx())/(2*self.cam.w_centre),3) # camera centre x2 or img width
                self.servo.set_differential_drive(drive, steering)
                time.sleep_ms(100)

            if found_obstacle != None:

                # Assuming obstacle_height is the vertical size of the obstacle blob in pixels
                distance = 7.1787*math.exp(0.0077* (sensor.height()-(found_obstacle.cy() + found_obstacle.h()/2)))
                distance -= 3 # offset in cm between camera view and robot actual position
                print(distance)

#                # Set an arbitrary distance threshold to stop the robot
#                distance_threshold = 2.5  # Adjust this based on your specific requirements and calibration

                # Check if the obstacle is within the specified distance
                if distance < distance_threshold:
                    print(f"Obstacle within {distance} units. Stopping.")
                    self.drive(0, 0)
                    self.track_blob(found_obstacle)
                    time.sleep_ms(5000)
                    break # TODO REMOVE THIS AFTER DEV

            else:
                print('Not found')
               # self.drive(0, 0) # might cause issues if servos are not calibrated so I put soft reset instead
                self.servo.soft_reset()
                #break

        self.servo.soft_reset()
        return



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
            blobs, _ = self.cam.get_blobs_bottom(threshold_idx)
            found_idx = self.cam.find_blob(blobs, threshold_idx)

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