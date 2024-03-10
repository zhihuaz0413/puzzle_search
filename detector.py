from camera import *

class Detector(object):
    def __init__(self,thresholds, gain = 10):
         # Blob IDs
        self.cam = Cam(thresholds, gain)
        self.mid_line_id = 0
        self.obstacle_id = 1
        self.destination_id = 2
        self.sun_id = 3
        self.lane_mark = None
        self.obstacle = None
        self.sun = None
        self.destination = None

    def detect_objects(self):
        blobs, img = self.cam.get_blobs()
        lane_marks = self.cam.find_blobs(blobs, self.mid_line_id)
        obstacles = self.cam.find_blobs(blobs, self.obstacle_id)
        sun = self.cam.find_blobs(blobs, self.sun_id)
        destination = self.cam.find_blobs(blobs, self.destination_id)
        # print('sun: ', sun)
        # print('lane_marks: ', lane_marks)
        # print('obstacles: ', obstacles)

        if len(sun) > 0:
            self.sun = self.cam.get_biggest_blob([blobs[i] for i in sun])  
        if len(destination) > 0:
            self.destination = self.cam.get_biggest_blob([blobs[i] for i in destination])
        if len(lane_marks) > 0 or len(obstacles) > 0:
            self.lane_mark = self.cam.get_biggest_blob([blobs[i] for i in lane_marks])
            self.obstacle = self.cam.get_biggest_blob([blobs[i] for i in obstacles])
            if self.lane_mark != None:
                img.draw_rectangle(self.lane_mark.rect(), color=(255,0,0))
                img.draw_string(self.lane_mark.cx(),self.obstacle.cy(),str(self.lane_mark.code()))
            if self.obstacle != None:
                img.draw_rectangle(self.obstacle.rect(), color=(0,255,0))
                img.draw_string(self.obstacle.cx(),self.obstacle.cy(),str(self.obstacle.code()))

    def detect_sum(self):
        # return sun bolb in current image view 
        pass
