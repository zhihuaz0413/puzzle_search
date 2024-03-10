class Localization(object):
    def __init__(self, w_center, pan_pos):
        self.w_center = w_center
        self.pan_pos = pan_pos
        self.pos = [2, 0]
        self.my_dir = 0.
        self.cur_grid = [0, 0, 0]
        self.next_grid = [0, 0, 0]
        self.last_grid = [0, 0, 0]
        self.destination = [0, 5]
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

    def get_next_grid(self, level):
        return map(level)
    
    def update_local_map(self, lane_mark, obstacle, pos):
        lane_block_size = 0
        obstacle_block_size = 0
        lane_cy = 0
        obs_cy = 0
        if lane_mark != None:
            center = self.w_centre + 20
            angle_err = lane_mark.cx() - center
            if angle_err < 80 or pos != 1:
                lane_block_size = lane_mark.pixels()
                lane_cy = lane_mark.cy()
#            print('\n' * 2)
#            print('Code:       ', lane_mark.code())
#            print('X-pos:      ',lane_mark.cx())
#            print('Pan angle:  ', self.servo.pan_pos)
#            print('Angle err:  ', angle_err)
#            print('Block size: ', lane_mark.pixels())
        if obstacle != None:
            center = self.w_centre + 20
            angle_err = obstacle.cx() - center
            if abs(angle_err) < 80 or pos != 1:
                obstacle_block_size = obstacle.pixels()
                obs_cy = obstacle.cy()
#            print('\n' * 2)
#            print('Code:       ', obstacle.code())
#            print('X-pos:      ',obstacle.cx())
#            print('Pan angle:  ', self.servo.pan_pos)
#            print('Angle err:  ', angle_err)
#            print('Block size: ', obstacle.pixels())
        # print('obs_size: ', obstacle_block_size)
        # print('lane_size: ', lane_block_size)
        if pos == 1:
            if obs_cy > lane_cy:
                self.next_grid[pos] = 1
            else:
                self.next_grid[pos] = 0

        else:
            if obstacle_block_size > lane_block_size:
                self.next_grid[pos] = 1
            else:
                self.next_grid[pos] = 0


    def update_my_pos(self, sun_blob):
        # According to current observation of sun, lane_mark and obstacles 
        # stored in current_grid, next_grid, update my current postion and direction
        pass


    def get_my_next_step(self, level):
        # According to the global map and current position and direction, 
        # return current next movemement toward the destination; 
        # basically 5 directions 
        #     2  3  4
        #      \ | /
        #     1--|--5 
        pass
