# -*- coding: utf-8 -*-
# author: li xu heng
# time: 2021/7/6 19:38
# project_name: NobodyIsFighting
# file_name: uav_a1_a2
# function: define the class of uav_a1 and uav_a2

import uav_parent
import time


class UavA1A2(uav_parent.StateCom):
    State = '0'  # up, down, 0

    def move_up(self):
        self.State = 'up'
        print('move to state %s' % self.State)
        pass

    def move_down(self):
        self.State = 'down'
        print('move to state %s' % self.State)
        pass

    def track(self):
        if self.Mode == 2:
            self.State = 'up'
        while self.Time <= 10:
            self.Time = time.time() - self.Initial_time
            source, info = self.receive_info()
            if info == 'up' and self.State != 'up':
                self.move_up()
            if info == 'down' and self.State != 'down':
                self.move_down()
            time.sleep(0.5)  # 每0.5s更新一次
        self.State = '0'
        self.transaction_mode(3)
        pass


if __name__ == '__main__':
    uav = UavA1A2(name='uav', uav_ac='uav_ac', current_position='init position')
    uav.init_plane()
    uav.fly(position='game area')
    uav.track()
    uav.ret()
