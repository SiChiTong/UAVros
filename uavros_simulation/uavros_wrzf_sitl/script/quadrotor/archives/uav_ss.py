# -*- coding: utf-8 -*-
# author: li xu heng
# time: 2021/7/6 21:28
# project_name: NobodyIsFighting
# file_name: uav_ss
# function: define the class of uav_ss

import uav_parent
import time


class UavSs(uav_parent.StateCom):
    State = '0'  # close, far, 0

    def move_close(self):
        self.State = 'close'
        print('move to state %s' % self.State)
        pass

    def move_far(self):
        self.State = 'far'
        print('move to state %s' % self.State)
        pass

    def detect(self):
        info = ''
        print('detect sth')
        return info

    def track(self):
        if self.Mode != 2:
            return
        self.State = 'far'
        while self.Time <= 10:  # 调整为 360
            self.Time = time.time() - self.Initial_time
            info = self.detect()  # 检测数字并传输
            self.transfer_info(info, self.Uav_center)
            source, info = self.receive_info()
            if info == 'far' and self.State != 'far':
                self.move_far()
            if info == 'close' and self.State != 'close':
                self.move_close()
            time.sleep(0.5)  # 每0.5s更新一次
        self.State = '0'
        self.transaction_mode(3)
        pass


if __name__ == '__main__':
    uav = UavSs(name='uav', uav_ac='uav_ac', current_position='init position')
    uav.init_plane()
    uav.fly(position='game area')
    uav.track()
    uav.ret()
