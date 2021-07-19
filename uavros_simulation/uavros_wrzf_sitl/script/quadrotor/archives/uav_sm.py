# -*- coding: utf-8 -*-
# author: li xu heng
# time: 2021/7/6 21:29
# project_name: NobodyIsFighting
# file_name: uav_sm
# function: define the class of uav_sm

import uav_parent
import time


class UavSm(uav_parent.StateCom):
    State = ''  # 2, 3, 0

    def move_2(self):
        self.State = '2'
        print('move to state %s' % self.State)

    def move_3(self):
        self.State = '3'
        print('move to state %s' % self.State)

    def detect(self):
        info = 'sth'
        print('detect %s' % info)
        return info

    def track(self):
        if self.Mode != 2:
            return
        self.State = '2'
        while self.Time <= 10:  # 调整为360
            self.Time = time.time() - self.Initial_time
            info = self.detect()  # 检测数字并传输
            self.transfer_info(info, self.Uav_center)
            source, info = self.receive_info()
            if info == '2' and self.State != '2':
                self.move_2()
            if info == '3' and self.State != '3':
                self.move_3()
            time.sleep(0.5)  # 每0.5s更新一次
        self.State = '0'
        self.transaction_mode(3)
        pass


if __name__ == '__main__':
    uav = UavSm(name='uav', uav_ac='uav_ac', current_position='init position')
    uav.init_plane()
    uav.fly(position='game area')
    uav.track()
    uav.ret()
