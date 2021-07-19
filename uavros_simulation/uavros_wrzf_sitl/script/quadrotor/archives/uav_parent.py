# -*- coding: utf-8 -*-
# author: li xu heng
# time: 2021/7/6 16:36
# project_name: NobodyIsFighting
# file_name: uav_parent
# function: define the parent of uav

import time


class Uav:
    mode = 0  # 0,1,2,3
    time = 0
    name = ''
    uav_center = ''  # 中心节点uav的信息
    position = [0, 0, 0]  # 位置信息
    initial_time = 0  # 系统初始时间，用来计算相对时间

    def __init__(self, name, uav_ac, current_position):
        self.name = name
        self.uav_center = uav_ac
        self.position = current_position
        self.transaction_mode(0)
        self.initial_time = time.time()
        # TODO: config XBee Device
        pass

    def set_initialise_time(self, time_init):
        self.initial_time = time_init

    def transfer_info(self, info, target):
        print('%s 发送 %s 给 %s' % (self.name, info, target))

    def receive_info(self):
        source = 'uav_center'
        info = '起飞'
        if self.position == 'game area':
            info = 'close'
        if self.time >= 10:
            info = '返航'
        print('从 %s 收到 %s' % (source, info))
        return source, info

    def move(self, target_position):
        # TODO: 修改。应该是逐渐达到的。
        self.position = target_position
        if self.position == target_position:
            print('成功到达 %s' % target_position)

    def transaction_mode(self, mode):
        self.mode = mode

    def init_plane(self):
        print("Initialise %s!" % self.name)
        info = '初始化成功'
        return True

    def fly(self, position):
        if self.mode != 1:
            print('尚未到起飞模式')
            return
        while 1:
            source, info = self.receive_info()
            if info == '起飞':
                break
        print('起飞')

        self.initial_time = time.time()

        while 1:
            self.move(position)
            if self.position == position:
                break
        info = '到达'
        self.transfer_info(info, self.uav_center)
        while self.time <= 6:  # 调整为180
            self.time = time.time() - self.initial_time
            print('准备完毕，时间：%0.2f' % self.time)
            time.sleep(2)
        self.transaction_mode(2)

    def track(self):
        while self.time <= 10:  # 调整为360
            self.time = time.time() - self.initial_time
            print('正在跟踪，时间：%0.2f' % self.time)
            time.sleep(2)
        self.transaction_mode(3)

    def ret(self):
        if self.mode != 3:
            return
        print('返航')
        while 1:
            source, info = self.receive_info()  # info包含返航指令和返航坐标
            if info == '返航':
                break
        self.move(info)

    def prepare_info(self, info):
        return self.name + ":" + info


if __name__ == '__main__':
    uav = Uav(name='uav', uav_ac='uav_ac', current_position='init position')
    uav.init_plane()
    uav.fly(position='game area')
    uav.track()
    uav.ret()
