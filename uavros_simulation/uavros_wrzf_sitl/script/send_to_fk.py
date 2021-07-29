# -*- coding: utf-8 -*-
# author: li xu heng
# time: 2021/717 11:22
# project_name: NobodyIsFighting
# file_name: send_to_fk
# function: 向飞控发送指令

import time
from digi.xbee.devices import XBeeDevice
import serial
import cmd_with_outer_hardware as cmd_outer

BAUD_RATE = 115200  # 115200


class Data2Fk:
    follower_desired_id = 0  # int 16
    follower_desired_cmd = 0  # unit_8
    follower_desired_type = 0  # unit_8
    current_loc_lat = 0.1  # int_32
    current_loc_lng = 0.1  # int_32
    altitude = 0.1  # int_32
    velocity_n = 0.1  # int_16
    velocity_e = 0.1  # int_16
    velocity_d = 0.1  # int_16
    yaw = 0.1  # int_16
    buffer = bytearray()

    def __init__(self, id=0, cmd=0, type=0, current_loc=None, v=None, yaw=0):

        if v is None:
            v = [0, 0, 0]
        if current_loc is None:
            current_loc = [0, 0, 0]

        self.follower_desired_id = id
        self.follower_desired_cmd = cmd
        self.follower_desired_type = type
        self.current_loc_lat = current_loc[0]
        self.current_loc_lng = current_loc[1]
        self.altitude = current_loc[2]
        self.velocity_n = v[0]
        self.velocity_e = v[1]
        self.velocity_d = v[2]
        self.yaw = yaw

    def update(self, id=0, cmd=0, type=0, current_loc=None, v=None, yaw=0):

        if v is None:
            v = [0, 0, 0]
        if current_loc is None:
            current_loc = [0, 0, 0]

        self.follower_desired_id = id
        self.follower_desired_cmd = cmd
        self.follower_desired_type = type
        self.current_loc_lat = current_loc[0]
        self.current_loc_lng = current_loc[1]
        self.altitude = current_loc[2]
        self.velocity_n = v[0]
        self.velocity_e = v[1]
        self.velocity_d = v[2]
        self.yaw = yaw

    # 编译即将发送的信息
    def trans_data(self):
        follower_desired_id = int(self.follower_desired_id)  # int 16
        follower_desired_cmd = int(self.follower_desired_cmd)  # unit_8
        follower_desired_type = int(self.follower_desired_type)  # unit_8
        current_loc_lat = int(self.current_loc_lat * 1e7)  # int_32
        current_loc_lng = int(self.current_loc_lng * 1e7)  # int_32
        posD = int(self.altitude * (-1000))  # int_32
        velocity_n = int(self.velocity_n * 100)  # int_16
        velocity_e = int(self.velocity_e * 100)  # int_16
        velocity_d = int(self.velocity_d * 100)  # int_16
        yaw = int(self.yaw * 100)  # int_16

        chk = 0
        buffer = bytearray()

        buffer.append(0xAA)
        buffer.append(0xAF)
        buffer.append(0)

        buffer.append((follower_desired_id & 0x00FF) >> 0)
        buffer.append((follower_desired_id & 0xFF00) >> 8)

        buffer.append(follower_desired_cmd)
        buffer.append(follower_desired_type)

        buffer.append((current_loc_lat & 0x000000FF) >> 0)
        buffer.append((current_loc_lat & 0x0000FF00) >> 8)
        buffer.append((current_loc_lat & 0x00FF0000) >> 16)
        buffer.append((current_loc_lat & 0xFF000000) >> 24)

        buffer.append((current_loc_lng & 0x000000FF) >> 0)
        buffer.append((current_loc_lng & 0x0000FF00) >> 8)
        buffer.append((current_loc_lng & 0x00FF0000) >> 16)
        buffer.append((current_loc_lng & 0xFF000000) >> 24)

        buffer.append((posD & 0x000000FF) >> 0)
        buffer.append((posD & 0x0000FF00) >> 8)
        buffer.append((posD & 0x00FF0000) >> 16)
        buffer.append((posD & 0xFF000000) >> 24)

        buffer.append((velocity_n & 0x000000FF) >> 0)
        buffer.append((velocity_n & 0x0000FF00) >> 8)

        buffer.append((velocity_e & 0x000000FF) >> 0)
        buffer.append((velocity_e & 0x0000FF00) >> 8)

        buffer.append((velocity_d & 0x000000FF) >> 0)
        buffer.append((velocity_d & 0x0000FF00) >> 8)

        buffer.append((yaw & 0x000000FF) >> 0)
        buffer.append((yaw & 0x0000FF00) >> 8)

        cnt = len(buffer)

        buffer[2] = cnt - 3

        for i in range(cnt):
            chk += buffer[i]

        # if chk > 0xFF:
        #     buffer.append((chk & 0xFF00) >> 8)
        buffer.append(chk & 0x00FF)

        # if (txspace < cnt):
        #     return

        return buffer

    def send_broadcast(self, port, data_to_send):
        print(" +-----------------------------------+")
        print(" | XBee Python Library Send Broadcast|")
        print(" +-----------------------------------+\n")

        ser = serial.Serial(port, BAUD_RATE)
        result = ser.write(data_to_send)
        print("已广播相关消息给从机飞控！！")


if __name__ == '__main__':
    id = 2
    cmd = cmd_outer.Follower_Action_Takeoff
    type = 1
    current_loc = [20, 5, 6]  # TODO: can not send float type
    v = [5, 5, 5]
    yaw = -10

    data2fk = Data2Fk()
    data2fk.update(id, cmd, type, current_loc=current_loc, v=v, yaw=yaw)
    data2send = data2fk.trans_data()
    print(data2send)
    ser = serial.Serial('com13', BAUD_RATE)
    # data2fk.send_broadcast(port='com13', data_to_send=data2send)

    while 1:
        result = ser.write(data2send)
        print('print')

    # a = unpack(">BBBhBBiiihhhhB", data2send)
    # print(a)
    a = ' '.join(hex(x) for x in data2fk.buffer)
    print('转换为hex的数据如下：\n')
    print(a)
