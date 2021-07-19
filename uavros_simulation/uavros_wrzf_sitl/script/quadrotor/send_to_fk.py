# -*- coding: utf-8 -*-
# author: li xu heng
# time: 2021/717 11:22
# project_name: NobodyIsFighting
# file_name: send_to_fk
# function: 向飞控发送指令

import time
from digi.xbee.devices import XBeeDevice

# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 115200


class Data2fk:
    follower_desired_id = 0  # int 16
    follower_desired_cmd = 0  # unit_8
    follower_desired_type = 0  # unit_8
    current_loc_lat = 0  # int_32
    current_loc_lng = 0  # int_32
    altitude = 0  # int_32
    velocity_n = 0  # int_16
    velocity_e = 0  # int_16
    velocity_d = 0  # int_16
    yaw = 0  # int_16

    def __init__(self, follower=[0, 0, 0], current_loc=[0, 0, 0], v=[0, 0, 0], yaw=0):
        self.follower_desired_id = follower[0]
        self.follower_desired_cmd = follower[1]
        self.follower_desired_type = follower[2]
        self.current_loc_lat = current_loc[0]
        self.current_loc_lng = current_loc[1]
        self.altitude = current_loc[2]
        self.velocity_n = v[0]
        self.velocity_e = v[1]
        self.velocity_d = v[2]
        self.yaw = yaw




# 编译即将发送的信息
def trans_data(Data2fk):
    follower_desired_id = Data2fk.follower_desired_id  # int 16
    follower_desired_cmd = Data2fk.follower_desired_cmd  # unit_8
    follower_desired_type = Data2fk.follower_desired_type  # unit_8
    current_loc_lat = Data2fk.current_loc_lat  # int_32
    current_loc_lng = Data2fk.current_loc_lng  # int_32
    posD = Data2fk.altitude  # int_32
    velocity_n = Data2fk.velocity_n  # int_16
    velocity_e = Data2fk.velocity_e  # int_16
    velocity_d = Data2fk.velocity_d  # int_16
    yaw = Data2fk.yaw  # int_16

    chk = 0
    buffer = bytearray()

    buffer.append(0xAA)
    buffer.append(0xAF)
    buffer.append(0)

    buffer.append((follower_desired_id & 0xFF00) >> 8)
    buffer.append((follower_desired_id & 0x00FF) >> 0)

    buffer.append(follower_desired_cmd)
    buffer.append(follower_desired_type)

    buffer.append((current_loc_lat & 0xFF000000) >> 24)
    buffer.append((current_loc_lat & 0x00FF0000) >> 16)
    buffer.append((current_loc_lat & 0x0000FF00) >> 8)
    buffer.append((current_loc_lat & 0x000000FF) >> 0)

    buffer.append((current_loc_lng & 0xFF000000) >> 24)
    buffer.append((current_loc_lng & 0x00FF0000) >> 16)
    buffer.append((current_loc_lng & 0x0000FF00) >> 8)
    buffer.append((current_loc_lng & 0x000000FF) >> 0)

    buffer.append(((posD * (-1000)) & 0xFF000000) >> 24)
    buffer.append(((posD * (-1000)) & 0x00FF0000) >> 16)
    buffer.append(((posD * (-1000)) & 0x0000FF00) >> 8)
    buffer.append(((posD * (-1000)) & 0x000000FF) >> 0)

    buffer.append((velocity_n * 100 & 0x0000FF00) >> 8)
    buffer.append((velocity_n * 100 & 0x000000FF) >> 0)

    buffer.append((velocity_e * 100 & 0x0000FF00) >> 8)
    buffer.append((velocity_e * 100 & 0x000000FF) >> 0)

    buffer.append((velocity_d * 100 & 0x0000FF00) >> 8)
    buffer.append((velocity_d * 100 & 0x000000FF) >> 0)

    buffer.append((yaw * 100 & 0x0000FF00) >> 8)
    buffer.append((yaw * 100 & 0x000000FF) >> 0)

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


def send_to_fk(DATA_TO_SEND, REMOTE_NODE_ID, PORT):

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        # Obtain the remote XBee device from the XBee network.
        xbee_network = device.get_network()
        remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
        if remote_device is None:
            print("Could not find the remote device")
            exit(1)

        print("Sending data to %s >> %s..." % (remote_device.get_64bit_addr(), DATA_TO_SEND))

        device.send_data(remote_device, DATA_TO_SEND)

        print("Success")

    finally:
        if device is not None and device.is_open():
            device.close()


if __name__ == '__main__':
    follower = [2, 5, 1]
    current_loc = [20.1, 5.2, 6]
    v = [0.5, 0.5, 0.5]
    yaw = -10

    Data2fk(follower=follower, current_loc=current_loc, v=v, yaw=yaw)
    data2send = trans_data(Data2fk)
    print(data2send)
    # a = unpack(">BBBhBBiiihhhhB", data2send)
    # print(a)
    a = ' '.join(hex(x) for x in data2send)
    print('转换为hex的数据如下：\n')
    print(a)
    # while 1:
    #     time.sleep(0.1)
    send_to_fk(DATA_TO_SEND=data2send, REMOTE_NODE_ID='fk', PORT='com11')
