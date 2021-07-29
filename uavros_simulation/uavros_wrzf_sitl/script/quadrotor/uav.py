# -*- coding: utf-8 -*-
# author: li xu heng
# time: 2021/7/6 16:36
# project_name: NobodyIsFighting
# file_name: uav_parent
# function: define the parent of uav
import time

import json
import quadrotor.parameters as para
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.util import utils
from digi.xbee.devices import DigiMeshDevice
import quadrotor.cmd as cmd
from cmd_with_outer_hardware import *
import random


class Uav:
    mode = 0  # 0,1,2,3
    time = 0
    name = ''
    uav_center = ''  # 中心节点uav的信息
    ground_station = ''  # 地面站
    my_position = [0, 0, 0]  # 位置信息
    initial_time = 0  # 系统初始时间，用来计算相对时间

    def __init__(self, port, name, uav_center_name, current_position, baud_rate=para.BAUD_RATE, action=cmd.GO_UP):
        # 基本属性
        self.name = name
        self.uav_center = uav_center_name
        self.ground_station = para.GROUND_STATION_NAME
        self.uav_center_position = [0, 0, 0]
        self.my_position = current_position
        self.initial_time = None

        # mode 1: 起飞
        self.target_pos = [0, 0, 0]

        # mode 2: 跟踪
        self.action = cmd.GO_UP

        # XBee 相关
        self.port = port
        self.xbee = DigiMeshDevice(port, baud_rate)
        self.xbee.open()

    def __del__(self):
        self.xbee.close()

    def config_xbee(self, node_id):
        # TODO: 对xbee进行设置，主要是node id，通讯模式等等
        # Set parameters.
        self.xbee.set_parameter(para.PARAM_NODE_ID, bytearray(node_id, 'utf8'))
        self.xbee.set_parameter(para.PARAM_PAN_ID, para.PARAM_VALUE_PAN_ID)
        self.xbee.set_parameter(para.PARAM_DEST_ADDRESS_H, para.PARAM_VALUE_DEST_ADDRESS_H)
        self.xbee.set_parameter(para.PARAM_DEST_ADDRESS_L, para.PARAM_VALUE_DEST_ADDRESS_L)

        # Get parameters.
        print("Node ID:                     %s" % self.xbee.get_parameter(para.PARAM_NODE_ID).decode())
        print("PAN ID:                      %s" % utils.hex_to_string(self.xbee.get_parameter(para.PARAM_PAN_ID)))
        print(
            "Destination address high:    %s" % utils.hex_to_string(self.xbee.get_parameter(para.PARAM_DEST_ADDRESS_H)))
        print(
            "Destination address low:     %s" % utils.hex_to_string(self.xbee.get_parameter(para.PARAM_DEST_ADDRESS_L)))

        print("")
        print("All parameters were set correctly!")
        # self.xbee.close()
        # self.xbee.open()

    def send_data_to_all(self, command, position=None):
        while True:
            try:
                data_dict = {cmd.KEY_NODE: self.name, cmd.KEY_CMD: command}
                if position is not None:
                    data_dict[cmd.KEY_POSITION] = position

                data_to_broadcast = json.dumps(data_dict)  # convert to JSON format

                print("Sending broadcast data: %s..." % data_to_broadcast)

                self.xbee.send_data_broadcast(data_to_broadcast)

                print("Success")

                break

            except:
                print("Something error when sending broadcast data, retrying......")

    def send_data_to_one(self, remote_node_id, command, position=None):
        while True:
            try:
                data_dict = {cmd.KEY_NODE: self.name, cmd.KEY_CMD: command}
                if position is not None:
                    data_dict[cmd.KEY_POSITION] = position

                data_to_send = json.dumps(data_dict)  # convert to JSON format

                # Obtain the remote XBee device from the XBee network.
                xbee_network = self.xbee.get_network()
                remote_device = xbee_network.discover_device(remote_node_id)
                if remote_device is None:
                    print("Could not find the remote device")
                    exit(1)

                print("Sending data to %s >> %s..." % (remote_device.get_node_id(), data_to_send))

                self.xbee.send_data(remote_device, data_to_send)

                print("Success")

                break

            except:
                print("Something error when sending data to node " + remote_node_id + ", retrying......")

    def listening(self, data_receive_callback):
        # data_receive_callback 是一种回调函数
        self.xbee.add_data_received_callback(data_receive_callback)
        print("Waiting for data...\n")

    def detect(self):
        result = []
        result.append([11, 12, 0])  # 检测到的第一辆车的坐标
        result.append([12, 13, 0])  # 检测到的第二辆车的坐标
        result.append([13, 14, 0])  # 检测到的第三辆车的坐标
        return result

    def update_position(self):
        """
        更新无人机自身的位置
        """
        self.my_position = [random.randint(0, 9), random.randint(0, 9), random.randint(0, 9)]

    def set_init_time(self):
        self.initial_time = time.time()

    def init_plane(self):
        """
        向导引头发送指令：
        关闭导引头跟踪功能，归中.
        """
        print("初始化 %s!" % self.name)

        # self.send_to_dyt(DYT_SHUT_DOWN)

        print(self.name + "初始化成功")
        return True

    # def send_to_dyt(self, str_data):
    #     with open(TO_DYT_F, 'w') as f:
    #         f.write(str_data)
    #         return True
    #
    #     return False
    #
    # def read_from_dyt(self):
    #     with open(FROM_DYT_F, 'r') as f:
    #         data = f.read()
    #         return data
    #
    #     return False

    def send_to_fk(self, str_data):
        with open(TO_FK_F, 'w') as f:
            f.write(str_data)
            return True

        return False

    def read_state_from_fk(self):
        with open(FROM_FK_STATE_F, 'r') as f:
            data = f.read()
            if data is None:
                return None
            return json.loads(data)

        return False

    def read_from_file(self, filename):
        with open(filename, 'r') as f:
            data = f.read()
            return data

        return False

    def fly(self):
        """
        给飞控发送 起飞 命令
        """
        if self.mode != para.MODE_1:
            raise ValueError('尚未到起飞模式')

        self.initial_time = time.time()

        with open(TO_FK_F, 'w') as f:
            f.write(FLY)

        print(self.name + ' 飞起来啦！')

        # while True:
        #     self.move(position)
        #     if self.position == position:
        #         break
        # info = '到达'
        # self.transfer_info(info, self.uav_center)
        # while self.time <= 6:  # 调整为180
        #     self.time = time.time() - self.initial_time
        #     print('准备完毕，时间：%0.2f' % self.time)
        #     time.sleep(2)
        # self.transaction_mode(2)

    def track(self, position):
        print("%s is tracking [%f, %f, %f]" % (self.name, position[0], position[1], position[2]))

    def ret(self):
        if self.mode != para.MODE_3:
            raise ValueError("模式错误")

        print(self.name + '开始返航')

# if __name__ == '__main__':
#     uav = Uav(name='uav', uav_ac='uav_ac', current_position='init position')
#     uav.init_plane()
#     uav.fly(position='game area')
#     uav.track()
#     uav.ret()
