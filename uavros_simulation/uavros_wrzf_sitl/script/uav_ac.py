#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
Author: LI Jinjie
File: decision_demo.py
Date: 2021/5/26 11:06
LastEditors: LI Jinjie
LastEditTime: 2021/5/26 11:06
Description: a center node
'''
import time

from quadrotor.decision import DecisionMaker
import json
from quadrotor.parameters import *
from quadrotor.uav import Uav
from quadrotor.data_distributor import DataDistributor
from quadrotor import tools as tls
import quadrotor.cmd as cmd
from quadrotor.send_to_fk import *

from cmd_with_outer_hardware import *

# TODO: 修改xbee对应的端口
PORT_CENTER = "/dev/ttyUSB0"
# "/dev/ttyUSB0"
# PORT2FK = 'com3'
# use global variables to convey information
receive_flag = False
addr_buffer = None
data_buffer = None


def main():
    # ---- initialization ----
    # Uav类里包含了XBee的初始化，open()等操作
    uav_ac = Uav(port=PORT_CENTER, name=UAV_AC_NAME, uav_center_name=UAV_AC_NAME, current_position=[0, 0, 0])
    uav_ac.config_xbee(node_id=uav_ac.name)

    # decision maker related
    decision_maker = DecisionMaker('fixed,score=3', cars=[1, 3, 1], detection_save_num=5)
    # 3个上，2个侧，没有上下动: 'fixed,score=3'; 2个上，3个侧，没有上下动: fixed,score=3.25

    # 监听实例
    distributor = DataDistributor(uav_ac, decision_maker)

    # 确认是wait状态
    uav_ac.send_to_fk(WAIT)

    # ==================== Mode 0: 准备 =====================
    uav_ac.mode = MODE_0  # preparation
    # 发起组网
    nodes = tls.find_devices(device=uav_ac.xbee, finding_time=10)
    if nodes is None:
        print("未发现设备, 退出。")
        return 0
    print("组网完毕，现在可以启动地面站和其他飞机的程序了！")

    # 判断所有飞机是否初始化成功
    decision_maker.reset_ready_dict(mode=uav_ac.mode)
    if uav_ac.init_plane() is True:
        decision_maker.ready_dict[UAV_AC_NAME] = True
        # print("uav_ac is ready!")

    # 开始监听: 回调函数 + 命令 + 阻塞
    uav_ac.listening(data_receive_callback=distributor.callback)

    distributor.break_flag = False  # 开始前重置break_flag
    while True:  # 监听必须搭配阻塞
        if decision_maker.all_ready():  # 全部成功，自动重置
            time.sleep(1)
            uav_ac.send_data_to_one(remote_node_id=GROUND_STATION_NAME, command=cmd.ALL_SUCCESS)
            print("全部飞机都准备好了!!!")
            break

    distributor.break_flag = False  # 开始前重置break_flag
    while True:
        time.sleep(1)
        if distributor.break_flag is True:
            break

    # input("输入任意键以进行起飞\r\n")  # 等待发送起飞指令
    # print("收到指令，起飞！")
    #
    # # ==================== Mode 1: 起飞 =====================
    # print("切换到 MODE_1：起飞模式")
    # uav_ac.mode = MODE_1
    # uav_ac.send_data_to_all(command=cmd.TO_MODE_1)
    uav_ac.set_init_time()

    # # 起飞, 给所有飞机发送起飞命令
    # uav_ac.send_data_to_all(command=cmd.FLY)
    uav_ac.fly()  # 给飞控发送起飞命令

    # while True:  # 读取飞控状态
    #     data = uav_ac.read_from_fk()
    #     if data == ARRIVED:  # 如果已经到达目标位置
    #         print("uav_ac 已经到达!")
    #         # uav_ac.send_to_dyt(DYT_OPEN)  # 开启导引头的跟踪模式
    #         break

    while time.time() - uav_ac.initial_time <= 15:  # TODO: change to 170 seconds when executed
        pass  # wait for reaching the target

    uav_ac.mode = MODE_2
    print("飞行时间到达，切换到 MODE_2：跟踪模式")

    # ==================== Mode 2: 跟踪 =====================
    which_car = decision_maker.uav_track_which_car
    car_num = decision_maker.car_num_real

    update_action_flag = False
    print('uav_ac保持升高状态！！！')
    while time.time() - uav_ac.initial_time <= 30:  # TODO: 正式要改为6min15s
        # 给uav_ss_sm飞控发送相关信息
        if decision_maker.new_detection_flag:  # 接收到新的检测信息
            decision_maker.new_detection_flag = False  # 关闭flag

            car_1 = decision_maker.car_num_detections[1]
            # 判断5次检测是否相同，且与之前的实际数据是否匹配.如果检测5次相同，且与之前数据不同，则更新。
            car_old = decision_maker.car_num_real[1]
            if car_1.count(car_1[0]) == len(car_1) and car_1[0] != decision_maker.car_num_real[1]:
                decision_maker.car_num_real[1] = car_1[0]
                print("确认car_1的数字是" + str(car_1[0]))
                if car_old == 0 and decision_maker.car_num_real[1] != 0:
                    update_action_flag = True
                if car_old != 0 and decision_maker.car_num_real[1] == 0:
                    update_action_flag = True

        if update_action_flag:
            update_action_flag = False
            data2fk = Data2fk()
            # send_to_fk(DATA_TO_SEND=data2fk, REMOTE_NODE_ID=FK, PORT=PORT2FK)
            print("发送相应动作给飞控！！！")


        # print("等待两侧飞机到达动作位置")
        # decision_maker.reset_ready_dict(mode=uav_ac.mode)
        # while True:  # 监听必须搭配阻塞
        #     if decision_maker.all_ready():  # 全部成功，自动重置
        #         print("全部飞机都到达动作位置!!!重新开启检测模式!!!")
        #         break

    # TODO: send_to_fk(DATA_TO_SEND=data2fk, REMOTE_NODE_ID=FK, PORT=PORT2FK) 应该是广播
    # uav_ac.send_data_to_all(command=cmd.TO_MODE_3)
    # 告诉本机飞控，该返航了
    uav_ac.send_to_fk(RETURN)
    uav_ac.mode = MODE_3

    print("切换到 MODE_3：返航模式")
    # ==================== Mode 3: 返航 =====================
    uav_ac.ret()
    uav_ac.send_data_to_one(GROUND_STATION_NAME, cmd.RETURN)


if __name__ == "__main__":
    main()
