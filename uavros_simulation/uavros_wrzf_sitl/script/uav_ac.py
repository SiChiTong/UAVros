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

from quadrotor.decision import DecisionMaker
from quadrotor.parameters import *
from quadrotor.uav import Uav
from quadrotor.data_distributor import DataDistributor
from quadrotor import tools as tls
import quadrotor.cmd as cmd
from cmd_with_outer_hardware import *
from send_to_fk import *

# TODO: 修改xbee对应的端口
PORT_CENTER = "/dev/ttyUSB1"
# "/dev/ttyUSB0"
PORT2FK = '/dev/ttyUSB2'
# use global variables to convey information
receive_flag = False
addr_buffer = None
data_buffer = None


def main():
    # ---- initialization ----
    # Uav类里包含了XBee的初始化，open()等操作
    uav_ac = Uav(port=PORT_CENTER, name=UAV_AC_NAME, uav_center_name=UAV_AC_NAME, current_position=[0, 0, 0])
    uav_ac.config_xbee(node_id=uav_ac.name)
    data_2_fk = Data2Fk()

    # decision maker related
    decision_maker = DecisionMaker('fixed,score=3', cars=[1, 3, 1], detection_save_num=5)
    # 3个上，2个侧，没有上下动: 'fixed,score=3'; 2个上，3个侧，没有上下动: fixed,score=3.25

    # 监听实例
    distributor = DataDistributor(uav_ac, data_2_fk, decision_maker=decision_maker)

    uav_ac.send_to_fk(WAIT)  # 强制刷新给飞控的指令是wait
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
    uav_ac.set_init_time()

    # # 起飞, 给所有飞机发送起飞命令
    uav_ac.fly()  # 给本机飞控发送起飞命令

    apm_height = 7  # TODO: 比赛时修改数据，apm从机高度

    # 只有主机解锁起飞，才能让从机起飞
    while True:
        try:
            state = uav_ac.read_state_from_fk()
            if state is False:
                continue
            # state = {"latitude": latitude, "longitude": longitude, "z": z, "vx": vx, "vy": vy, "vz": vz, "armed": armed,
            #                     "mode": mode, "yaw": yaw}

            # id, cmd, type, loc(3), v(3), yaw
            loc = [state['latitude'], state['longitude'], apm_height]
            v = [state['vx'], state['vy'], state['vz']]
            yaw = state['yaw']
        except:
            continue

        if state['armed'] is True and state['mode'] == 'AUTO.TAKEOFF':
            break
    data_2_fk.update(id=0, cmd=Follower_Action_Takeoff, type=TYPE_CLOSE, current_loc=loc, v=v, yaw=yaw)
    data_2_send = data_2_fk.trans_data()
    data_2_fk.send_broadcast(PORT2FK, data_to_send=data_2_send)
    
    # while True:  # 读取飞控状态
    #     data = uav_ac.read_from_fk()
    #     if data == ARRIVED:  # 如果已经到达目标位置
    #         print("uav_ac 已经到达!")
    #         # uav_ac.send_to_dyt(DYT_OPEN)  # 开启导引头的跟踪模式
    #         break

    print("开始往起始点飞行")
    while time.time() - uav_ac.initial_time <= 15:  # TODO: 修改时间，正式改为3min
        pass
    print("飞到！")

    uav_ac.mode = MODE_2
    print("切换到 MODE_2：跟踪模式")

    # ==================== Mode 2: 跟踪 =====================
    # which_car = decision_maker.uav_track_which_car
    # car_num = decision_maker.car_num_real

    update_action_flag = False
    # print('uav_ac保持升高状态！！！')
    car_old = 3
    formation_type = TYPE_CLOSE
    # TODO:可能调整初始队形（现在是接近）
    print("现在从机是接近状态")

    apm_height = 4  # TODO: 根据数字识别情况进行修改
    while time.time() - uav_ac.initial_time <= 30:  # TODO: 正式要改为6min15s

        # 一直发送编队信息
        try:
            state = uav_ac.read_state_from_fk()
            if state is False:
                continue
            # state = {"latitude": latitude, "longitude": longitude, "z": z, "vx": vx, "vy": vy, "vz": vz, "armed": armed,
            #                     "mode": mode, "yaw": yaw}

            # id, cmd, type, loc(3), v(3), yaw
            loc = [state['latitude'], state['longitude'], apm_height]
            v = [state['vx'], state['vy'], state['vz']]
            yaw = state['yaw']
        except:
            continue

        data_2_fk.update(id=0, cmd=Follower_Action_Takeoff, type=formation_type, current_loc=loc, v=v, yaw=yaw)
        data_2_send = data_2_fk.trans_data()
        data_2_fk.send_broadcast(PORT2FK, data_to_send=data_2_send)

        # 给uav_ss_sm飞控发送相关信息
        if decision_maker.new_detection_flag:  # 接收到新的检测信息
            decision_maker.new_detection_flag = False  # 关闭flag

            # 判断是否相同，且与之前的实际数据是否匹配.如果相同，且与之前数据不同，则更新。
            # car_old = decision_maker.car_num_real[1]
            print(decision_maker.real_num)
            if car_old == 0 and decision_maker.real_num != 0:
                update_action_flag = 1  # 接近
            elif car_old != 0 and decision_maker.real_num == 0:
                update_action_flag = 2  # 远离
            else:
                update_action_flag = 0  # 不改变

            car_old = decision_maker.real_num

        if update_action_flag:
            if update_action_flag == 1:
                formation_type = TYPE_CLOSE
                print("发送接近接近接近动作给飞控！！！")
                print("切换状态")
            if update_action_flag == 2:
                formation_type = TYPE_FAR
                print("发送远离远离远离动作给飞控！！！")
                print("切换状态")
            update_action_flag = 0

    # 告诉本机飞控，该返航了
    uav_ac.send_to_fk(RETURN)
    uav_ac.mode = MODE_3

    # 告诉从机该返航了
    state = uav_ac.read_state_from_fk()
    # state = {"latitude": latitude, "longitude": longitude, "z": z, "vx": vx, "vy": vy, "vz": vz, "armed": armed,
    #                     "mode": mode, "yaw": yaw}

    apm_height = 7  # TODO: 设置返航高度
    # id, cmd, type, loc(3), v(3), yaw
    loc = [state['latitude'], state['longitude'], apm_height]
    v = [state['vx'], state['vy'], state['vz']]
    yaw = state['yaw']

    data_2_fk.update(id=0, cmd=Follower_Action_RTL, type=formation_type, current_loc=loc, v=v, yaw=yaw)
    data_2_send = data_2_fk.trans_data()
    data_2_fk.send_broadcast(PORT2FK, data_to_send=data_2_send)

    print("切换到 MODE_3：返航模式")
    # ==================== Mode 3: 返航 =====================
    uav_ac.ret()
    uav_ac.send_data_to_one(GROUND_STATION_NAME, cmd.RETURN)


if __name__ == "__main__":
    main()
