# -*- coding: utf-8 -*-
# author: li xu heng
# time: 2021/7/6 16:36
# project_name: NobodyIsFighting
# file_name: uav_parent
# function: define the parent of uav

import json
from digi.xbee.util import utils
from digi.xbee.devices import DigiMeshDevice

import quadrotor.archives.state_com.para_com as para
import quadrotor.archives.state_com.state_com.cmd_com as cmd


class StateCom:

    def __init__(self, port, name, center_name, baud_rate=para.BAUD_RATE):
        # 基本属性
        self.name = name
        self.center = center_name

        # XBee 相关
        self.port = port
        self.xbee = DigiMeshDevice(port, baud_rate)
        self.xbee.open()

    def __del__(self):
        self.xbee.close()

    def config_xbee(self, node_id):
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
        pass

    def send_data_to_all(self, command, position, velocity):
        while True:
            try:
                data_dict = {cmd.KEY_NODE: self.name, cmd.KEY_CMD: command,
                             cmd.KEY_POSITION: position, cmd.KEY_VEL: velocity}

                data_to_broadcast = json.dumps(data_dict)  # convert to JSON format

                print("Sending broadcast data: %s..." % data_to_broadcast)

                self.xbee.send_data_broadcast(data_to_broadcast)

                print("Success")

                break

            except:
                print("Something error when sending broadcast data, retrying......")

    def send_data_to_one(self, remote_node_id, command, position, velocity):
        while True:
            try:
                data_dict = {cmd.KEY_NODE: self.name, cmd.KEY_CMD: command,
                             cmd.KEY_POSITION: position, cmd.KEY_VEL: velocity}

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

    def write_state(self, file_name, str_data):
        with open(file_name, 'w') as f:
            f.write(str_data)
            return True

        return False

    def read_state(self, file_name):
        with open(file_name, 'r') as f:
            data = f.read()
            return data

        return False
