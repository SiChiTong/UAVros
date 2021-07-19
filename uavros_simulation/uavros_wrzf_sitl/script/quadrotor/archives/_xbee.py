# lxh 2021/07/02
# nobody is fighting
# content: 只被Uav()类调用，不被外部调用
#

import time
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.devices import XBeeDevice
from digi.xbee.util import utils

# TODO: Replace with the serial port where your local module is connected to.
# PORT_GROUND_STATION = "COM8"
# PORT_UAV_5 = "COM4"
# PORT_UAV_4 = "COM3"
# PORT_UAV_3 = "COM4"
# PORT_UAV_2 = "COM4"
# PORT_UAV_1 = "COM4"

# TODO: Replace with the baud rate of your local module.
BAUD_RATE = 57600

# Set_And_Get_Parameters to be set
PARAM_NODE_ID = "NI"
PARAM_PAN_ID = "ID"
PARAM_DEST_ADDRESS_H = "DH"
PARAM_DEST_ADDRESS_L = "DL"

# PARAM_VALUE_NODE_ID = "GROUND_STATION"
PARAM_VALUE_PAN_ID = utils.hex_string_to_bytes("7FFF")
PARAM_VALUE_DEST_ADDRESS_H = utils.hex_string_to_bytes("00")
PARAM_VALUE_DEST_ADDRESS_L = utils.hex_string_to_bytes("FFFF")


# =============== Configuration ====================
def Set_And_Get_Parameters(PORT, PARAM_VALUE_NODE_ID):
    print(" +----------------------------------------+")
    print(" | XBee Python Library Set/Get parameters |")
    print(" +----------------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        # Set parameters.
        device.set_parameter(PARAM_NODE_ID, bytearray(PARAM_VALUE_NODE_ID, 'utf8'))
        device.set_parameter(PARAM_PAN_ID, PARAM_VALUE_PAN_ID)
        device.set_parameter(PARAM_DEST_ADDRESS_H, PARAM_VALUE_DEST_ADDRESS_H)
        device.set_parameter(PARAM_DEST_ADDRESS_L, PARAM_VALUE_DEST_ADDRESS_L)

        # Get parameters.
        print("Node ID:                     %s" % device.get_parameter(PARAM_NODE_ID).decode())
        print("PAN ID:                      %s" % utils.hex_to_string(device.get_parameter(PARAM_PAN_ID)))
        print("Destination address high:    %s" % utils.hex_to_string(device.get_parameter(PARAM_DEST_ADDRESS_H)))
        print("Destination address low:     %s" % utils.hex_to_string(device.get_parameter(PARAM_DEST_ADDRESS_L)))

        print("")
        print("All parameters were set correctly!")

    finally:
        if device is not None and device.is_open():
            device.close()


def Reset_Module(PORT):
    print(" +----------------------------------+")
    print(" | XBee Python Library Reset Module |")
    print(" +----------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        device.reset()

        print("XBee module reset successfully")

    finally:
        if device is not None and device.is_open():
            device.close()

# ============= Form a network ================
def Discover_Devices(PORT):
    print(" +--------------------------------------+")
    print(" | XBee Python Library Discover Devices |")
    print(" +--------------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        xbee_network = device.get_network()

        xbee_network.set_discovery_timeout(15)  # 15 seconds.

        xbee_network.clear()

        # Callback for discovered devices.
        def callback_device_discovered(remote):
            print("Device discovered: %s" % remote)

        # Callback for discovery finished.
        def callback_discovery_finished(status):
            if status == NetworkDiscoveryStatus.SUCCESS:
                print("Discovery process finished successfully.")
            else:
                print("There was an error discovering devices: %s" % status.description)

        xbee_network.add_device_discovered_callback(callback_device_discovered)

        xbee_network.add_discovery_process_finished_callback(callback_discovery_finished)

        xbee_network.start_discovery_process()

        print("Discovering remote XBee devices...")

        while xbee_network.is_discovery_running():
            time.sleep(0.1)

    finally:
        if device is not None and device.is_open():
            device.close()

# ================= Data transmission ===============
def Send_Data(PORT, REMOTE_NODE_ID, DATA_TO_SEND):
    print(" +-------------------------------+")
    print(" | XBee Python Library Send Data |")
    print(" +-------------------------------+\n")

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


def Send_Broadcast_Data(PORT, DATA_TO_SEND):
    print(" +-----------------------------------------+")
    print(" | XBee Python Library Send Broadcast Data |")
    print(" +-----------------------------------------+\n")

    try:
        print("Sending broadcast data: %s..." % DATA_TO_SEND)

        device.send_data_broadcast(DATA_TO_SEND)

        print("Success")

    finally:
        if device is not None and device.is_open():
            device.close()


def Receive_Data(PORT):
    print(" +----------------------------------+")
    print(" | XBee Python Library Receive Data |")
    print(" +----------------------------------+\n")

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()

        def data_receive_callback(xbee_message):
            print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                                     xbee_message.data.decode()))

        device.add_data_received_callback(data_receive_callback)

        print("Waiting for data...\n")
        input()

    finally:
        if device is not None and device.is_open():
            device.close()
