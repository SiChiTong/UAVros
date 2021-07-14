import can
from STE5proto import *
import rospy
import threading
time_out = 0.1 

bus = can.interface.Bus('can0', bustype='socketcan')
bus_reader = can.BufferedReader()
bus_notifier = can.Notifier(bus,[bus_reader])
SData = SeekerData()

rospy.init_node('rospy_node',anonymous=True)
rate = rospy.Rate(20)
state_topic_name = "/gimbal/gimbal_state"
cmd_topic_name = "/gimbal/gimbal_cmd"

def pack_state_msg(SData):
    msg = TrackState()
    msg.pitch = SData.pitch
    msg.yaw = SData.yaw
    msg.servo_state = SData.servo_state
    msg.track_state = SData.track_state
    return msg

def cmd_callback(msg):
    if msg.data == "OPEN":
        on_msg = TrackingOnMsg()
        bus.send(on_msg)
    elif msg.data == "CLOSE":
        off_msg = TrackingOffMsg()
        bus.send(off_msg)

def pub_

state_puber = rospy.Publisher(state_topic_name,TrackState,queue_size=1)
cmd_suber = rospy.Subscriber(cmd_topic_name,String,cmd_callback)

try:
    while not rospy.is_shutdown():
        msg = bus_reader.get_message(time_out)
        if msg is None:
            break
        DataParse(msg, SData)
        print(SData)
except KeyboardInterrupt:
    print("Program Exited")

