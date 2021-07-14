import can
from STE5proto import *
import rospy
from GimbalMsgTest.msg import trackingState
from std_msgs.msg import String

time_out = 0.1 

bus = can.interface.Bus('can0', bustype='socketcan')
bus_reader = can.BufferedReader()
bus_notifier = can.Notifier(bus,[bus_reader])
SData = SeekerData()

rospy.init_node('rospy_node',anonymous=True)
rate = rospy.Rate(20)
state_topic_name = "/gimbal/gimbal_state"
cmd_topic_name = "/gimbal/gimbal_cmd"

state_puber = rospy.Publisher(state_topic_name,trackingState,queue_size=1)
cmd_suber = rospy.Subscriber(cmd_topic_name,String,cmd_callback)

def pack_state_msg(SData):
    msg = trackingState()
    trackingState.pitch = SData.pitch
    trackingState.yaw = SData.yaw
    trackingState.servoState = SData.servo_state
    trackingState.trackState = SData.track_state
    return msg

def cmd_callback(msg):
    if msg.data == "OPEN":
        on_msg = TrackingOnMsg
        bus.send(on_msg)
    elif msg.data == "CLOSE":
        off_msg = TrackingOnMsg
        bus.send(off_msg)


try:
    while not rospy.is_shutdown():
        can_msg = bus_reader.get_message(time_out)
        if can_msg is None:
            break
        DataParse(can_msg, SData)
        print(SData)
        msg_state = pack_state_msg(SData)
        state_puber.publish(msg_state)
        rate.sleep()


except KeyboardInterrupt:
    print("Program Exited")

