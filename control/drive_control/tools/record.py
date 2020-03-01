#!/usr/bin/env python
import rospy
import numpy as np
import atexit

from geometry_msgs.msg import TwistStamped
from adam_msgs.msg import VehicleCmd

ctrlData=[]
hallData=[]
saving = False

def ctrlCB(ctrlMsg):
    msgData = []
    timestamp = rospy.get_rostime()
    msgData.append(timestamp)
    msgData.append(ctrlMsg.ctrl_cmd.speed)
    ctrlData.append(msgData)
    print 'receive a ctrl msg'

def hallCB(hallMsg):
    msgData = []
    timestamp = rospy.get_rostime()
    msgData.append(timestamp)
    msgData.append(hallMsg.twist)
    hallData.append(msgData)
    print ' receive a hall msg'

def exit_handler():
    np.save("ctrlData", ctrlData)
    np.save("hallData", hallData)
    print 'save as ctrlData.py and hallData.py'


if __name__ == "__main__":
    rospy.init_node('recorder')
    rospy.Subscriber("/vehicle_cmd", VehicleCmd, ctrlCB)
    rospy.Subscriber("/wheel_circles", TwistStamped, hallCB)
    atexit.register(exit_handler)

    rospy.spin()



