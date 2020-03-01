#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import time

from geometry_msgs.msg import TwistStamped
from adam_msgs.msg import VehicleCmd
from std_msgs.msg import Bool

ctrlData=[]
hallData=[]

ctrlTime = []
hallTime = []
execFlag = True
# def show():
#     plt.plot(ctrlTime, ctrlData, label="ctrl")
#     plt.plot(hallTime, hallData, label="hall")
#     plt.legend(labels=['ctrl', 'hall'], loc='best')
#     plt.show()
    
def plot_durations():
    # plt.figure(1)
    plt.clf()
    plt.plot(ctrlTime, ctrlData)
    plt.plot(hallTime, hallData)
    plt.pause(0.01)

def ctrlCB(ctrlMsg):
    timestamp = rospy.get_rostime()
    ctrlTime.append(timestamp.secs % 100 * 1000 + timestamp.nsecs / 1000000.0)
    ctrlData.append(ctrlMsg.ctrl_cmd.speed)
    print 'receive a ctrl msg'
    # plot()
    # show()

def hallCB(hallMsg):
    timestamp = rospy.get_rostime()
    hallTime.append(timestamp.secs % 100 * 1000 + timestamp.nsecs / 1000000.0)
    hallData.append(hallMsg.twist.linear.x)
    print ' receive a hall msg'
    # show()

def viewCB(viewMsg):
    print 'viewCB'
    stopFlag = viewMsg.data
    exit()
    # plot = viewMsg.data

if __name__ == "__main__":
    rospy.init_node('recorder')
    rospy.Subscriber("/vehicle_cmd", VehicleCmd, ctrlCB)
    rospy.Subscriber("/wheel_circles", TwistStamped, hallCB)
    rospy.Subscriber("/view_ctrl", Bool, viewCB)

    # plt.ion()
    while execFlag:
        plot_durations()
    print 'receive exit cmd'
            # exit()
    rospy.spin()



