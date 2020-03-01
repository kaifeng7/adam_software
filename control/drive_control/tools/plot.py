#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt

from adam_msgs.msg import VehicleCmd
from geometry_msgs.msg import TwistStamped

if __name__ == "__main__":
    ctrlData = np.load("/Users/soyx/catkin_ws/src/adam_software/control_0725/drive_control/tools/ctrlData.npy")
    hallData = np.load("/Users/soyx/catkin_ws/src/adam_software/control_0725/drive_control/tools/hallData.npy")
    ctrlTime = []
    hallTime = []
    ctrlValue =[]
    hallValue = []
    for element in ctrlData:
        ctrlTime.append(element[0].secs % 100 * 1000 + element[0].nsecs / 1000000.0)
        ctrlValue.append(element[1])
    for element in hallData:
        hallTime.append(element[0].secs % 100 * 1000 + element[0].nsecs / 1000000.0)
        hallValue.append(element[1].linear.x)

    plt.figure(1)
    plt.plot(ctrlTime, ctrlValue, label="ctrl")
    plt.plot(hallTime, hallValue, label="hall")

    plt.legend(labels=['ctrl', 'hall'], loc='best')

    plt.show()