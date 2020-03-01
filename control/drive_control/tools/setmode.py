import rospy
import sys

if __name__ == '__main__':
    if sys.argv[1] == 'human':
        rospy.set_param('/drive_control_node/human_drive', True)
    else:
        rospy.set_param('/drive_control_node/human_drive', False)
