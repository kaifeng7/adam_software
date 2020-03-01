import rospy
import socket
import sys
from std_msgs.msg import Float64

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip_addr = ''

def localization_CB(locMsg):
    sendmsg = '{localization:' + str(locMsg.data) + '}'
    s.sendto(sendmsg, (ip_addr, 11113))


if __name__ == '__main__':
    rospy.init_node('status_monitor')
    ip_addr = sys.argv[1]
    #global address = (sys.argv[1], 11113)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rospy.Subscriber("/ndt/ndt_score", Float64, localization_CB)
    rospy.spin()
