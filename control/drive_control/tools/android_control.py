
import rospy
import socket
import re

from adam_msgs.msg import VehicleCmd

maxV = 3.
minV = 0.
maxW = 0.60
minW = -0.60
pub_cmd_vel = rospy.Publisher("/vehicle_cmd",VehicleCmd, queue_size=5)

def getVec(vPersent = 0.):
    return minV + (maxV - minV) * vPersent
 #   return 0;

def getAngle(wPersent = 0.):
    wPersent = - wPersent
    midW = (maxW + minW) * 0.5
    if wPersent >= 0:
        return midW + (maxW - midW) * wPersent
    else:
        return midW + (midW - minW) * wPersent

def parse(msg):
    cmd_list=[]
    reg = r'wheel=(-?\d*.\d*) speed=(\d*.\d*)'
    matchObj = re.match(reg, msg);
    #print('msg:'+msg)
    #print(float(matchObj.group(1)))
    
    cmd_list.append(getAngle(float(matchObj.group(1))))
    cmd_list.append(getVec(float(matchObj.group(2))))
    return cmd_list

def pub_cmd(cmd_list=[]):
    t = VehicleCmd()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = '/base_link'
    t.ctrl_cmd.speed = cmd_list[1]
    t.ctrl_cmd.steering_angle = cmd_list[0]
    if cmd_list[1] > 0.0001:
        t.mode = 1;
    elif cmd_list[1] < -0.0001:
        t.mode = 0;
    else:
        t.mode = 2;
    pub_cmd_vel.publish(t)



def udp_server(host='', port=11112):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    while True:
        (data, addr) = s.recvfrom(1024*8)
        #print(addr)
        #print('addr='+addr[0]+':'+str(addr[1])+'\n')
        msg = data.decode('utf-8')
        cmd_list = parse(msg)
        pub_cmd(cmd_list)



if __name__ == '__main__':
    rospy.init_node('android_control')
    udp_server()
