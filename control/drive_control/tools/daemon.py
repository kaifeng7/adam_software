#!/usr/local/bin/ python3

from socket import *
import subprocess
import os

import datetime

# trajectories_dir_path = '/home/nvidia/workspace/catkin_ws/src/ddl_0725/ndt_localization/trajectories'

trajectories_dir_path = '/Users/soyx/catkin_ws/src/adam_software/localization_0725/pf_localization/trajectories'



cmd_dict = {'roscore': 'roscore',
       'sensor': 'roslaunch common sensor.launch',
       'drive_control': 'roslaunch drive_controller drive_control.launch',
       'self_drive': 'roslaunch common test_0123.launch',
       'kill_all': 'rosnode kill -a',
       'android_control': 'python /home/nvidia/workspace/catkin_ws/src/ddl_0725/control_0725/drive_control/tools/android_control.py',
       'record_path_begin': 'rosrun pf_localization trajectory_builder.py'}
def process_cmd(cmd, remote_ip=''):
    b = 'zzzzz'
    if cmd == 'remote_control':
        os.system('python /home/nvidia/workspace/catkin_ws/src/ddl_0725/control_0725/drive_control/tools/setmode.py human')
        os.system((cmd_dict.get('sensor') + '&'))
        os.system((cmd_dict.get('drive_control') + '&'))
        os.system((cmd_dict.get('android_control') + '&'))
        # 视频流
        run_cmd = 'python /home/nvidia/workspace/catkin_ws/src/ddl_0725/control_0725/drive_control/tools/video.py '
        run_cmd = run_cmd + remote_ip
        os.system(run_cmd)
    elif cmd == 'self_drive':
        os.system('python /home/nvidia/workspace/catkin_ws/src/ddl_0725/control_0725/drive_control/tools/setmode.py nohuman')
        os.system((cmd_dict.get('sensor') + '&'))
        os.system((cmd_dict.get('drive_control') + '&'))
        os.system((cmd_dict.get('self_drive') + '&'))
    elif cmd == 'roscore':
        os.system((cmd_dict.get('roscore') + '&'))
    elif cmd == 'kill_all':
        os.system((cmd_dict.get('kill_all') + '&'))
    # 请求路径信息
    elif cmd == 'request_path':
        trajecotries_files=os.listdir(trajectories_dir_path)
        send_msgs='Path:{'
        for file in trajecotries_files:
            send_msgs = send_msgs + file + ','
        send_msgs = send_msgs[:-1]
        send_msgs = send_msgs + '}'
        b = send_msgs
    elif cmd == 'record_path_end':
        os.system('rosnode kill /build_trajectory_rt')
        # 返回保存的文件名
        file_list = os.listdir(trajectories_dir_path)
        file_list.sort(key=lambda fn:os.path.getmtime(trajectories_dir_path+'\\'+fn))
        b = file_list[-1]
    else:
        a, b = subprocess.getstatusoutput(cmd + ' &')
    return b


def echo_handler(address, client_sock):
    #print('Got connection from {}'.format(address))
    remote_ip = address[0]
    msg = client_sock.recv(8192)
    if not msg:
        print('message receive failed\n')
        return
    else:
        cmd = msg.decode('utf-8')
        print(cmd)
    b = process_cmd(cmd, remote_ip)
    client_sock.sendall(bytes(b+'\n', 'utf-8'))

    #if cmd_dict.get(cmd):
    #    os.system((cmd_dict.get(cmd)+' &'))
    #    client_sock.sendall(bytes('zzzzz\n', 'utf-8'))
    #else:
    #    a, b = subprocess.getstatusoutput(cmd + ' &')
    #    print(b)
    #    client_sock.sendall(bytes(b + '\n', 'utf-8'))
    print('connection closed\n')
    client_sock.close()

def echo_server(address, backlog=5):
    sock = socket(AF_INET, SOCK_STREAM)
    sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    sock.bind(address)
    sock.listen(backlog)
    while True:
        client_sock, client_addr = sock.accept()
        echo_handler(client_addr, client_sock)
        


if __name__ == '__main__':
    process_cmd('roscore')
    echo_server(('',1111))

