#!/usr/local/bin/ python3

from socket import *
import subprocess
import os
import sys

import datetime

trajectories_dir_path = '../../../../localization_0725/ndt_localization/trajectories'
traj_name = 'none'

#trajectories_dir_path = '/Users/soyx/catkin_ws/src/adam_software/localization_0725/pf_localization/trajectories'

cmd_dict_adam = {'roscore': 'roscore',
                 'sensor': 'roslaunch common sensor.launch',
                 'drive_control': 'roslaunch test_launch drive_control_adam.launch',
                #  'self_drive': 'roslaunch common test_0123.launch',
                 'start_localization': 'roslaunch test_launch test_aview.launch',
                 'start_plan': 'roslaunch test_launch planning_0426.launch',
                 'kill_all': 'rosnode kill -a',
                 'android_control': 'python ../../../../control_0725/drive_control/tools/android/android_control.py',
                 'record_path_begin': 'roslaunch ndt_localization build_traj.launch',
                 'start_monitor': 'python ../../../../control_0725/drive_control/tools/android/status_monitor.py'}


cmd_dict_lobo = {'roscore': 'roscore',
                 'sensor': '',
                 'drive_control': 'roslaunch test_launch drive_control.launch',
                #  'self_drive': 'roslaunch common test_0123.launch',
                 'start_localization': 'roslaunch test_launch test_pursuit.launch',
                 'start_plan': 'roslaunch test_launch planning_0426.launch',
                 'kill_all': 'rosnode kill -a',
                 'android_control': 'python ../../../../control_0725/drive_control/tools/android/android_control.py',
                 'record_path_begin': 'roslaunch ndt_localization build_traj.launch',
                 'start_monitor': 'python ../../../../control_0725/drive_control/tools/android/status_monitor.py'}

cmd_dict = {}

def traj_name_cb(traj_msg):
    traj_name = traj_msg.data

def overwrite_ndt_launch(traj_file_name, launch_file_path='../../../../test_launch/launch/robo_0426/planning_0426.launch'):
    file_content = []
    origin_file = open(launch_file_path, 'r')
    for line in origin_file:
        target_index = line.find('name="trajectory"')
        if target_index == -1:
            file_content.append(line)
        else:
            file_content.append(
                '        <param name="trajectory" value="$(find ndt_localization)/trajectories/' + traj_file_name + '" />\n')
    origin_file.close()
    overwrite_file = open(launch_file_path, 'w')
    for line in file_content:
        overwrite_file.write(line)
    overwrite_file.close()


def process_cmd(cmd, remote_ip):
    b = 'done'
    print(cmd[:10] + '\n')
    print(cmd[11:])
    if cmd == 'remote_control':
        os.system(
            'python ../../../../control_0725/drive_control/tools/android/setmode.py human')
        # os.system((cmd_dict.get('sensor') + '&'))
        os.system((cmd_dict.get('drive_control') + '&'))
        os.system((cmd_dict.get('android_control') + '&'))
        # 视频流
        run_cmd = 'python ../../../../control_0725/drive_control/tools/android/video.py '
        run_cmd = run_cmd + remote_ip
        os.system(run_cmd)
    elif cmd[:10] == 'self_drive':
        os.system(
            'python ../../../../control_0725/drive_control/tools/android/setmode.py nohuman')
        path_file_name = cmd[11:]
        overwrite_ndt_launch(path_file_name)
        # os.system((cmd_dict.get('start_localization') + '&'))
        os.system((cmd_dict.get('start_plan') + '&'))
        os.system((cmd_dict.get('drive_control') + '&'))
    elif cmd == 'start_monitor':
        # 安卓状态监控
        print(remote_ip)
        os.system(cmd_dict.get('start_monitor') + ' ' + remote_ip + '&')
    elif cmd =='start_localization':
        os.system((cmd_dict.get('start_localization') +'&'))
    elif cmd == 'roscore':
        os.system((cmd_dict.get('roscore') + '&'))
    elif cmd == 'kill_all':
        os.system((cmd_dict.get('kill_all') + '&'))
    # 请求路径信息
    elif cmd == 'request_path':
        trajecotries_files = os.listdir(trajectories_dir_path)
        send_msgs = 'Path:{'
        for file in trajecotries_files:
            send_msgs = send_msgs + file + ','
        send_msgs = send_msgs[:-1]
        send_msgs = send_msgs + '}'
        b = send_msgs
    elif cmd == 'record_path_begin':
        os.system((cmd_dict.get('record_path_begin') + '&'))
    elif cmd == 'record_path_end':
        os.system('rosnode kill /traj_builder')
        # 返回保存的文件名
        #if traj_name == 'none':
        #    b = "record failed"
        #else:
        #    b = traj_name
        file_list = os.listdir(trajectories_dir_path)
        file_list.sort(key=lambda fn: os.path.getmtime(
            trajectories_dir_path+'/'+fn))
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

    # if cmd_dict.get(cmd):
    #    os.system((cmd_dict.get(cmd)+' &'))
    #    client_sock.sendall(bytes('zzzzz\n', 'utf-8'))
    # else:
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
    if len(sys.argv) > 1 and sys.argv[1] == 'adam':
        cmd_dict = cmd_dict_adam
        print('adam\n')
    else:
        cmd_dict = cmd_dict_lobo
        print('f1\n')
    process_cmd('roscore', '127.0.0.1')
    echo_server(('', 1111))
