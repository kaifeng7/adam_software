#! /bin/bash
modprobe can
modprobe can-raw
modprobe can-bcm
modprobe can-gw
modprobe can_dev
modprobe mttcan

ip link set can0 type can bitrate 500000 

ip link set up can0

ip link set can1 type can bitrate 500000 

ip link set up can1

