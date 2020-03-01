/** 
 * Copyright (c) 2019,
 * All rights reserved.
 * 
 * @file protocol.h
 * stm32控制协议
 * 
 * @author 金睿   @version V1.0      @date 2019-12-26
 * 
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

struct odom_raw_data {
    int8_t start;   //包头0xFA
    int8_t type;    // 报文标识 0x30
    int8_t no;      // 0-255循环
    int16_t imut_x; // 加速度计x
    int16_t imut_y; // 加速度计y
    int16_t imut_z; // 加速度计z
    int16_t imut_t; // 温度
    int16_t imua_x; // 陀螺仪x;
    int16_t imua_y; // 陀螺仪y;
    int16_t imua_z; // 陀螺仪z;
    int16_t imuh_x; // 磁针x;
    int16_t imuh_y; // 磁针y;
    int16_t can1[4]; // 超声波雷达第一路数据
    int16_t can2[4]; // 超声波雷达第二路数据
    int16_t wheel_v; // 车轮转速，浮点数×100
    int16_t temp; // 控制器温度
    int8_t io; // 从高到低 推杆上限(左转)，推杆下限（右转），车子状态（开关），防撞杆，低四位暂时保留
    int16_t vol; // 电压
    int16_t current; // 电流
    int8_t valcode; // 校验码 0x00
    int8_t end; //包尾0xFB
};


struct control_raw_data {
    int8_t start;   //包头0xFA
    int8_t type;    // 报文标识 0x31
    int8_t no;      // 0-255循环
    int8_t quadrant; // 象限 0-4，0-不操作，1,4-前进，2,3-后退
    int8_t angle;   // 角度 0-90，1,2向右，3,4向左
    int8_t velocity; // 0-10 速度等级
    int8_t spare[5]; //备用
    int8_t valcode; // 校验码 0x00
    int8_t end; //包尾0xFB
};

