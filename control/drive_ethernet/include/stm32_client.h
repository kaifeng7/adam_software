/** 
 * Copyright (c) 2019,
 * All rights reserved.
 * 
 * @file stm32client.h
 * 控制模块头文件
 * 
 * @author 金睿   @version V1.0      @date 2019-12-26
 * 
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <cmath>

using namespace std;

/**
 * @class stm32client
 * @brief 控制模块类
 * 
 * @author 金睿   @version V1.0      @date 2019-12-26
 */

class stm32client
{
public:
    stm32client();
    ~stm32client();

    int recv_stm32();  //接收stm32信息
    int send_stm32(int,double,double);  //向stm32发送信息
    bool connect_stm32(char* stm32_ip,int port);  //连接stm32

    double wheel_speed;   //轮数计

private:
    int  client_fd;
};
