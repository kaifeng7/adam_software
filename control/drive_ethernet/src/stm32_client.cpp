/**
* Copyright (c) 2019
* All rights reserved.
*
* @file stm32client.cpp
* 控制模块代码实现主文件
*
* @author 金睿   @version V1.0      @date 2019-12-26
*
*/

#include "stm32_client.h"
#include "protocol.h"

using namespace std;

stm32client::stm32client()
{
    client_fd = -1;
    wheel_speed = 0.00;
}

stm32client::~stm32client()
{
    close(client_fd);
}

/**
* connect_stm32
* @return bool 是否成功连接stm32
*/
bool stm32client::connect_stm32(char* stm32_ip,int port)
{
    struct sockaddr_in conn_addr;
    if((client_fd=socket(AF_INET,SOCK_STREAM,0))<0){
        printf("socket error\n");   
        return false;
    }

    bzero(&conn_addr,sizeof(conn_addr));
    conn_addr.sin_family=AF_INET;
    conn_addr.sin_port=htons(port);
    conn_addr.sin_addr.s_addr=inet_addr(stm32_ip);

    if(connect(client_fd,(struct sockaddr*)&conn_addr,sizeof(conn_addr))<0){
        printf("connect stm32 error\n");
        close(client_fd);
        return false;
    }

    return true;
}

/**
* send_stm32 向stm32发送控制指令
* @param  mode      前进、刹车
* @param  speed     速度
* @param  angle     转向角度，正负
* @return int 成功发送的字节数
*/
int stm32client::send_stm32(int mode,double speed,double angle)
{
    struct control_raw_data crd;
    memset(&crd,0,sizeof(struct control_raw_data));
    crd.start      = 0xFA;
    crd.type       = 0x31;
    crd.no         = 0x00;

    crd.spare[0]   = 0x00;
    crd.spare[1]   = 0x00;
    crd.spare[2]   = 0x00;
    crd.spare[3]   = 0x00;
    crd.spare[4]   = 0x00;
    crd.valcode    = 0x00;
    crd.end        = 0xFB;
        
    if(mode == 1)
    {
        if(speed > 0)
        {
            if(speed > 2)
            { 
                speed = 2;
            }

            if((angle >= 0)&&(angle < 1.57))
            {
                crd.quadrant   = 1;
            }
            else if((angle > -1.57)&&(angle < 0))
            {
                crd.quadrant   = 4;
            }
            else
            {
                crd.quadrant   = 0x00;
                crd.angle      = 0x00;
                crd.velocity   = 0x00;
            }
                            
            crd.angle      = (int8_t)(((abs(angle))/3.14)*180);
            crd.velocity   = (int8_t)(speed/0.4);
        }
        else
        {
            crd.quadrant   = 0x00;
            crd.angle      = 0x00;
            crd.velocity   = 0x00;
        }
    
    }
    else
    {
        crd.quadrant   = 0x00;
        crd.angle      = 0x00;
        crd.velocity   = 0x00;
    }

    int send_num = -1;
    if(send_num = (write(client_fd,(void *)&crd,sizeof(struct control_raw_data))) < 0)
    {
        printf("send error\n"); 
        return send_num;
    }
    
    printf("input mode: %d, speed: %f, angle: %f\n",mode,speed,angle); 
    printf("Send num: %d, quadrant: %d, velocity: %d, angle: %d\n",send_num,crd.quadrant,crd.velocity,crd.angle);
    return send_num;
}

/**
* recv_stm32 接收stm发送信息
* @return int 成功接收的字节数
*/
int stm32client::recv_stm32()
{
    int count = -1;
    struct odom_raw_data ord;
    char recv_buf[sizeof(struct odom_raw_data)];
    memset(recv_buf,0,sizeof(struct odom_raw_data));
    memset(&ord,0,sizeof(struct odom_raw_data));
        
    count=read(client_fd,(void *)recv_buf,sizeof(struct odom_raw_data));
    printf("recv stm32 num = %d\n",count);
    if(count > 0)
    {
        memcpy(&ord,recv_buf,sizeof(struct odom_raw_data));
        wheel_speed = (double)(ord.wheel_v/10000);
//        printf("wheel_v = %f\n",ord.wheel_v);
        printf("wheel_speed = %f\n",wheel_speed);
    }
    else
    {
        printf("recv from stm32 wheel_speed fail!\n");
        return 0;
    }    
    return count;
}

