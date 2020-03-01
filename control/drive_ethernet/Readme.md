# stm32_client.h、stm32_client.cpp是控制stm32的主要文件，
- 其中向stm发送控制指令的函数是int stm32client::send_stm32(int mode,double speed,double angle)
## param  
- mode  前进、刹车
- speed 速度
- angle 转向角度，正负