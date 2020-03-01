Header header
float32 steer_cmd
float32 accel_cmd # 油门，应该永远大于等于 0，需要结合当前档位计算速度
float32 brake_cmd # 刹车，应该永远大于等于 0，需要结合当前档位计算速度
adam_msgs/lamp_cmd lamp_cmd
int32 gear
int32 mode # 档位 0: 倒退，1: 前进
geometry_msgs/TwistStamped twist_cmd
adam_msgs/ControlCommand ctrl_cmd
int32 emergency