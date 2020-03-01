#include "drive_controller/can/can.h"

// todo 不传入rosnode 传入canid,类似于tcp端口号,每个绑定一个can对象
Can::Can(std::string can_interface, canid_t can_id) : c_id_(can_id)
{
    init(can_interface, can_id);
}

Can::~Can(){
    close(s_);
}


void Can::init(std::string can_interface, canid_t can_id){
    c_id_ = can_id;
    can_name_ = can_interface;

    // open socket
    if ((s_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        ROS_ERROR("can raw socket init error!!");
        return;
    }

    std::strcpy(ifr_.ifr_name, can_name_.c_str());
    printf("%s\n", ifr_.ifr_name);
    ifr_.ifr_ifindex = if_nametoindex(ifr_.ifr_name);
    if (!ifr_.ifr_ifindex){
        ROS_ERROR("if_nametoindex error!!");
        return;
    }
    ioctl(s_, SIOCGIFINDEX, &ifr_);

    std::memset(&c_addr_, 0, sizeof(c_addr_));
    c_addr_.can_family = AF_CAN;
    c_addr_.can_ifindex = ifr_.ifr_ifindex;


    if(bind(s_, (sockaddr *)&c_addr_, sizeof(c_addr_)) < 0){
        ROS_ERROR("socket bind error!");
        return;
    }

    c_filter_.can_id = can_id;
    c_filter_.can_mask = CAN_SFF_MASK;
    setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FILTER, &c_filter_, sizeof(c_filter_));
    //   setsockopt(s_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
}

int Can::sendStandard(const uint8_t *data, const size_t &size, const canid_t &id ){
    std::memset(&c_sframe_, 0, sizeof(canfd_frame));
    c_sframe_.can_id = id;
    c_sframe_.can_dlc = size;
    std::memcpy(c_sframe_.data, data, size);
    for(int i = 0; i < size; i++){
        c_sframe_.data[i] = data[i];
    }
    printf("-------------------\n");
    printf("canid:%d\n", (int)c_sframe_.can_id);
    printf("candlc:%d\n", (int)c_sframe_.can_dlc);
    printf("icandata:%d | %d %d | %d | %d | %d %d | %d ...\n", c_sframe_.data[0], c_sframe_.data[1], c_sframe_.data[2], c_sframe_.data[3], c_sframe_.data[4], c_sframe_.data[5], c_sframe_.data[6], c_sframe_.data[7]);

    int n = write(s_, &c_sframe_, sizeof(can_frame));
    printf("send:%d\n", n);

    return n;
}

int Can::sendStandard(const uint8_t *data, const size_t &size ){
    std::memset(&c_sframe_, 0, sizeof(canfd_frame));
    c_sframe_.can_id = c_id_;
    c_sframe_.can_dlc = size;
    std::memcpy(c_sframe_.data, data, size);
    for(int i = 0; i < size; i++){
        c_sframe_.data[i] = data[i];
    }
    printf("-------------------\n");
    printf("canid:%d\n", (int)c_sframe_.can_id);
    printf("candlc:%d\n", (int)c_sframe_.can_dlc);
    printf("icandata:%d | %d %d | %d | %d | %d %d | %d ...\n", c_sframe_.data[0], c_sframe_.data[1], c_sframe_.data[2], c_sframe_.data[3], c_sframe_.data[4], c_sframe_.data[5], c_sframe_.data[6], c_sframe_.data[7]);

    int n = write(s_, &c_sframe_, sizeof(can_frame));
    printf("send:%d\n", n);

    return n;
}


bool Can::receive(uint8_t *data){
    std::memset(data, 0, 8);

    if(read(s_, &c_rframe_, sizeof(can_frame)) < 0){
        ROS_ERROR("receive error!");
        return false;
    }   
    else{
        for(int i = 0; i < 8; i++){
            data[i] = c_rframe_.data[i];
        }
    }
    return true;
}

Can &Can::operator=(const Can &can){
    this->c_addr_ = can.c_addr_;
    this->c_filter_ = can.c_filter_;
    this->c_id_ = can.c_id_;
    this->ifr_ = can.ifr_;
    this->s_ = can.s_;
    this->can_name_ = can.can_name_;
    return *this;
}
