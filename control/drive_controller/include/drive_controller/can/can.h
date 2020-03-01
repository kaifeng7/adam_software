#ifndef DRIVE_CONTROLLER_CAN_CAN_H
#define DRIVE_CONTROLLER_CAN_CAN_H

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>

#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <ros/ros.h>

class Can
{
  public:
    Can() = default;
    Can(std::string can_interface, canid_t can_id);
    ~Can();

    void init(std::string can_interface, canid_t canid);

    int sendStandard(const uint8_t *data, const size_t &size, const canid_t &id);
    int sendStandard(const uint8_t *data, const size_t &size);

    bool sendExtention(canid_t id, uint8_t *data);
    bool sendRTR(canid_t id, uint8_t *data);

    bool receive(uint8_t *data);

    Can &operator=(const Can &can);

    

  private:
    sockaddr_can c_addr_;
    
    can_frame c_sframe_;
    can_frame c_rframe_;
    ifreq ifr_;
    can_filter c_filter_;
    canid_t c_id_;

    int s_; // can raw socket
    std::string can_name_; // can interface name

};

#endif // DRIVE_CONTROLLER_CAN_CAN_H