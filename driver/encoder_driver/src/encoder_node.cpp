#include <encoder_driver/encoder_driver.h>
using namespace encoder_driver;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "encoder_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Encoder encoder(nh, pnh);

    encoder.run();

    return 0;
}