#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class SimpleSubscriber
{
    public:
        SimpleSubscriber(const ros::NodeHandle nh): _nh(nh)
        {
            _subscriber = _nh.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners/scan", 10, \
										boost::bind(&SimpleSubscriber::callback, this, _1));
        }

        void callback(const sensor_msgs::LaserScan::ConstPtr msg)
        {
            ROS_INFO("Received Scan info: \n Frame ID: %s, \n Max angle: %f, \n Min angle: %f, \
					\n Angle increment: %f, \n Max range: %f, \n Min range: %f", \
					msg->header.frame_id.c_str(), msg->angle_max, msg->angle_min, msg->angle_increment,  \
					msg->range_max, msg->range_min);
        }

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _subscriber;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_scan_subscriber_node");
    ros::NodeHandle nh;

    SimpleSubscriber sub(nh);
    ros::spin();

    return 0;
}