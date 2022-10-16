#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class SimpleSubscriber
{
    public:
        SimpleSubscriber(const ros::NodeHandle nh): _nh(nh)
        {
            _subscriber_front = _nh.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners_front/scan", 10, \
										boost::bind(&SimpleSubscriber::callback_front, this, _1));
            _subscriber_rear = _nh.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners_rear/scan", 10, \
										boost::bind(&SimpleSubscriber::callback_back, this, _1));
        }

        void callback_front(const sensor_msgs::LaserScan::ConstPtr msg)
        {
            ROS_INFO("Received Front Scan info: \n Frame ID: %s, \n Max angle: %f, \n Min angle: %f, \
					\n Angle increment: %f, \n Max range: %f, \n Min range: %f", \
					msg->header.frame_id.c_str(), msg->angle_max, msg->angle_min, msg->angle_increment,  \
					msg->range_max, msg->range_min);
        }

        void callback_back(const sensor_msgs::LaserScan::ConstPtr msg)
        {
            ROS_INFO("Received Rear Scan info: \n Frame ID: %s, \n Max angle: %f, \n Min angle: %f, \
					\n Angle increment: %f, \n Max range: %f, \n Min range: %f", \
					msg->header.frame_id.c_str(), msg->angle_max, msg->angle_min, msg->angle_increment,  \
					msg->range_max, msg->range_min);
        }

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _subscriber_front;
        ros::Subscriber _subscriber_rear;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sick_subscriber");
    ros::NodeHandle nh;

    SimpleSubscriber sub(nh);
    ros::spin();

    return 0;
}