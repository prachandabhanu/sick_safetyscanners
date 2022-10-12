#include <thread>
#include <mutex>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

class ObjectDetection
{
    public:
        ObjectDetection(const ros::NodeHandle nh): _nh(nh)
        {
            _subscriber = _nh.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners/scan", 10, \
										boost::bind(&ObjectDetection::callback, this, _1));
            
            _publisher = _nh.advertise<std_msgs::String>("/collision_object_info", 1000);
        }

        void callback(const sensor_msgs::LaserScan::ConstPtr msg)
        {
            int i = 0;
            double dist, angle;
            for (auto range: msg->ranges)
            {
                i ++;
                if (range < 20.0)
                {
                    dist = range;
                    angle = msg->angle_min + i * msg->angle_increment;
                    break;
                } else {
                    dist = 0.0;
                    angle = 0.0;
                }
            }

            if(dist != 0)
            {
                std_msgs::String message;
                std::stringstream ss;
                ss << "Collision object is detected at angle: " << (angle * (180/3.1475)) << " and distance: " << dist; 
                message.data = ss.str();
                _publisher.publish(message);
            }

        }

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _subscriber;
        ros::Publisher _publisher;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_scan_subscriber_node");
    ros::NodeHandle nh;

    ObjectDetection detect(nh);

    ROS_INFO("Checkning Collision object.");
    
    ros::spin();

    return 0;
}