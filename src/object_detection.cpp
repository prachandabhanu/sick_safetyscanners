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
            _subscriber_front = _nh.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners_front/scan", 10, \
										boost::bind(&ObjectDetection::callback_front, this, _1));
            _subscriber_rear = _nh.subscribe<sensor_msgs::LaserScan>("/sick_safetyscanners_rear/scan", 10, \
										boost::bind(&ObjectDetection::callback_rear, this, _1));
            
            _publisher = _nh.advertise<std_msgs::String>("/collision_object_info", 1000);
        }

        void callback_front(const sensor_msgs::LaserScan::ConstPtr msg)
        {
            int i = 0;
            double dist, angle;
            for (auto range: msg->ranges)
            {
                i ++;
                if (range < 10.0)
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
                ss << "Collision object is detected at Front Scanner at the angle: " << (angle * (180/3.1475)) << " degree and distance: " << dist << "m"; 
                message.data = ss.str();
                std::unique_lock<std::mutex> gaurd(_pub_gaurd);
                _publisher.publish(message);
                gaurd.unlock();
            }

        }

        void callback_rear(const sensor_msgs::LaserScan::ConstPtr msg)
        {
            int i = 0;
            double dist, angle;
            for (auto range: msg->ranges)
            {
                i ++;
                if (range < 10.0)
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
                ss << "Collision object is detected at Rear Scanner at the angle: " << (angle * (180/3.1475)) << " degree and distance: " << dist << "m "; 
                message.data = ss.str();
                std::unique_lock<std::mutex> gaurd(_pub_gaurd);
                _publisher.publish(message);
                gaurd.unlock();
            }

        }

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _subscriber_front;
        ros::Subscriber _subscriber_rear;
        ros::Publisher _publisher;
        std::mutex _pub_gaurd;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_detection_node");
    ros::NodeHandle nh;

    ObjectDetection detect(nh);

    ROS_INFO("Checkning Collision object.");
    
    ros::spin();

    return 0;
}