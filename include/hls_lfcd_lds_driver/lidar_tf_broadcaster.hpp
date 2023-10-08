#ifndef LIDAR_TF_BROADCASTER_HPP_
#define LIDAR_TF_BROADCASTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class LidarTFBroadcaster : public rclcpp::Node
{
public:
    LidarTFBroadcaster();

private:
    void broadcastLidarTF();

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // LIDAR_TF_BROADCASTER_HPP_
