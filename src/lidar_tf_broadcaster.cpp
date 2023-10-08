
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class LidarTFBroadcaster : public rclcpp::Node
{
public:
    LidarTFBroadcaster() : Node("lidar_tf_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&LidarTFBroadcaster::broadcastLidarTF, this));
    }

private:
    void broadcastLidarTF()
    {
        geometry_msgs::msg::TransformStamped lidar_transform;
        lidar_transform.header.stamp = now();
        lidar_transform.header.frame_id = "base_link"; // Parent frame
        lidar_transform.child_frame_id = "laser_frame";      // Child frame
        lidar_transform.transform.translation.x = 0.0;  // Adjust these values accordingly
        lidar_transform.transform.translation.y = 0.0;
        lidar_transform.transform.translation.z = 0.0;
        lidar_transform.transform.rotation.x = 0.0;
        lidar_transform.transform.rotation.y = 0.0;
        lidar_transform.transform.rotation.z = 0.0;
        lidar_transform.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(lidar_transform);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
