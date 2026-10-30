#ifndef SEAPATHNODE_H
#define SEAPATHNODE_H

#include "defs.h"
#include "types.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdint.h>

#include <rclcpp/rclcpp.hpp>
#include <io_interfaces/msg/raw_packet.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

NS_HEAD

#define BUFSIZE 44

class SeapathNode : public rclcpp::Node
{
public:
    SeapathNode();

private:
    void onPacket(const io_interfaces::msg::RawPacket::SharedPtr msg);

    void publishNavsat(const Seapath23_raw_t* raw);
    void publishOdom(const Seapath23_raw_t* raw);
    void publishImu(const Seapath23_raw_t* raw);
    void publishTwist(const Seapath23_raw_t* raw);

    rclcpp::Subscription<io_interfaces::msg::RawPacket>::SharedPtr sub_;

    struct {
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    } pub_;

    std::string frame_id_ = "imu";
    uint64_t    crc_errors_ = 0;
};

NS_FOOT

#endif // SEAPATHNODE_H
