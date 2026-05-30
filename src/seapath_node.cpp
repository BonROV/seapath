#include "seapath_node.h"

using namespace std::chrono_literals;

NS_HEAD

// ── helpers ───────────────────────────────────────────────────────────────────

static int32_t s_ntohl(int32_t val)
{
    return static_cast<int32_t>(ntohl(static_cast<uint32_t>(val)));
}

static int16_t s_ntohs(int16_t val)
{
    return static_cast<int16_t>(ntohs(static_cast<uint16_t>(val)));
}

// CCITT CRC16 — from page 144 of the Seapath manual
static uint16_t seapath_crc16(const uint8_t* buf, size_t len)
{
    uint8_t  i;
    uint16_t data;
    uint16_t crc = 0xffff;

    if (len == 0) return ~crc;

    do {
        data = static_cast<uint16_t>(0xff & *buf++);
        for (i = 0; i < 8; i++) {
            if ((crc & 0x0001) ^ (data & 0x0001))
                crc = (crc >> 1) ^ POLY;
            else
                crc >>= 1;
            data >>= 1;
        }
    } while (--len);

    crc  = ~crc;
    data = crc;
    crc  = static_cast<uint16_t>((crc << 8) | ((data >> 8) & 0xff));
    return crc;
}

static void setStamp(std_msgs::msg::Header& header, const Seapath23_raw_t* raw)
{
    header.stamp.sec     = s_ntohl(raw->time);
    header.stamp.nanosec = static_cast<uint32_t>(ntohs(raw->time_frac)) * 100000UL;
}

// ── SeapathNode ───────────────────────────────────────────────────────────────

SeapathNode::SeapathNode()
    : Node("seapath_node")
{
    declare_parameter("connection_topic", std::string(""));
    declare_parameter("frame_id",         std::string("imu"));

    std::string conn = get_parameter("connection_topic").as_string();
    frame_id_        = get_parameter("frame_id").as_string();

    pub_.navsat = create_publisher<sensor_msgs::msg::NavSatFix>("~/fix",        10);
    pub_.imu    = create_publisher<sensor_msgs::msg::Imu>("~/imu",              10);
    pub_.twist  = create_publisher<geometry_msgs::msg::TwistStamped>("~/twist", 10);
    pub_.odom   = create_publisher<nav_msgs::msg::Odometry>("~/odom",           10);

    if (conn.empty()) {
        RCLCPP_WARN(get_logger(), "connection_topic is empty — not subscribing");
        return;
    }

    sub_ = create_subscription<io_interfaces::msg::RawPacket>(
        conn, 10,
        [this](const io_interfaces::msg::RawPacket::SharedPtr msg) { onPacket(msg); });

    RCLCPP_INFO(get_logger(), "Subscribed to %s", conn.c_str());
}

// ── packet handler ────────────────────────────────────────────────────────────

void SeapathNode::onPacket(const io_interfaces::msg::RawPacket::SharedPtr msg)
{
    if (msg->data.size() != BUFSIZE) {
        RCLCPP_WARN(get_logger(), "Unexpected packet size: %zu (expected %d)",
                    msg->data.size(), BUFSIZE);
        return;
    }

    const uint8_t* bytes = msg->data.data();

    uint16_t crc_calc = seapath_crc16(bytes + 2, msg->data.size() - 4);
    uint16_t crc_recv = ntohs(*reinterpret_cast<const uint16_t*>(bytes + 42));
    if (crc_calc != crc_recv) {
        ++crc_errors_;
        RCLCPP_WARN(get_logger(), "CRC mismatch: calc=%04x recv=%04x (total errors: %lu)",
                    crc_calc, crc_recv, crc_errors_);
        return;
    }

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    const auto* raw = reinterpret_cast<const Seapath23_raw_t*>(bytes);
    publishNavsat(raw);
    publishOdom(raw);
    publishImu(raw);
    publishTwist(raw);
}

// ── publish helpers ───────────────────────────────────────────────────────────

void SeapathNode::publishNavsat(const Seapath23_raw_t* raw)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.header.frame_id = "mru";
    setStamp(msg.header, raw);
    msg.latitude  = static_cast<double>(s_ntohl(raw->latitude))  / LAT_SCALE * 90.0;
    msg.longitude = static_cast<double>(s_ntohl(raw->longitude)) / LON_SCALE * 90.0;
    msg.altitude  = static_cast<double>(s_ntohl(raw->height))    / 100.0;
    pub_.navsat->publish(msg);
}

void SeapathNode::publishImu(const Seapath23_raw_t* raw)
{
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = frame_id_;
    setStamp(msg.header, raw);

    const double rad = PI / 180.0;
    double roll  = static_cast<double>(s_ntohs(raw->roll))   / ANG_SCALE * 90.0 * rad;
    double pitch = static_cast<double>(s_ntohs(raw->pitch))  / ANG_SCALE * 90.0 * rad;
    double hdg   = static_cast<double>(ntohs(raw->heading))  / ANG_SCALE * 90.0;
    double yaw   = (90.0 - hdg) * rad;

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    msg.orientation.x = quat.x();
    msg.orientation.y = quat.y();
    msg.orientation.z = quat.z();
    msg.orientation.w = quat.w();

    msg.angular_velocity.x = static_cast<double>(s_ntohs(raw->rate_roll))    / ANG_SCALE * 90.0 * rad;
    msg.angular_velocity.y = static_cast<double>(s_ntohs(raw->rate_pitch))   / ANG_SCALE * 90.0 * rad;
    msg.angular_velocity.z = static_cast<double>(s_ntohs(raw->rate_heading)) / ANG_SCALE * 90.0 * rad;

    pub_.imu->publish(msg);
}

void SeapathNode::publishTwist(const Seapath23_raw_t* raw)
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id = frame_id_;
    setStamp(msg.header, raw);

    const double rad = PI / 180.0;
    msg.twist.linear.x = static_cast<double>(s_ntohs(raw->vel_north)) / 100.0;
    msg.twist.linear.y = static_cast<double>(s_ntohs(raw->vel_east))  / 100.0;
    msg.twist.linear.z = static_cast<double>(s_ntohs(raw->vel_down))  / 100.0;

    msg.twist.angular.x = static_cast<double>(s_ntohs(raw->rate_roll))    / ANG_SCALE * 90.0 * rad;
    msg.twist.angular.y = static_cast<double>(s_ntohs(raw->rate_pitch))   / ANG_SCALE * 90.0 * rad;
    msg.twist.angular.z = static_cast<double>(s_ntohs(raw->rate_heading)) / ANG_SCALE * 90.0 * rad;

    pub_.twist->publish(msg);
}

void SeapathNode::publishOdom(const Seapath23_raw_t* raw)
{
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "sea_surface";
    msg.child_frame_id  = "mru";
    setStamp(msg.header, raw);

    const double rad = PI / 180.0;
    double roll  = static_cast<double>(s_ntohs(raw->roll))  / ANG_SCALE * 90.0 * rad;
    double pitch = static_cast<double>(s_ntohs(raw->pitch)) / ANG_SCALE * 90.0 * rad;
    double hdg   = static_cast<double>(ntohs(raw->heading)) / ANG_SCALE * 90.0 * rad;

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, hdg);
    msg.pose.pose.orientation.x = quat.x();
    msg.pose.pose.orientation.y = quat.y();
    msg.pose.pose.orientation.z = quat.z();
    msg.pose.pose.orientation.w = quat.w();

    msg.twist.twist.linear.x = static_cast<double>(s_ntohs(raw->vel_north)) / 100.0;
    msg.twist.twist.linear.y = static_cast<double>(s_ntohs(raw->vel_east))  / 100.0;
    msg.twist.twist.linear.z = static_cast<double>(s_ntohs(raw->vel_down))  / 100.0;

    msg.twist.twist.angular.x = static_cast<double>(s_ntohs(raw->rate_roll))    / ANG_SCALE * 90.0 * rad;
    msg.twist.twist.angular.y = static_cast<double>(s_ntohs(raw->rate_pitch))   / ANG_SCALE * 90.0 * rad;
    msg.twist.twist.angular.z = static_cast<double>(s_ntohs(raw->rate_heading)) / ANG_SCALE * 90.0 * rad;

    pub_.odom->publish(msg);
}

NS_FOOT
