/*
 * Pure Pursuit Controller (ROS2)
 * Author: KangJehun 20170016 wpgnssla34@kaist.ac.kr
 * Last Edit : 2024.04.23
 * Implementation of pure-pursuit steering controller (ROS2)
 */

// Basic Libraries
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
// TF
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// Msg
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PurePursuitController : public rclcpp::Node
{
private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vehicle_cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_waypoint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_cmdvel_pub_;
    // Msgs
    nav_msgs::msg::Path waypoints_;
    geometry_msgs::msg::Twist cmd_vel_msg_;
    visualization_msgs::msg::Marker marker_waypoint_msg_;
    visualization_msgs::msg::Marker marker_cmdvel_msg_;
    // Parameters 
    std::string odometry_topic_;                // topic name of the odometry data
    std::string waypoint_topic_;                // topic name of the waypoint data
    float frequency_;
    bool verbose_;                              // show debugging messages
    float lookahead_;                           // lookahead distance
    float wheelbase_length_;                    // wheel-base length of vehicle
    float min_steering_;                        // minimum steering angle of vehicle [rad]
    float max_steering_;                        // maximum steering angle of vehicle [rad]
    float min_steering_clipping_;               // minimum steering angle of vehicle [rad]
    float max_steering_clipping_;               // maximum steering angle of vehicle [rad]
    // Member variables
    float theta_rear2goal_;                     // global heading angle of rear to goal vector
    float heading_;                             // global heading angle of the vehicle
    float alpha_;                               // theta_rear2goal - heading
    bool vehicle_state_is_updated_;
    int vehicle_state_update_count_;
    bool waypoint_is_updated_;
    int waypoint_update_count_;
    float x_ego_, y_ego_, z_ego_, yaw_ego_;
    float vx_ego_, wz_ego_;
    size_t wp_arr_size_;
    int nearest_idx_;
    int lookahead_idx_;
    float control_signal_theta_;
public: 
    //
private:
    // Private Methods
    void init_params();
    float find_distance(float x, float y);
    float find_distance_index_based(int idx);
    void find_nearest_waypoint();
    void find_idx_close_to_lookahead();
    void update_alpha();
    void visualize_waypoint_with_idx(int idx, int id, float r, float g, float b);
    void visualize_control_signal(int id, float r, float g, float b);
    // Callback
    void callback_vehiclestate(const nav_msgs::msg::Odometry::SharedPtr msg);
    void callback_waypoints(const nav_msgs::msg::Path::SharedPtr msg);
public:
    PurePursuitController();
    ~PurePursuitController();
    void purepursuit();
    // utils
    float deg2rad(float degrees);
    float rad2deg(float radians);
    float normalizeAngle(float angle);
    float normalizeVector(const std::vector<float>& vect);
    float normalizeControlSignal(float control_signal, float from_min, float from_max, float to_min, float to_max);
    void  quaternionToEuler(const geometry_msgs::msg::Quaternion &quat, double &roll, double &pitch, double &yaw);
    geometry_msgs::msg::Quaternion eulerToQuaternion(const double &roll, const double &pitch, const double &yaw);
};

// Constructor
PurePursuitController::PurePursuitController()
: Node("pure_pursuit_controller"),
  theta_rear2goal_(0), heading_(0), alpha_(0),
  vehicle_state_is_updated_(false), vehicle_state_update_count_(0), 
  waypoint_is_updated_(false), waypoint_update_count_(0),
  x_ego_(0), y_ego_(0), z_ego_(0), yaw_ego_(0),
  vx_ego_(0), wz_ego_(0),
  wp_arr_size_(0), nearest_idx_(-1), lookahead_idx_(-1),
  control_signal_theta_(0)
{
    RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node is Launched");

    // Set Parameters, any additional dynamic parameters can be handled here
    init_params();

    // Initialize publishers
    vehicle_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_purepursuit", 10);
    marker_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_waypoint", 10);
    marker_cmdvel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker_cmdvel", 10);

    // Initialize subscribers
    vehicle_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 10, std::bind(&PurePursuitController::callback_vehiclestate, this, _1));
    waypoints_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        waypoint_topic_, 10, std::bind(&PurePursuitController::callback_waypoints, this, _1));
}

// Destructor
PurePursuitController::~PurePursuitController(){}

void PurePursuitController::init_params()
{
    // Declare parameters
    this->declare_parameter<std::string>("odometry_topic", "/odometry/base");
    this->declare_parameter<std::string>("waypoint_topic", "/navigation/global_waypoints");
    this->declare_parameter<float>("frequency", 100.0);
    this->declare_parameter<bool>("verbose", true);
    this->declare_parameter<float>("lookahead_distance", 0.0);
    this->declare_parameter<float>("wheelbase_length", 0.475);
    this->declare_parameter<float>("steering_min", -25.0);
    this->declare_parameter<float>("steering_max", 25.0);
    this->declare_parameter<float>("steering_min_clipping", -25.0);
    this->declare_parameter<float>("steering_max_clipping", 25.0);
    // Get parameters
    this->get_parameter("odometry_topic", odometry_topic_);
    this->get_parameter("waypoint_topic", waypoint_topic_);
    this->get_parameter("frequency", frequency_);
    this->get_parameter("verbose", verbose_);
    this->get_parameter("lookahead_distance", lookahead_);
    this->get_parameter("wheelbase_length", wheelbase_length_);
    this->get_parameter("steering_min", min_steering_);
    this->get_parameter("steering_max", max_steering_);
    this->get_parameter("steering_min_clipping", min_steering_clipping_);
    this->get_parameter("steering_max_clipping", max_steering_clipping_);
    // Covnert units
    max_steering_ = deg2rad(max_steering_);
    min_steering_ = deg2rad(min_steering_);
    max_steering_clipping_ = deg2rad(max_steering_clipping_);
    min_steering_clipping_ = deg2rad(min_steering_clipping_);
    // [DEBUG]
    // [DEBUG]
    if (verbose_)
    {
        RCLCPP_INFO(this->get_logger(), "Parameter Setting");
        RCLCPP_INFO(this->get_logger(), "lookahead: %f", lookahead_);
        RCLCPP_INFO(this->get_logger(), "wheelbase_length: %f", wheelbase_length_);
        RCLCPP_INFO(this->get_logger(), "max_steering [rad]: %f", max_steering_);                  
        RCLCPP_INFO(this->get_logger(), "min_steering [rad]: %f", min_steering_);                  
        RCLCPP_INFO(this->get_logger(), "max_steering_clipping [rad]: %f", max_steering_clipping_);
        RCLCPP_INFO(this->get_logger(), "min_steering_clipping [rad]: %f", min_steering_clipping_);
        RCLCPP_INFO(this->get_logger(), "odometry_topic: %s", odometry_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "waypoint_topic: %s", waypoint_topic_.c_str());
    }
}

float PurePursuitController::find_distance(float x, float y)
{
    float distance = sqrt(pow(x - x_ego_, 2.) + pow(y - y_ego_, 2.));
    return distance;
}

float PurePursuitController::find_distance_index_based(int idx)
{
    float x = waypoints_.poses[idx].pose.position.x;
    float y = waypoints_.poses[idx].pose.position.y;
    return find_distance(x, y);
}

void PurePursuitController::find_nearest_waypoint()
{
    float idx_dist;
    float smallest_dist;

    for (size_t i = 0; i < wp_arr_size_ ; i++)
    {
        idx_dist = find_distance_index_based(i);
        if (i == 0) 
        {
            smallest_dist = idx_dist;
            nearest_idx_ = i;
        }
        if (idx_dist < smallest_dist)
        {
            smallest_dist = idx_dist;
            nearest_idx_ = i;
        }
    }
}

void PurePursuitController::find_idx_close_to_lookahead()
{
    size_t idx = static_cast<size_t>(nearest_idx_);
    if(find_distance_index_based(idx) > lookahead_)
    {
        lookahead_idx_ = nearest_idx_;
    }
    if (vx_ego_ >= 0)
    {
        while (find_distance_index_based(idx) <= lookahead_)
        {
            lookahead_idx_ = idx;
            idx += 1;
            if (idx >= wp_arr_size_) { break; }
        }
    }
    else
    {
        while (find_distance_index_based(idx) < lookahead_)
        {
            lookahead_idx_ = idx;
            idx -= 1;
            if (idx <= 0) { break; }
        }
    }
} 

void PurePursuitController::update_alpha()
{
    float target_x = waypoints_.poses[lookahead_idx_].pose.position.x;
    float target_y = waypoints_.poses[lookahead_idx_].pose.position.y;
    float x_delta = target_x - x_ego_;
    float y_delta = target_y - y_ego_;
    heading_ = normalizeAngle(yaw_ego_);
    theta_rear2goal_ = normalizeAngle(atan2(y_delta, x_delta));
    alpha_ = normalizeAngle(theta_rear2goal_ - heading_);

    //[DEBUG]
    // RCLCPP_INFO(this->get_logger(), "heading angle    [deg]: %f", rad2deg(heading_));
    // RCLCPP_INFO(this->get_logger(), "theta_rear2goal  [deg]: %f", rad2deg(theta_rear2goal_));
    // RCLCPP_INFO(this->get_logger(), "alpha            [deg]: %f", rad2deg(alpha_));
}

void PurePursuitController::callback_vehiclestate(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!vehicle_state_is_updated_)
    {
        vehicle_state_is_updated_ = true;
    }
    vehicle_state_update_count_++;

    // Position
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    float z = msg->pose.pose.position.z;

    // Orientation
    auto quat = msg->pose.pose.orientation;

    // Linear velocity
    float vx = msg->twist.twist.linear.x;

    // Euler from Quaternion
    double roll, pitch, yaw;
    quaternionToEuler(quat, roll, pitch, yaw);

    // [DEBUG]
    // RCLCPP_INFO(this->get_logger(), "Vehicle State is updated %d times", vehicle_state_update_count_);
    // RCLCPP_INFO(this->get_logger(), "Global Position [%f, %f, %f], Vx [%f], Yaw [%f]", x, y, z, vx, yaw);

    // Update member variable
    x_ego_ = x;
    y_ego_ = y;
    z_ego_ = z;
    vx_ego_ = vx;
    yaw_ego_ = yaw;
}

void PurePursuitController::callback_waypoints(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!waypoint_is_updated_)
    {
        waypoint_is_updated_ = true;
    } 
    else { return; } // Do not update waypoints after the first update
    waypoint_update_count_++;

    wp_arr_size_ = waypoints_.poses.size();
    waypoints_ = *msg;
}

void PurePursuitController::purepursuit()
{
    if(!vehicle_state_is_updated_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Vehicle State is not updated yet");
        return;
    }
    if(!waypoint_is_updated_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waypoints are not updated yet");
        return;
    }
    // Get the closest waypoint
    find_nearest_waypoint();
    find_idx_close_to_lookahead();

    // Pure Pursuit Algorithm
    update_alpha();
    control_signal_theta_ = normalizeAngle(std::atan2(2. * wheelbase_length_ * std::sin(alpha_), lookahead_));

    // [DEBUG]
    // RCLCPP_INFO(this->get_logger(), "nearest waypoint index: %d", nearest_idx_);
    // RCLCPP_INFO(this->get_logger(), "lookahead waypoint index: %d", lookahead_idx_);
    // RCLCPP_INFO(this->get_logger(), "Before clipping");
    // RCLCPP_INFO(this->get_logger(), "control signal (theta) [deg]: %f", rad2deg(control_signal_theta_));

    visualize_waypoint_with_idx(nearest_idx_, 0, 1.0, 0.0, 0.0);
    visualize_waypoint_with_idx(lookahead_idx_, 1, 0.0, 0.0, 1.0);
    visualize_control_signal(0, 1.0, 1.0, 0.0);

    // Clipping
    control_signal_theta_ = std::max(min_steering_clipping_, std::min(control_signal_theta_, max_steering_clipping_));

    // [DEBUG]
    // RCLCPP_INFO(this->get_logger(), "After clipping");
    // RCLCPP_INFO(this->get_logger(), "control signal (theta) [deg]: %f", rad2deg(control_signal_theta_));

    // Remapping
    control_signal_theta_ = 
        normalizeControlSignal(control_signal_theta_, min_steering_, max_steering_, -1, 1);

    // [DEBUG]
    // RCLCPP_INFO(this->get_logger(), "After remapping");
    // RCLCPP_INFO(this->get_logger(), "control signal (theta) [-1, 1]: %f", control_signal_theta_);

    // Publish control signal
    cmd_vel_msg_.angular.z = control_signal_theta_;
    vehicle_cmd_vel_pub_->publish(cmd_vel_msg_);
}

void PurePursuitController::visualize_waypoint_with_idx(int idx, int id, float r, float g, float b)
{
    marker_waypoint_msg_.header.frame_id = "odom";
    marker_waypoint_msg_.header.stamp = this->get_clock()->now();
    marker_waypoint_msg_.ns = "nearest_waypoint";
    marker_waypoint_msg_.id = id;
    marker_waypoint_msg_.action = visualization_msgs::msg::Marker::ADD;
    marker_waypoint_msg_.type = visualization_msgs::msg::Marker::SPHERE;
    marker_waypoint_msg_.pose.position.x = waypoints_.poses[idx].pose.position.x;
    marker_waypoint_msg_.pose.position.y = waypoints_.poses[idx].pose.position.y;
    marker_waypoint_msg_.pose.position.z = waypoints_.poses[idx].pose.position.z;
    marker_waypoint_msg_.scale.x = 0.5;
    marker_waypoint_msg_.scale.y = 0.5;
    marker_waypoint_msg_.scale.z = 0.5;
    marker_waypoint_msg_.color.a = 1.0;
    marker_waypoint_msg_.color.r = r;
    marker_waypoint_msg_.color.g = g;
    marker_waypoint_msg_.color.b = b;
    marker_waypoint_pub_->publish(marker_waypoint_msg_);
}

void PurePursuitController::visualize_control_signal(int id, float r, float g, float b)
{
    marker_cmdvel_msg_.header.frame_id = "odom";
    marker_cmdvel_msg_.header.stamp = this->get_clock()->now();
    marker_cmdvel_msg_.ns = "control_signal";
    marker_cmdvel_msg_.id = id;
    marker_cmdvel_msg_.action = visualization_msgs::msg::Marker::ADD;
    marker_cmdvel_msg_.type = visualization_msgs::msg::Marker::ARROW;
    marker_cmdvel_msg_.pose.position.x = x_ego_;
    marker_cmdvel_msg_.pose.position.y = y_ego_;
    marker_cmdvel_msg_.pose.position.z = z_ego_;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = normalizeAngle(heading_ + control_signal_theta_);
    geometry_msgs::msg::Quaternion quat = eulerToQuaternion(roll, pitch, yaw);
    marker_cmdvel_msg_.pose.orientation.x = quat.x;
    marker_cmdvel_msg_.pose.orientation.y = quat.y;
    marker_cmdvel_msg_.pose.orientation.z = quat.z;
    marker_cmdvel_msg_.pose.orientation.w = quat.w;
    marker_cmdvel_msg_.scale.x = 1.0;
    marker_cmdvel_msg_.scale.y = 0.1;
    marker_cmdvel_msg_.scale.z = 0.1;
    marker_cmdvel_msg_.color.a = 1.0;
    marker_cmdvel_msg_.color.r = r;
    marker_cmdvel_msg_.color.g = g;
    marker_cmdvel_msg_.color.b = b;
    marker_cmdvel_pub_->publish(marker_cmdvel_msg_);
}

float PurePursuitController::deg2rad(float degrees) { return normalizeAngle(degrees * (M_PI / 180.0)); }
float PurePursuitController::rad2deg(float radians) { return normalizeAngle(radians) * (180.0 / M_PI); }
float PurePursuitController::normalizeAngle(float angle)
{
    while (angle < 0) { angle += 2 * M_PI; }
    while (angle >= 2 * M_PI) { angle -= 2 * M_PI; }
    if (angle > M_PI) { angle -= 2 * M_PI; }
    return angle;
}
float PurePursuitController::normalizeVector(const std::vector<float> &vect)
{
    float sum = 0.;
    for (size_t i = 0; i < vect.size(); i++) { sum += pow(vect[i], 2.); }
    return sqrt(sum);
}
float PurePursuitController::normalizeControlSignal(
    float control_signal, float from_min, float from_max, float to_min, float to_max)
{
    if (from_max - from_min == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Normalization error : Division by zero");
        rclcpp::shutdown();
        return -1;
    }
    return ((control_signal - from_min) / (from_max - from_min)) * (to_max - to_min) + to_min;
}
void PurePursuitController::quaternionToEuler(
    const geometry_msgs::msg::Quaternion &quat, double &roll, double &pitch, double &yaw)
{
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}
geometry_msgs::msg::Quaternion PurePursuitController::eulerToQuaternion(
    const double &roll, const double &pitch, const double &yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();

    double frequency = 100.0;
    node->get_parameter("frequency", frequency);
    rclcpp::Rate loop_rate(frequency);

    while (rclcpp::ok()) {
        node->purepursuit();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}