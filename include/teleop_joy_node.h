/**
 * @author Matteo Caruso
 * @email matteo.caruso@phd.units.it
 * @email matteo.caruso1993@gmail.com
**/
#ifndef TELEOP_JOY_NODE_H
#define TELEOP_JOY_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot4ws_msgs/srv/kinematic_mode.hpp>
#include <robot4ws_msgs/msg/sound.hpp>
#include <robot4ws_msgs/srv/enable_logiteck_joystick.hpp>
#include <vector>
#include <map>

#define LINEAR_Y_AXIS 0
#define LINEAR_X_AXIS 1
#define ANGULAR_Z_AXIS 2
#define SYMM_ACKERMANN_BUTTON 0 //X BUTTON
#define CAR_LIKE_BUTTON 1 //A BUTTON
#define OUTER_ACKERMANN 2 //B BUTTON
#define INNER_ACKERMANN 3 //Y BUTTON
#define GENERAL_ACKERMANN 6 //LT BUTTON
#define PARALLEL_DRIVE 7 //RT BUTTON
#define LATERAL_DRIVE 4 //LB BUTTON
#define DEADMAN_BUTTON 5 //RB BUTTON

class Robot4WSTeleop : public rclcpp::Node
{
public:
    Robot4WSTeleop();
    ~Robot4WSTeleop();
    void publishJoyCommandsTransformed(void);
    void printSummary(void);
    void run(void);

private:
    void joystickSubscriberCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    void resetSpeeds(bool publish);
    bool checkCommandedDriveMode(std::vector<int> buttons);
    void callService(int button, std::string mode);
    bool enableJoystickControl(
        const std::shared_ptr<robot4ws_msgs::srv::EnableLogiteckJoystick::Request> req,
        std::shared_ptr<robot4ws_msgs::srv::EnableLogiteckJoystick::Response> resp);
    void initializeButtonMapping(void);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joystick_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _joystick_command_publisher;
    rclcpp::Publisher<robot4ws_msgs::msg::Sound>::SharedPtr _mode_change_sound_publisher;
    rclcpp::Client<robot4ws_msgs::srv::KinematicMode>::SharedPtr _client_mode_change;
    rclcpp::Service<robot4ws_msgs::srv::EnableLogiteckJoystick>::SharedPtr _server_service_provider;

    geometry_msgs::msg::Twist _cmd_vel_msg;
    rclcpp::Time _last_joy_time;
    rclcpp::Time _last_sound_cmd_sent_time;
    rclcpp::Time _last_cmd_sent_time;
    rclcpp::Time _node_initial_time;
    rclcpp::Time _last_mode_changed;

    bool is_moving = false;
    bool enabled = true;
    bool debug = false;
    bool debug_on_publishing = false;
    bool exclude_timeout = true;

    double _linear_x_vel_scale_factor;
    double _linear_y_vel_scale_factor;
    double _angular_z_vel_scale_factor;
    double _controller_timeout;
    double _cmd_publish_rate;
    double _sound_publish_rate;
    double _print_warning_rate;

    double dot_x;
    double dot_y;
    double dot_theta;

    std::map<int, std::string> _button_mapping;
    std::map<int, std::string> _button_mapping_human;
};

#endif //TELEOP_JOY_NODE_H