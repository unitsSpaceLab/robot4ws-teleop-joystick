/**
 * @author Matteo Caruso
 * @email matteo.caruso@phd.units.it
 * @email matteo.caruso1993@gmail.com
**/

#include "../include/teleop_joy_node.h"

Robot4WSTeleop::Robot4WSTeleop() : Node("robot4ws_joy_teleop_transform_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting Robot4WS joystick teleoperation transform node...");

    // Declare and get parameters
    this->declare_parameter("scale_x_vel", 0.2);
    this->declare_parameter("scale_y_vel", 0.2);
    this->declare_parameter("scale_omega_vel", 0.5);
    this->declare_parameter("controller_timeout", 2.0);
    this->declare_parameter("cmd_publish_rate", 10.0);
    this->declare_parameter("sound_publish_rate", 0.2);
    this->declare_parameter("warning_print_rate", 0.2);

    this->get_parameter("scale_x_vel", _linear_x_vel_scale_factor);
    this->get_parameter("scale_y_vel", _linear_y_vel_scale_factor);
    this->get_parameter("scale_omega_vel", _angular_z_vel_scale_factor);
    this->get_parameter("controller_timeout", _controller_timeout);
    this->get_parameter("cmd_publish_rate", _cmd_publish_rate);
    this->get_parameter("sound_publish_rate", _sound_publish_rate);
    this->get_parameter("warning_print_rate", _print_warning_rate);

    // Initialize button mapping
    initializeButtonMapping();

    // Publishers
    _joystick_command_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    _mode_change_sound_publisher = this->create_publisher<robot4ws_msgs::msg::Sound>("/sound", 1);

    // Subscriber
    _joystick_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&Robot4WSTeleop::joystickSubscriberCallback, this, std::placeholders::_1));

    // Service client
    _client_mode_change = this->create_client<robot4ws_msgs::srv::KinematicMode>("kinematic_mode");

    // Service server
    _server_service_provider = this->create_service<robot4ws_msgs::srv::EnableLogiteckJoystick>(
        "enable_logiteck_controller",
        std::bind(&Robot4WSTeleop::enableJoystickControl, this, std::placeholders::_1, std::placeholders::_2));

    _node_initial_time = this->now();
    _last_joy_time = this->now();
    _last_mode_changed = this->now();

    resetSpeeds(true);

    RCLCPP_INFO(this->get_logger(), "Robot4WS joystick teleoperation transform node initialized!");
    printSummary();
}

Robot4WSTeleop::~Robot4WSTeleop() {}

void Robot4WSTeleop::printSummary(void)
{
    std::cerr << "==============================================================================================================\n";
    std::cerr << "\t\t\t\t\tLOGITECH CONTROLLER SUMMARY\n";
    std::cerr << "==============================================================================================================\n";

    for(auto& it : _button_mapping)
    {
        int k = it.first;
        std::string v_1 = it.second;
        std::string v_2 = _button_mapping_human[k];
        std::cerr << "+ Button number [" << k << "],\t Button label ['" << v_2 <<"'],\t Description [" << v_1 << "]\n";
    }
    std::cerr << "==============================================================================================================\n";
}

void Robot4WSTeleop::initializeButtonMapping(void)
{
    _button_mapping = {{SYMM_ACKERMANN_BUTTON, "Drive mode: Symmetric Ackerman"},
                       {CAR_LIKE_BUTTON, "Drive mode: Car like Ackermann"},
                       {OUTER_ACKERMANN, "Drive mode: Outer Ackermann"},
                       {INNER_ACKERMANN, "Drive mode: Inner Ackermann"},
                       {GENERAL_ACKERMANN, "Drive mode: General Ackermann"},
                       {PARALLEL_DRIVE, "Drive mode: Parallel drive (Ackermann)"},
                       {LATERAL_DRIVE, "Drive mode: Lateral drive"},
                       {DEADMAN_BUTTON, "Deadman Button (Press to move robot)"}};

    _button_mapping_human = {
        {SYMM_ACKERMANN_BUTTON, "X BUTTON"},
        {CAR_LIKE_BUTTON, "A BUTTON"},
        {OUTER_ACKERMANN, "B BUTTON"},
        {INNER_ACKERMANN, "Y BUTTON"},
        {GENERAL_ACKERMANN, "LT BUTTON"},
        {PARALLEL_DRIVE, "RT BUTTON"},
        {LATERAL_DRIVE, "LB BUTTON"},
        {DEADMAN_BUTTON, "RB BUTTON"}
    };
}

bool Robot4WSTeleop::enableJoystickControl(
    const std::shared_ptr<robot4ws_msgs::srv::EnableLogiteckJoystick::Request> req,
    std::shared_ptr<robot4ws_msgs::srv::EnableLogiteckJoystick::Response> resp)
{
    enabled = req->enable_controller;
    resp->success = true;
    if (req->enable_controller)
    {
        RCLCPP_INFO(this->get_logger(), "Requested joystick controlled state [active].");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Requested joystick controlled state [deactivate].");
    }
    return true;
}

void Robot4WSTeleop::resetSpeeds(bool publish)
{
    dot_x = 0;
    dot_y = 0;
    dot_theta = 0;
    if (publish)
    {
        auto tmp_msg = geometry_msgs::msg::Twist();
        _joystick_command_publisher->publish(tmp_msg);
    }
}

void Robot4WSTeleop::joystickSubscriberCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    bool is_dead_button_pressed = (bool)joy->buttons[DEADMAN_BUTTON];

    if (is_dead_button_pressed)
    {
        is_moving = true;
        dot_x = _linear_x_vel_scale_factor * joy->axes[LINEAR_X_AXIS];
        dot_y = _linear_y_vel_scale_factor * joy->axes[LINEAR_Y_AXIS];
        dot_theta = _angular_z_vel_scale_factor * joy->axes[ANGULAR_Z_AXIS];

        if (debug)
        {
            std::cerr << "Vel x:\t" << dot_x << "\n";
            std::cerr << "Vel y:\t" << dot_y << "\n";
            std::cerr << "Vel z:\t" << dot_theta << "\n";
        }

        checkCommandedDriveMode(joy->buttons);
        _last_joy_time = this->now();
    }
    else
    {
        is_moving = false;
        resetSpeeds(false);
    }

    if (!exclude_timeout)
    {
        if ((this->now() - _last_joy_time).seconds() >= _controller_timeout)
        {
            static rclcpp::Time last_message_time_printed = this->now();

            if ((this->now() - last_message_time_printed).seconds() >= 1/_print_warning_rate)
            {
                RCLCPP_WARN(this->get_logger(), "Controller timeout... Resetting motors.");
                last_message_time_printed = this->now();
            }
            resetSpeeds(false);
        }
    }
}

bool Robot4WSTeleop::checkCommandedDriveMode(std::vector<int> buttons)
{
    if ((this->now() - _last_mode_changed).seconds() < 5)
        return false;

    std::string mode;
    
    if (buttons[SYMM_ACKERMANN_BUTTON]) { mode = "symmetric_ackermann"; callService(SYMM_ACKERMANN_BUTTON, mode); return true; }
    if (buttons[CAR_LIKE_BUTTON]) { mode = "car_like"; callService(CAR_LIKE_BUTTON, mode); return true; }
    if (buttons[OUTER_ACKERMANN]) { mode = "outer_ackermann"; callService(OUTER_ACKERMANN, mode); return true; }
    if (buttons[INNER_ACKERMANN]) { mode = "inner_ackermann"; callService(INNER_ACKERMANN, mode); return true; }
    if (buttons[GENERAL_ACKERMANN]) { mode = "full_ackermann"; callService(GENERAL_ACKERMANN, mode); return true; }
    if (buttons[PARALLEL_DRIVE]) { mode = "parallel_ackermann"; callService(PARALLEL_DRIVE, mode); return true; }
    if (buttons[LATERAL_DRIVE]) { mode = "lateral_drive"; callService(LATERAL_DRIVE, mode); return true; }

    return false;
}

void Robot4WSTeleop::callService(int button, std::string mode)
{
    auto request = std::make_shared<robot4ws_msgs::srv::KinematicMode::Request>();
    request->requested_kinematic_mode = mode;

    if (debug)
    {
        RCLCPP_INFO(this->get_logger(), "Entering service call function");
    }

    resetSpeeds(true);
    _last_mode_changed = this->now();

    auto result = _client_mode_change->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Changed Driving mode with button [%d - (%s)]", button, mode.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to change driving mode with button [%d - (%s)]", button, mode.c_str());
    }
}

void Robot4WSTeleop::run(void)
{
    _node_initial_time = this->now();
    _last_cmd_sent_time = this->now();
    _last_sound_cmd_sent_time = this->now();

    auto timer = this->create_wall_timer(
        std::chrono::milliseconds((int)(1000.0/_cmd_publish_rate)),
        [this]() {
            auto cmd_vel_msg = geometry_msgs::msg::Twist();
            cmd_vel_msg.linear.x = dot_x;
            cmd_vel_msg.linear.y = dot_y;
            cmd_vel_msg.angular.z = dot_theta;
            _joystick_command_publisher->publish(cmd_vel_msg);
            
            if (debug_on_publishing)
            {
                RCLCPP_INFO(this->get_logger(), "PUBLISHING VEL");
            }
        });
    
    rclcpp::spin(this->shared_from_this());
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto teleop_node = std::make_shared<Robot4WSTeleop>();
    teleop_node->run();
    rclcpp::shutdown();
    return 0;
}