#ifndef TELEOP_JOY_NODE_H
#define TELEOP_JOY_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <robot4ws_msgs/kinematic_mode.h>
#include <robot4ws_msgs/Sound.h>
#include <robot4ws_msgs/enable_logiteck_joystick.h>
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






class Robot4WSTeleop
{
    public:
        Robot4WSTeleop();
        ~Robot4WSTeleop();

        void publishJoyCommandsTransformed(void);

        void printSummary(void);

        void run(void);


    private:
        void joystickSubscriberCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void resetSpeeds(bool publish);

        bool checkCommandedDriveMode(std::vector<int> buttons);
        void callService(int button, std::string mode);
        bool enableJoystickControl(robot4ws_msgs::enable_logiteck_joystick::Request &req, robot4ws_msgs::enable_logiteck_joystick::Response &resp);


        void initializeButtonMapping(void);
        ros::NodeHandle _nh; //Ros Node

        ros::Subscriber _joystick_subscriber; //Subscriber to joy node

        ros::Publisher _joystick_command_publisheer; //ROS Publisher
        ros::Publisher _mode_change_sound_publisher; //ROS Publisher

        ros::ServiceClient _client_mode_change; //ROS Service Client for mode change
        ros::ServiceServer _server_service_provider; //ROS Service Provider


        geometry_msgs::Twist _cmd_vel_msg;

        ros::Time _last_joy_time;
        ros::Time _last_sound_cmd_sent_time;
        ros::Time _last_cmd_sent_time;
        ros::Time _node_initial_time;
        ros::Time _last_mode_changed;


        bool is_moving = false;
        bool enabled = true;

        bool debug = false;
        bool debug_on_publishing = false;
        bool exclude_timeout = true;



        //Constants
        double _linear_x_vel_scale_factor;
        double _linear_y_vel_scale_factor;
        double _angular_z_vel_scale_factor;
        double _controller_timeout;
        double _cmd_publish_rate;
        double _sound_publish_rate;
        double _print_warning_rate;


        //Variables
        double dot_x;
        double dot_y;
        double dot_theta;


        //Buttons Mapping initialization (just for printing purpouse)
        std::map<int, std::string> _button_mapping;
        std::map<int, std::string> _button_mapping_human;



};
#endif //TELEOP_JOY_NODE_H