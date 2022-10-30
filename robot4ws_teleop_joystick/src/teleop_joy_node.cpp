#include "../include/teleop_joy_node.h"



Robot4WSTeleop::Robot4WSTeleop()
{
    ROS_INFO("Starting Robot4WS joystick teleoperation tranform node...");

    _nh.param("scale_x_vel", this -> _linear_x_vel_scale_factor, 0.2);
    _nh.param("scale_y_vel", this -> _linear_x_vel_scale_factor, 0.2);
    _nh.param("scale_omega_vel", this -> _angular_z_vel_scale_factor, 0.5);
    _nh.param("controller_timeout", this -> _controller_timeout, 2.0);
    _nh.param("cmd_publish_rate", this -> _cmd_publish_rate, 10.0);
    _nh.param("sound_publish_rate", this -> _sound_publish_rate, 0.2);


    //Initialize command vel (cmd_vel) publisher
    _joystick_command_publisheer = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    //Initialize Joystick subscriber to topic /joy
    _joystick_subscriber = _nh.subscribe<sensor_msgs::Joy>("/joy", 10, &Robot4WSTeleop::joystickSubscriberCallback, this);

    //Initialize sound publisher
    _mode_change_sound_publisher = _nh.advertise<robot4ws_msgs::Sound>("/sound",1);


    //Initialize service client
    _client_mode_change = _nh.serviceClient<robot4ws_msgs::kinematic_mode>("kinematic_mode");




    this -> _node_initial_time = ros::Time::now();
    this -> resetSpeeds(true);


    ROS_INFO("Robot4WS joystick teleoperation tranform node initialized!");


};

Robot4WSTeleop::~Robot4WSTeleop()
{

}


void Robot4WSTeleop::resetSpeeds(bool publish)
{
    this -> dot_x = 0;
    this -> dot_y = 0;
    this -> dot_theta = 0;
    if (publish)
    {
        geometry_msgs::Twist tmp_msg;
        this -> _joystick_command_publisheer.publish(tmp_msg);
    }
}



void Robot4WSTeleop::joystickSubscriberCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //Check if deadman button is pressed
    bool is_dead_button_pressed = (bool) joy -> buttons[DEADMAN_BUTTON];

    if (is_dead_button_pressed)
    {
        //DO-SOMETHING
        this -> dot_x = this -> _linear_x_vel_scale_factor * joy -> axes[LINEAR_X_AXIS];
        this -> dot_y = this -> _linear_y_vel_scale_factor * joy -> axes[LINEAR_Y_AXIS];
        this -> dot_theta = this -> _angular_z_vel_scale_factor * joy -> axes[ANGULAR_Z_AXIS];

        std::vector<int> buttons_in = joy-> buttons;
        this ->checkCommandedDriveMode(buttons_in);
    }
    else
    {
        this -> resetSpeeds(false);

    }

};


bool Robot4WSTeleop::checkCommandedDriveMode(std::vector<int> buttons)
{
    std::string mode;

    if ((bool) buttons[SYMM_ACKERMANN_BUTTON])
    {
        //Call service to change mode to ackermann symmetric
        mode = "symmetric_ackermann";


        this -> callService(SYMM_ACKERMANN_BUTTON, mode);
        return true;
    }

    if ((bool) buttons[CAR_LIKE_BUTTON])
    {
        //Call service to change mode to ackermann car-like
        mode = "car_like";
        this -> callService(CAR_LIKE_BUTTON, mode);
        return true;
    }

    if ((bool) buttons[OUTER_ACKERMANN])
    {
        //Call service to change mode to outer ackermann drive mode
        mode = "outer_ackermann";
        this -> callService(OUTER_ACKERMANN, mode);
        return true;
    }

    if ((bool) buttons[INNER_ACKERMANN])
    {
        //Call service to change mode to inner ackermann drive mode
        mode = "inner_ackermann";
        this -> callService(INNER_ACKERMANN, mode);
        return true;
    }

    if ((bool) buttons[GENERAL_ACKERMANN])
    {
        //Call service to change mode to general ackermann drive mode
        mode = "full_ackermann";
        this -> callService(GENERAL_ACKERMANN, mode);
        return true;
    }

    if ((bool) buttons[PARALLEL_DRIVE])
    {
        mode = "parallel_ackermann";
        this -> callService(PARALLEL_DRIVE, mode);
        return true;
    }

    if ((bool) buttons[LATERAL_DRIVE])
    {
        mode = "lateral_drive";
        this -> callService(LATERAL_DRIVE, mode);
        return true;
    }

    return false;
}


void Robot4WSTeleop::callService(int button, std::string mode)
{
    robot4ws_msgs::kinematic_mode srv;
    srv.request.requested_kinematic_mode = mode;
    const char *c_mode = mode.c_str();
    this -> resetSpeeds(true);

    if (_client_mode_change.call(srv))
    {
        ROS_INFO("Changed Driving mode with button [%d - (%s)]", SYMM_ACKERMANN_BUTTON, c_mode);
    }
    else
    {
        ROS_ERROR("Failed to change driving mode with button [%d - (%s)]", SYMM_ACKERMANN_BUTTON, c_mode);
    }

}

void Robot4WSTeleop::run(void)
{
    _node_initial_time = ros::Time::now();
    _last_cmd_sent_time = ros::Time::now();
    _last_sound_cmd_sent_time = ros::Time::now();

    while (true)
    {
        /* code */

        //Publish commands
        if ((ros::Time::now().toSec() - _last_cmd_sent_time.toSec() >= 1/this -> _cmd_publish_rate))
        {
            geometry_msgs::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = this -> dot_x;
            cmd_vel_msg.linear.y = this -> dot_y;
            cmd_vel_msg.angular.z = this -> dot_theta;
            _joystick_command_publisheer.publish(cmd_vel_msg);
            _last_cmd_sent_time = ros::Time::now();

        }

        if ((ros::Time::now().toSec() - _last_sound_cmd_sent_time.toSec() >= 1/this -> _sound_publish_rate))
        {
            robot4ws_msgs::Sound sound_msg;
            sound_msg.value = 1;
            _mode_change_sound_publisher.publish(sound_msg);
            _last_sound_cmd_sent_time = ros::Time::now();
        }
    }
    
}






int main(int argc,char* argv[])
{
    ros::init(argc, argv, "robot4ws_joy_teleop_tranform_node");
    Robot4WSTeleop teleop_node = Robot4WSTeleop();

    teleop_node.run();
    ros::spin();
    return 0;
}




