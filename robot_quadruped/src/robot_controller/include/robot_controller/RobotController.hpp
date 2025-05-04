#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

#include "robot_controller/RestController.hpp"
#include "robot_controller/Body.hpp"
#include "robot_controller/Leg.hpp"

#define RATE 60

enum State
{
    REST = 0,
    STAND = 1,
    GAIT = 2,
    TROT = 3
};

class RobotController : public rclcpp::Node
{
    public:

        //Constructor 
        RobotController();

		// ROS imu callback
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        void control_loop();

        // change mode
        void change_mode();

    private:

        State state = REST;
        
        //Body body;

        std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

        // Timer for control loop
        rclcpp::TimerBase::SharedPtr timer_;

        // Last IMU reading
        sensor_msgs::msg::Imu imu_data_;

        // rest controller
        //RestController restController;
};

#endif