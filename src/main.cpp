#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "Structs.hpp"
#include "ControlInterface.hpp"
#include "ReadJSON.hpp"

class MyNode : public rclcpp::Node
{
private:
    ProtocolConfig config{"/dev/ttyUSB0"};
    ControllerData controllerData;
    ControlInterface controlInterface;

    double wheel_radius = 0.05;          // 5cm radius
    double wheel_base = 0.3;             // 30cm wheel separation
    double ticks_per_revolution = 500.0; // Example value

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    void publish_odometry()
    {
        static std::vector<int> encoder_ticks = {0, 0};     // Example encoder tick storage
        static std::vector<double> tick_rates = {0.1, 0.1}; // Example tick rate storage (ticks per second)

        // Convert encoder ticks to wheel displacement (meters)
        double left_distance = (encoder_ticks[0] / ticks_per_revolution) * (2 * M_PI * wheel_radius);
        double right_distance = (encoder_ticks[1] / ticks_per_revolution) * (2 * M_PI * wheel_radius);

        // Compute total displacement and heading change
        double d = (left_distance + right_distance) / 2.0;
        double delta_theta = (right_distance - left_distance) / wheel_base;

        // Update position directly
        static double x = 0.0, y = 0.0, theta = 0.0;
        theta += delta_theta;
        x += d * cos(theta);
        y += d * sin(theta);

        // Compute linear and angular velocity from tick rates
        double left_velocity = (tick_rates[0] / ticks_per_revolution) * (2 * M_PI * wheel_radius);
        double right_velocity = (tick_rates[1] / ticks_per_revolution) * (2 * M_PI * wheel_radius);
        double v = (left_velocity + right_velocity) / 2.0;
        double w = (right_velocity - left_velocity) / wheel_base;

        // Publish odometry message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
        odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = w;
        odom_pub_->publish(odom_msg);

        // Publish transform from odom to base_link
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.rotation.z = sin(theta / 2.0);
        t.transform.rotation.w = cos(theta / 2.0);
        tf_broadcaster_.sendTransform(t);
    }

    void listener_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        controllerData.motorData[0].pwmValue = msg->data[0];
        controllerData.motorData[1].pwmValue = msg->data[1];
        controlInterface.SetMotorPWMs();
        RCLCPP_INFO(this->get_logger(), "Received PWM values: [%d, %d]", msg->data[0], msg->data[1]);
    }

    void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service called with: %s", request->data ? "true" : "false");
        response->success = true;
        response->message = "Service executed successfully.";
    }

    void run_control()
    {
        controlInterface.Run();
        RCLCPP_INFO(this->get_logger(), "ControlInterface Run() executed.");
    }

    void setup_controller()
    {
        loadControllerDataFromJson("/home/tarun/ros2_ws/src/motor_controller/controller_data.json", controllerData);

        for (int i = 0; i < 10; i++)
        {
            RCLCPP_INFO(this->get_logger(), "Pinging controller...");
            if (controlInterface.Ping())
            {
                RCLCPP_INFO(this->get_logger(), "Controller response received.");
                break;
            }
            RCLCPP_WARN(this->get_logger(), "No response from controller. Retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (controlInterface.SetControllerProperties())
            RCLCPP_INFO(this->get_logger(), "Controller properties set successfully.");
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to set controller properties.");

        for (int i = 0; i < controllerData.controllerProperties.numMotors; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            if (controlInterface.SetMotorData(i))
                RCLCPP_INFO(this->get_logger(), "Motor %d data set successfully.", i);
            else
                RCLCPP_ERROR(this->get_logger(), "Failed to set data for motor %d.", i);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (controlInterface.SetMotorControlMode())
            RCLCPP_INFO(this->get_logger(), "Motor control mode set to direct PWM.");
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to set motor control mode.");
    }

    void setup_odometry()
    {
        controllerData.motorData[0].odoBroadcastStatus.angleBroadcast = true;
        controllerData.motorData[1].odoBroadcastStatus.angleBroadcast = true;
        controllerData.motorData[0].odoBroadcastStatus.speedBroadcast = true;
        controllerData.motorData[1].odoBroadcastStatus.speedBroadcast = true;

        for (int i = 0; i < controllerData.controllerProperties.numMotors; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (controlInterface.SetOdoBroadcastStatus(i))
                RCLCPP_INFO(this->get_logger(), "Odo broadcast enabled for motor %d.", i);
            else
                RCLCPP_ERROR(this->get_logger(), "Failed to enable odo broadcast for motor %d.", i);
        }
    }

public:
    MyNode() : Node("my_node"), controlInterface(config, controllerData), tf_broadcaster_(this)
    {
        setup_controller();
        setup_odometry();

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MyNode::publish_odometry, this));

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "pwm_array_topic", 10, std::bind(&MyNode::listener_callback, this, std::placeholders::_1));

        service_ = this->create_service<std_srvs::srv::SetBool>(
            "bool_service", std::bind(&MyNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MyNode::run_control, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
