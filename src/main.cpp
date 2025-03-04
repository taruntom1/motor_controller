#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "Structs.hpp"
#include "ControlInterface.hpp"
#include "ReadJSON.hpp"

class MyNode : public rclcpp::Node
{
private:
    ProtocolConfig config{ "/dev/ttyUSB0" };
    ControllerData controllerData;
    ControlInterface controlInterface;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    void publish_data()
    {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data = {controllerData.motorData[0].odometryData.angle, controllerData.motorData[1].odometryData.angle};
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published: [%d, %d]", msg.data[0], msg.data[1]);
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
        loadControllerDataFromJson("/home/tarun/ros2_ws/src/motor_controller/controller_data.json", controllerData, get_logger());

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
    MyNode() : Node("my_node"), controlInterface(config, controllerData)
    {
        setup_controller();
        setup_odometry();

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("odometry_array_topic", 10);
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MyNode::publish_data, this));

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
