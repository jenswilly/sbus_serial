/*
MIT License

Copyright (c) 2024 Jens Willy Johannsen <jens@jwrobotics.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This is a minimal node which publishes an Sbus message on /sbus with hardcoded values.
Use this to test receiving Sbus messages in other nodes without any hardware.
*/

#include "rclcpp/rclcpp.hpp"
#include "sbus_interfaces/msg/sbus.hpp"

class SbusTestPublisher : public rclcpp::Node
{
public:
    SbusTestPublisher() : Node("sbus_test_publisher")
    {
        // Test parameters
        std::string port;
        this->declare_parameter("port", "/dev/default");
        this->get_parameter("port", port);
        RCLCPP_INFO(this->get_logger(), "Using port: %s", port.c_str());

        sbus_msg_.raw_channels[0] = 172;
        sbus_msg_.raw_channels[1] = 172;
        sbus_msg_.raw_channels[2] = 172;
        sbus_msg_.raw_channels[3] = 172;
        sbus_msg_.raw_channels[4] = 172;
        sbus_msg_.raw_channels[5] = 172;
        sbus_msg_.raw_channels[6] = 172;
        sbus_msg_.raw_channels[7] = 172;
        sbus_msg_.raw_channels[8] = 0;
        sbus_msg_.raw_channels[9] = 0;
        sbus_msg_.raw_channels[10] = 0;
        sbus_msg_.raw_channels[11] = 0;
        sbus_msg_.raw_channels[12] = 0;
        sbus_msg_.raw_channels[13] = 0;
        sbus_msg_.raw_channels[14] = 0;
        sbus_msg_.raw_channels[15] = 0;
        sbus_msg_.failsafe = false;
        sbus_msg_.frame_lost = false;
        sbus_msg_.mapped_channels[0] = 50;
        sbus_msg_.mapped_channels[1] = 50;
        sbus_msg_.mapped_channels[2] = 50;
        sbus_msg_.mapped_channels[3] = 50;
        sbus_msg_.mapped_channels[4] = 50;
        sbus_msg_.mapped_channels[5] = 50;
        sbus_msg_.mapped_channels[6] = 50;
        sbus_msg_.mapped_channels[7] = 50;
        sbus_msg_.mapped_channels[8] = 50;
        sbus_msg_.mapped_channels[9] = 50;
        sbus_msg_.mapped_channels[10] = 50;
        sbus_msg_.mapped_channels[11] = 50;
        sbus_msg_.mapped_channels[12] = 50;
        sbus_msg_.mapped_channels[13] = 50;
        sbus_msg_.mapped_channels[14] = 50;
        sbus_msg_.mapped_channels[15] = 50;

        pub_ = this->create_publisher<sbus_interfaces::msg::Sbus>("/sbus", 10);
        RCLCPP_INFO(this->get_logger(), "SbusTestPublisher node initialized");
        timer_ = this->create_wall_timer(std::chrono::seconds(3),
                                         std::bind(&SbusTestPublisher::timerCallback, this));
    }

private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing Sbus message");
        this->sbus_msg_.header.stamp = this->get_clock()->now();
        pub_->publish(this->sbus_msg_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sbus_interfaces::msg::Sbus>::SharedPtr pub_;
    sbus_interfaces::msg::Sbus sbus_msg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SbusTestPublisher>());
    rclcpp::shutdown();
    return 0;
}