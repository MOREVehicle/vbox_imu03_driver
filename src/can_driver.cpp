/*BSD 2-Clause License

Copyright (c) 2023, MORE
        All rights reserved.

Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
        IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
        FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
        DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
        SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
        CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 can_driver.cpp
 VBOX IMU03 driver
 Author Antonios Gkougkoulidis
 MORE Minor 2023-2024
 */
#include <iostream>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/can.hpp"

// Define constants for CAN IDs
constexpr uint16_t YAW_PITCH_CAN_ID = 0x600;
constexpr uint16_t ROLL_XACCEL_CAN_ID = 0x601;
constexpr uint16_t YZ_ACCEL_CAN_ID = 0x602;

// Function declarations
void receiveCANCallback(const more_interfaces::msg::Can::SharedPtr can_frame);

// Global variables
double canData = 0.0;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> publisher;
sensor_msgs::msg::Imu imuMsg;

float toFloat(const uint8_t* raw_data) {
    // Change from Motorola (big endian) to little endian
    uint32_t value = static_cast<uint32_t>((raw_data[0] << 24) | (raw_data[1] << 16) | (raw_data[2] << 8) | (raw_data[3]));

    float floatValue;
    // Insert value to a float
    std::memcpy(&floatValue, &value, sizeof(floatValue));
    double result = static_cast<double>(floatValue);
    return result;
}

void extractIMUData(const more_interfaces::msg::Can::SharedPtr can_frame) {
    if (can_frame->id == YAW_PITCH_CAN_ID) { // Yaw Rate & Pitch Rate
        canData = toFloat(&can_frame->data[0]); // Extract Yaw Rate
        imuMsg.angular_velocity.x = canData;
        canData = toFloat(&can_frame->data[4]); // Extract Pitch rate
        imuMsg.angular_velocity.y = canData;
    }

    if (can_frame->id == ROLL_XACCEL_CAN_ID) { // Roll Rate & X_Acceleration
        canData = toFloat(&can_frame->data[0]); // Extract Roll Rate
        imuMsg.angular_velocity.z = canData;
        canData = toFloat(&can_frame->data[4]); // Extract X_Acceleration
        imuMsg.linear_acceleration.x = canData;
    }

    if (can_frame->id == YZ_ACCEL_CAN_ID) { // Y_Acceleration & Z_Acceleration
        canData = toFloat(&can_frame->data[0]); // Extract Y_Acceleration
        imuMsg.linear_acceleration.y = canData;
        canData = toFloat(&can_frame->data[4]); // Extract Z_Acceleration
        imuMsg.linear_acceleration.z = canData;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("imu");
    publisher = node->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    auto subscription = node->create_subscription<more_interfaces::msg::Can>(
        "can_publisher", rclcpp::QoS(100), receiveCANCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

void receiveCANCallback(const more_interfaces::msg::Can::SharedPtr can_frame) {
    extractIMUData(can_frame);
    publisher->publish(imuMsg);
    std::cout << "Received CAN message with ID: " << can_frame->id << std::endl;
    std::cout << "Published IMU message: " << imuMsg << std::endl;
}
