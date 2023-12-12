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
#include "tutorial_interfaces/msg/num.hpp"

static int32_t can_data;
static rclcpp::Publisher publisher;
static sensor_msgs::msg::Imu imu_msg;

float toFloat(const uint8_t* raw_data)

int main(int argc, char **argv){

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("imu");
  
  rclcpp::Subscriber sub = node.subscribe("PCAN", 100, receive_can_callback);
  publisher= node.advertise<sensor_msgs::msg::Imu>("data_raw", 100);
  rclcpp::spin();

  return 0;
}

void receive_can_callback(const tutorial_interfaces::msg::Num* can_frame){


  if(can_frame.id == 0x600){ // Yaw Rate & Pitch Rate
    can_data = toFloat(can_frame.data); //extract Yaw Rate
    imu_msg->angular_velocity.x = can_data;
    can_data = toFloat(can_frame.data + 4); // extract Pitch rate
    imu_msg->angular_velocity.y = can_data;
  }

  if( == 0x601){  //Roll Rate & X_Acceleration
        can_data = toFloat(can_frame.data); //extract Roll Rate
        imu_msg->angular_velocity.z = can_data;
        can_data = toFloat(can_frame.data + 4); //extract X_Acceleration
        imu_msg->linear_acceleration.x = can_data;
  }

  if(id == 0x602){  //Y_Acceleration & Z_Acceleration
      can_data = toFloat(can_frame.data); //extract Y_Acceleration
      imu_msg->linear_acceleration.y = can_data;
      can_data = toFloat(can_frame.data + 4); //extract Z_Acceleration
      imu_msg->linear_acceleration.z = can_data
  }
}

float toFloat(const uint8_t* raw_data) {
  //change from Motorolla(big endian) to little endian
  uint32_t Value = static_cast<uint32_t>((raw_data[0] << 24) | (raw_data[1] << 16)
                                         (raw_data[2] << 8)  | (raw_data[3]));
  float floatvalue;
  //insert value to a float
  std::memcpy(&floatvalue, &Value, sizeof(result));
  double result = static_cast<double>(floatvalue);
  return result;
}
