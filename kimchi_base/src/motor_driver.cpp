// BSD 3-Clause License
//
// Copyright (c) 2023, Ekumen Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "kimchi_base/motor_driver.h"

#include <cstdlib>
#include <iostream>
#include <sstream>

namespace kimchi_base {

void MotorDriver::Setup(const std::string& serial_device, int32_t baud_rate, int32_t timeout_ms) {
  timeout_ms_ = timeout_ms;
  try {
    serial_port_.Open(serial_device);
  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
  }

  // TODO: Use baud_rate from parameter.
  if (baud_rate != 57600) {
    std::cerr << "A baudrate different than 57600 is not supported yet." << std::endl;
  }
  // Configure the serial port.
  serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_57600);
  serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
  serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
  serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
  serial_port_.SetDTR(false);
  serial_port_.SetRTS(false);
  // Flush buffers.
  serial_port_.FlushIOBuffers();
}

bool MotorDriver::is_connected() const { return serial_port_.IsOpen(); }

void MotorDriver::SendEmptyMsg() { std::string response = SendMsg(""); }

std::optional<MotorDriver::HardwareData> MotorDriver::ReadHardwareData() {
  std::optional<HardwareData> output;

  static const std::string delimiter = " ";

  // Send the command to read the hardware data.
  const std::string response = SendMsg("h");

  const size_t del_pos_1 = response.find(delimiter);
  const size_t del_pos_2 = response.find(delimiter, del_pos_1 + delimiter.length());
  const size_t del_pos_3 = response.find(delimiter, del_pos_2 + delimiter.length());
  const size_t del_pos_4 = response.find(delimiter, del_pos_3 + delimiter.length());
  
  // Check if all delimiters were found
  if (del_pos_1 == std::string::npos || del_pos_2 == std::string::npos || del_pos_3 == std::string::npos ||
      del_pos_4 == std::string::npos) {
      std::cerr << "Error: Not enough delimiters found in response. msg: " << response << std::endl;
      return std::nullopt;
  }

  const std::string token_1 = response.substr(0, del_pos_1);
  const std::string token_2 = response.substr(del_pos_1 + delimiter.length(), 
                                            del_pos_2 - del_pos_1 - delimiter.length());
  const std::string token_3 = response.substr(del_pos_2 + delimiter.length(), 
                                            del_pos_3 - del_pos_2 - delimiter.length());
  const std::string token_4 = response.substr(del_pos_3 + delimiter.length(), 
                                            del_pos_4 - del_pos_3 - delimiter.length());
  const std::string token_5 = response.substr(del_pos_4 + delimiter.length());

  size_t pos1, pos2, pos3, pos4, pos5;
  int left_encoder_value = std::stoi(token_1, &pos1);
  if (pos1 != token_1.length()) {
    std::cerr << "Error parsing token_1: " << token_1 << std::endl;
    return std::nullopt;
  }

  int right_encoder_value = std::stoi(token_2, &pos2);
  if(pos2 != token_2.length()) {
    std::cerr << "Error parsing token_2: " << token_2 << std::endl;
    std::cerr << "pos2: " << pos2 << "token_2.length(): " << token_2.length() << std::endl;
    return std::nullopt;
  }

  int left_bumper_value = std::stoi(token_3, &pos3);
  if(pos3 != token_3.length()) {
    std::cerr << "Error parsing token_3: " << token_3 << std::endl;
    std::cerr << "pos3: " << pos3 << "token_3.length(): " << token_3.length() << std::endl;
    return std::nullopt;
  }

  int right_bumper_value = std::stoi(token_4, &pos4);
  if(pos4 != token_4.length()) {
    std::cerr << "Error parsing token_4: " << token_4 << std::endl;
    std::cerr << "pos4: " << pos4 << "token_4.length(): " << token_4.length() << std::endl;
    return std::nullopt;
  } 

  int button_value = std::stoi(token_5, &pos5);
  // -2 because of /r/n
  if(pos5 != token_5.length() - 2) {
    std::cerr << "Error parsing token_5: " << token_5 << std::endl;
    std::cerr << "pos5: " << pos5 << "token_5.length(): " << token_5.length() << std::endl;
    return std::nullopt;
  } 

  output = HardwareData{left_encoder_value, right_encoder_value, left_bumper_value, right_bumper_value, button_value};
  return output;
}

void MotorDriver::SetMotorValues(int val_1, int val_2) {
  std::stringstream ss;
  ss << "m " << val_1 << " " << val_2;
  SendMsg(ss.str());
}

void MotorDriver::SetPidValues(float k_p, float k_d, float k_i, float k_o) {
  std::stringstream ss;
  ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o;
  SendMsg(ss.str());
}

std::string MotorDriver::SendMsg(const std::string& msg) {
  // Add carriage return to the message.
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string msg_to_send = msg + '\r';

  // Send the message.
  serial_port_.Write(msg_to_send);

  // Get response from the motor driver.
  std::string response;
  try {
    serial_port_.ReadLine(response, '\n', timeout_ms_);
  } catch (LibSerial::ReadTimeout&) {
    std::cerr << "Response to " << msg << " timed out." << std::endl;
  }
  return response;
}

}  // namespace kimchi_base
