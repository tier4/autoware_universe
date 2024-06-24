// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMMON__CONVERTER__UDP_SENDER_HPP_
#define COMMON__CONVERTER__UDP_SENDER_HPP_

#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace leader_election_converter
{

template <typename T>
class UdpSender
{
public:
  UdpSender(const std::string & ip, const std::string & port);
  ~UdpSender();

  void send(const T & data);

private:
  int socketfd_;
  struct addrinfo * res_;
};

template <typename T>
UdpSender<T>::UdpSender(const std::string & ip, const std::string & port)
{
  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_DGRAM;

  if (getaddrinfo(ip.c_str(), port.c_str(), &hints, &res_) != 0) {
    throw std::runtime_error("getaddrinfo failed");
  }

  socketfd_ = socket(res_->ai_family, res_->ai_socktype, res_->ai_protocol);
  if (socketfd_ < 0) {
    freeaddrinfo(res_);
    throw std::runtime_error("socket failed");
  }
}

template <typename T>
UdpSender<T>::~UdpSender()
{
  shutdown(socketfd_, SHUT_RDWR);
  freeaddrinfo(res_);
  close(socketfd_);
}

template <typename T>
void UdpSender<T>::send(const T & data)
{
  if (sendto(socketfd_, &data, sizeof(T), 0, res_->ai_addr, res_->ai_addrlen) < 0) {
    throw std::runtime_error("sendto failed");
  }
}

}  // namespace leader_election_converter

#endif  // COMMON__CONVERTER__UDP_SENDER_HPP_
