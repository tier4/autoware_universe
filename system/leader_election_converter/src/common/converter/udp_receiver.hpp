
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

#ifndef COMMON__CONVERTER__UDP_RECEIVER_HPP_
#define COMMON__CONVERTER__UDP_RECEIVER_HPP_

#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>

namespace leader_election_converter
{

template <typename T>
class UdpReceiver
{
public:
  using CallbackType = std::function<void(const T &)>;
  UdpReceiver(const std::string & ip, const std::string & port);
  UdpReceiver(const std::string & ip, const std::string & port, CallbackType callback);
  UdpReceiver(const std::string & ip, const std::string & port, bool is_non_blocking);
  UdpReceiver(
    const std::string & ip, const std::string & port, bool is_non_blocking, CallbackType callback);
  ~UdpReceiver();

  bool receive(T & data);  // for non callback
  void receive();          // for callback

private:
  int socketfd_;
  struct addrinfo * res_;
  CallbackType callback_;

  void setCallback(CallbackType callback);
};

template <typename T>
UdpReceiver<T>::UdpReceiver(const std::string & ip, const std::string & port)
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

  if (bind(socketfd_, res_->ai_addr, res_->ai_addrlen) < 0) {
    freeaddrinfo(res_);
    close(socketfd_);
    throw std::runtime_error("bind failed");
  }
}

template <typename T>
UdpReceiver<T>::UdpReceiver(const std::string & ip, const std::string & port, CallbackType callback)
: UdpReceiver(ip, port)
{
  setCallback(callback);
}

template <typename T>
UdpReceiver<T>::UdpReceiver(const std::string & ip, const std::string & port, bool is_non_blocking)
: UdpReceiver(ip, port)
{
  if (is_non_blocking) {
    if (fcntl(socketfd_, F_SETFL, O_NONBLOCK) < 0) {
      freeaddrinfo(res_);
      close(socketfd_);
      throw std::runtime_error("fcntl failed");
    }
  }
}

template <typename T>
UdpReceiver<T>::UdpReceiver(
  const std::string & ip, const std::string & port, bool is_non_blocking, CallbackType callback)
: UdpReceiver(ip, port, is_non_blocking)
{
  setCallback(callback);
}

template <typename T>
UdpReceiver<T>::~UdpReceiver()
{
  shutdown(socketfd_, SHUT_RDWR);
  freeaddrinfo(res_);
  close(socketfd_);
}

template <typename T>
void UdpReceiver<T>::setCallback(CallbackType callback)
{
  callback_ = callback;
}

template <typename T>
bool UdpReceiver<T>::receive(T & data)
{
  struct sockaddr_storage addr;
  socklen_t addr_len = sizeof(addr);
  ssize_t recv_size = recvfrom(socketfd_, &data, sizeof(T), 0, (struct sockaddr *)&addr, &addr_len);
  if (recv_size < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return false;
    }
    throw std::runtime_error("recvfrom failed");
  }
  return true;
}

template <typename T>
void UdpReceiver<T>::receive()
{
  T data;
  if (receive(data) && callback_) {
    callback_(data);
  }
}

}  // namespace leader_election_converter

#endif  // COMMON__CONVERTER__UDP_RECEIVER_HPP_
