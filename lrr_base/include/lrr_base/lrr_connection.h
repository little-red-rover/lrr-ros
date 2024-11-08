#pragma once

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <ros/ros.h>
#include <thread>

#include "messages.pb.h"

namespace lrr_base {
class LRRConnection {
public:
  LRRConnection(std::function<void(OutgoingData &)> callback,
                OutgoingMessageID subscription);
  ~LRRConnection();

  void send(IncomingCommand cmd);

private:
  void main_thread_();
  std::thread main_thread_handle_;

  std::function<void(OutgoingData &)> callback_;

  OutgoingMessageID subscription_;

  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_;

  void handle_connect_(const boost::system::error_code &err);
  void handle_read_(const boost::system::error_code &err);

  void *send_buffer_;
};
} // namespace lrr_base
