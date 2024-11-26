#pragma once

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <ros/ros.h>
#include <thread>

#include "messages.pb.h"

namespace lrr_base {
class ConnectionParser {
public:
  virtual void parse(OutgoingData &data) = 0;
  virtual ~ConnectionParser() = default;
};

class LRRConnection {
public:
  LRRConnection(ConnectionParser *connection_parser,
                OutgoingMessageID subscription);
  ~LRRConnection();

  void send(IncomingCommand cmd);

  void connect();

private:
  void main_thread_();
  std::thread main_thread_handle_;

  ConnectionParser *connection_parser_;

  OutgoingMessageID subscription_;

  boost::asio::io_context io_context_;
  boost::asio::ip::tcp::socket socket_;

  void handle_connect_(const boost::system::error_code &err);
  void handle_read_(const boost::system::error_code &err);

  void *send_buffer_;

  bool connected_;
};
} // namespace lrr_base
