#include "lrr_base/lrr_connection.h"

#include <arpa/inet.h>
#include <boost/asio/completion_condition.hpp>
#include <boost/asio/error.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <cstdio>
#include <cwchar>
#include <netinet/in.h>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/message.h>
#include <google/protobuf/util/delimited_message_util.h>

#include <boost/asio/socket_base.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include "lrr_base/socket_helpers.h"
#include "messages.pb.h"
#include "ros/init.h"

namespace lrr_base {
LRRConnection::LRRConnection(ConnectionParser *connection_parser,
                             OutgoingMessageID subscription)
    : connection_parser_(connection_parser), subscription_(subscription),
      socket_(boost::asio::ip::tcp::socket(io_context_)), connected_(false) {
  // Start main thread
  main_thread_handle_ = std::thread(&LRRConnection::main_thread_, this);

  // struct timeval tv;
  // tv.tv_sec = 1000;
  // tv.tv_usec = 0;
  // setsockopt(socket_.native_handle(), SOL_SOCKET, SO_RCVTIMEO, &tv,
  // sizeof(tv)); setsockopt(socket_.native_handle(), SOL_SOCKET, SO_SNDTIMEO,
  // &tv, sizeof(tv));
}

LRRConnection::~LRRConnection() {
  if (socket_.is_open()) {
    socket_.shutdown(boost::asio::socket_base::shutdown_both);
    socket_.close();
  }
  main_thread_handle_.join();
}

void LRRConnection::main_thread_() {
  // Connect to rover
  std::printf("Attempting to connect to rover:\n");

  boost::asio::ip::tcp::endpoint endpoint(
      boost::asio::ip::address::from_string("192.168.4.1"),
      8001); // TODO: Make IP a parameter

  socket_.async_connect(endpoint,
                        boost::bind(&LRRConnection::handle_connect_, this,
                                    boost::asio::placeholders::error));

  // Start read callback
  socket_.async_read_some(boost::asio::null_buffers(),
                          boost::bind(&LRRConnection::handle_read_, this,
                                      boost::asio::placeholders::error));

  boost::asio::io_service io_service;
  while (ros::ok()) {
    try {
      io_context_.run_one();
    } catch (boost::system::system_error &e) {
      if (e.code().value() == boost::system::errc::broken_pipe ||
          e.code().value() == boost::asio::error::eof ||
          e.code().value() == boost::system::errc::connection_reset ||
          e.code().value() == boost::system::errc::network_unreachable) {

        std::printf("Connection with rover dropped. Reconnecting...:\n");
        connected_ = false;

        try {
          socket_.shutdown(boost::asio::socket_base::shutdown_both);
          socket_.close();
        } catch (boost::system::system_error) {
          socket_.close();
        }

        boost::asio::ip::tcp::endpoint endpoint(
            boost::asio::ip::address::from_string("192.168.4.1"), 8001);
        socket_.async_connect(endpoint,
                              boost::bind(&LRRConnection::handle_connect_, this,
                                          boost::asio::placeholders::error));

        socket_.async_read_some(boost::asio::null_buffers(),
                                boost::bind(&LRRConnection::handle_read_, this,
                                            boost::asio::placeholders::error));
      } else if (e.code().value() == boost::system::errc::bad_file_descriptor) {
        std::printf("Closing connection to rover.\n");
        return;
      } else {
        std::printf("Unhandled error code: %s\n", e.what());
        return;
      }
    }
  }
}

void LRRConnection::handle_connect_(const boost::system::error_code &err) {
  if (err) {
    return;
  }

  std::printf("Successfully connected to rover.\n");
  connected_ = true;

  if (subscription_ != NONE) {
    IncomingCommand cmd;
    cmd.mutable_subscribe_request()->set_msg_id(subscription_);
    send(cmd);
  }
}

void LRRConnection::send(IncomingCommand cmd) {
  // Check if the connection is live
  if (!connected_) {
    return;
  }

  // Serialize
  boost::asio::streambuf b;
  std::ostream stream(&b);
  google::protobuf::util::SerializeDelimitedToOstream(cmd, &stream);

  // Send over socket
  try {
    boost::asio::write(socket_, b);
  } catch (boost::system::system_error) {
  };
}

void LRRConnection::handle_read_(const boost::system::error_code &err) {
  if (err) {
    return;
  }

  // Get message size from delimiter
  size_t size = SocketHelpers::read_varint(socket_);

  if (size == 0) {
    printf("Got EOF\n");
    return;
  }

  // Start the next read
  socket_.async_read_some(boost::asio::null_buffers(),
                          boost::bind(&LRRConnection::handle_read_, this,
                                      boost::asio::placeholders::error));

  // Create input stream for message
  boost::asio::streambuf b;
  size_t num_read =
      boost::asio::read(socket_, b, boost::asio::transfer_exactly(size));
  assert(num_read == size);
  std::istream stream(&b);

  // Read message
  OutgoingData data;
  data.ParseFromIstream(&stream);

  // Call callback
  connection_parser_->parse(data);
}
} // namespace lrr_base
