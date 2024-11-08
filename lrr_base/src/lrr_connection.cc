#include "lrr_base/lrr_connection.h"

#include <arpa/inet.h>
#include <boost/asio/completion_condition.hpp>
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

namespace lrr_base {
LRRConnection::LRRConnection(std::function<void(OutgoingData &)> callback,
                             OutgoingMessageID subscription)
    : callback_(callback), subscription_(subscription),
      socket_(boost::asio::ip::tcp::socket(io_context_)) {
  // Start main thread
  main_thread_handle_ = std::thread(&LRRConnection::main_thread_, this);
}

LRRConnection::~LRRConnection() {
  socket_.shutdown(boost::asio::socket_base::shutdown_both);
  socket_.close();
  main_thread_handle_.join();
}

void LRRConnection::main_thread_() {
  // Connect to rover
  boost::asio::ip::tcp::endpoint endpoint(
      boost::asio::ip::address::from_string("192.168.4.1"), 8001);
  socket_.async_connect(endpoint,
                        boost::bind(&LRRConnection::handle_connect_, this,
                                    boost::asio::placeholders::error));

  // Start read callback
  socket_.async_read_some(boost::asio::null_buffers(),
                          boost::bind(&LRRConnection::handle_read_, this,
                                      boost::asio::placeholders::error));

  io_context_.run();
}

void LRRConnection::handle_connect_(const boost::system::error_code &err) {
  if (err) {
    std::fprintf(stderr, "Got error: %d\n", err.value());
  }

  if (subscription_ != NONE) {
    IncomingCommand cmd;
    cmd.mutable_subscribe_request()->set_msg_id(subscription_);
    send(cmd);
  }
}

void LRRConnection::send(IncomingCommand cmd) {
  // Serialize
  boost::asio::streambuf b;
  std::ostream stream(&b);
  google::protobuf::util::SerializeDelimitedToOstream(cmd, &stream);

  // Send over socket
  boost::asio::write(socket_, b);
}

void LRRConnection::handle_read_(const boost::system::error_code &err) {

  // Get message size from delimiter
  size_t size = SocketHelpers::read_varint(socket_);

  // Create input stream for message
  boost::asio::streambuf b;
  boost::asio::read(socket_, b, boost::asio::transfer_exactly(size));
  std::istream stream(&b);

  // Read message
  OutgoingData data;
  data.ParseFromIstream(&stream);

  // Call callback
  callback_(data);

  // Setup next read
  socket_.async_read_some(boost::asio::null_buffers(),
                          boost::bind(&LRRConnection::handle_read_, this,
                                      boost::asio::placeholders::error));
}
} // namespace lrr_base
