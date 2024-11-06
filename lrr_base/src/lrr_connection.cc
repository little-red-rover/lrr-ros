#include "lrr_base/lrr_connection.h"

#include <arpa/inet.h>
#include <cstdio>
#include <google/protobuf/message.h>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>

#include "lrr_base/socket_helpers.h"
#include "ros/init.h"

#include <google/protobuf/io/coded_stream.h>

#include <google/protobuf/util/delimited_message_util.h>

#include "messages.pb.h"

namespace lrr_base {
LRRConnection::LRRConnection(void (*callback)(OutgoingData &data),
                             OutgoingMessageID subscription)
    : callback_(callback), subscription_(subscription) {
  // Create the TCP client
  socket_ = socket(AF_INET, SOCK_STREAM, 0);

  // Start the main thread
  main_thread_ = std::thread(&LRRConnection::thread_main_, this);

  main_thread_.detach();
}

void LRRConnection::thread_main_() {
  // Connect to the rover
  if (connect_() < 0) {
    std::fprintf(stderr,
                 "Connection to rover failed with unrecoverable error :(\n");
  };

  recv_thread_ = std::thread(&LRRConnection::recv_loop_, this);

  // Start a thread to poll for messages
  recv_thread_.join();
}

LRRConnection::~LRRConnection() { close(socket_); }

int LRRConnection::connect_() {
  // Set the server address
  // 192.168.4.1 is the default gateway address of an ESP in access point mode
  // Port 8001 is arbitrarily chosen for LRR
  sockaddr_in robotAddr;
  robotAddr.sin_family = AF_INET;
  robotAddr.sin_port = htons(8001);
  if (inet_pton(AF_INET, "192.168.4.1", &robotAddr.sin_addr) <= 0) {
    std::cerr << "Invalid address / Address not supported." << std::endl;
    return -1;
  }

  while (ros::ok() && SocketHelpers::connect_with_timeout(
                          socket_, (struct sockaddr *)&robotAddr,
                          sizeof(robotAddr), 1000) < 0) {
    std::printf("Could not connect to rover. Retrying...\n");
    sleep(1);
  };

  if (subscription_ != NONE) {
    IncomingCommand cmd;
    cmd.mutable_subscribe_request()->set_msg_id(subscription_);
    send(cmd);
  }

  std::printf("Socket connected\n");

  return 0;
}

void LRRConnection::send(IncomingCommand cmd) {
  // Serialize the message, delimited by its length
  std::ostringstream stream;
  google::protobuf::util::SerializeDelimitedToOstream(cmd, &stream);
  std::string as_text = stream.str();
  const void *buff = reinterpret_cast<const void *>(as_text.c_str());
  size_t size = as_text.size();

  // Send to rover
  SocketHelpers::send_all(socket_, buff, size);
}

void LRRConnection::recv_loop_() {
  // Setup to wait for socket to be readable
  struct pollfd fds[1]{{.fd = socket_, .events = POLLIN}};

  while (ros::ok()) {
    int ret = poll(fds, 1, -1);
    printf("Connection got message.\n");

    // Get message size from delimiter
    size_t size;

    // Read message

    // Call callback

    // callback_();
  }
}
} // namespace lrr_base
