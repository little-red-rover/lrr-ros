#include "lrr_base/lrr_connection.h"

#include <arpa/inet.h>
#include <cstdio>
#include <netinet/in.h>
#include <sys/socket.h>

#include "lrr_base/socket_helpers.h"
#include "ros/init.h"

#include "messages.pb.h"

namespace lrr_base {
LRRConnection::LRRConnection(void (*callback)(OutgoingData &data))
    : callback_(callback) {
  // Create the TCP client
  socket_ = socket(AF_INET, SOCK_STREAM, 0);

  // Connect to the rover
  if (connect_() < 0) {
    std::fprintf(stderr,
                 "Connection to rover failed with unrecoverable error :(\n");
  };

  // Start a thread to poll for messages
  recv_thread_ = std::thread(&LRRConnection::recv_loop_, this);
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

  return 0;
}

void LRRConnection::send(IncomingCommand cmd) {
  size_t size = cmd.ByteSizeLong();
  void *buffer = malloc(size);

  // TODO varint delimited
  cmd.SerializeToArray(buffer, size);

  SocketHelpers::send_all(socket_, buffer, size);

  free(buffer);
}

void LRRConnection::recv_loop_() {
  while (ros::ok()) {
    printf("hello from thread\n");
    sleep(1);
  }
}
} // namespace lrr_base
