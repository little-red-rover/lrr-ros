#pragma once

#include <ros/ros.h>
#include <thread>

#include "messages.pb.h"

namespace lrr_base {
class LRRConnection {
public:
  LRRConnection(void (*callback)(OutgoingData &data));
  ~LRRConnection();

  void send(IncomingCommand cmd);

private:
  void recv_loop_();
  void (*callback_)(OutgoingData &data);
  std::thread recv_thread_;

  int socket_;

  int connect_();

  void *send_buffer_;
};
} // namespace lrr_base
