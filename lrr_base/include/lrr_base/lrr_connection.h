#pragma once

#include <ros/ros.h>
#include <thread>

#include "messages.pb.h"

namespace lrr_base {
class LRRConnection {
public:
  LRRConnection(void (*callback)(OutgoingData &data),
                OutgoingMessageID subscription);
  ~LRRConnection();

  void send(IncomingCommand cmd);

private:
  void thread_main_();
  std::thread main_thread_;

  void recv_loop_();
  std::thread recv_thread_;

  void (*callback_)(OutgoingData &data);
  OutgoingMessageID subscription_;

  int socket_;

  int connect_();

  void *send_buffer_;
};
} // namespace lrr_base
