#pragma once
#include <boost/date_time/posix_time/posix_time_config.hpp>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <poll.h>

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>

namespace SocketHelpers {
inline int read_varint(boost::asio::ip::tcp::socket &socket) {
  // Read a varint from the socket
  std::vector<char> buffer(1);
  int result = 0;
  int shift = 0;
  do {
    boost::asio::read(socket, boost::asio::buffer(buffer, 1));
    result |= (buffer[0] & 0x7f) << shift;
    shift += 7;
  } while (buffer[0] & 0x80);

  return result;
}
} // namespace SocketHelpers
