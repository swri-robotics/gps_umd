// *****************************************************************************
//
// Copyright (c) 2026, Southwest Research Institute® (SwRI®)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef FAKE_GPSD_HPP_
#define FAKE_GPSD_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <stdexcept>
#include <string>

namespace gpsd_client
{
namespace test
{

/* A stand-in for the GPSd daemon: a loopback socket that speaks just enough
 * of GPSd's line protocol for libgps to connect to, read what gpsd_client
 * sends, and hand it reports.
 *
 * gpsfake plays the same part more faithfully, but needs a GPSd build that
 * the buildfarm and most CI jobs do not have. This needs nothing, so the
 * node's own behavior -- its parameters, topics and connection handling --
 * is tested everywhere. Reports are written by the test, so each one can be
 * exactly what a test needs.
 *
 * Single-threaded: every wait polls with a timeout, so a test that goes wrong
 * fails rather than hangs.
 */
class FakeGpsd
{
public:
  FakeGpsd()
  {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      throw std::runtime_error("socket() failed");
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;  // any free port
    socklen_t length = sizeof(address);
    if (bind(listen_fd_, reinterpret_cast<sockaddr *>(&address), length) != 0 ||
      listen(listen_fd_, 1) != 0 ||
      getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&address), &length) != 0)
    {
      close(listen_fd_);
      throw std::runtime_error("could not listen on a loopback port");
    }
    port_ = ntohs(address.sin_port);
  }

  ~FakeGpsd()
  {
    disconnect();
    close(listen_fd_);
  }

  FakeGpsd(const FakeGpsd &) = delete;
  FakeGpsd & operator=(const FakeGpsd &) = delete;

  int port() const
  {
    return port_;
  }

  /// Accept the client's connection. The kernel completes it as soon as the
  /// client connects, so this can run after gps_open() has returned.
  bool acceptClient(std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    if (client_fd_ >= 0) {
      return true;
    }
    if (!readable(listen_fd_, timeout)) {
      return false;
    }
    client_fd_ = accept(listen_fd_, nullptr, nullptr);
    return client_fd_ >= 0;
  }

  /// Send one report, as GPSd does: a JSON object on a CRLF-terminated line.
  /// Several reports joined by newlines go out in one write.
  bool send(const std::string & reports)
  {
    std::string data = reports;
    if (data.empty() || data.back() != '\n') {
      data += "\r\n";
    }
    size_t sent = 0;
    while (sent < data.size()) {
      const ssize_t n = ::send(client_fd_, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
      if (n <= 0) {
        return false;
      }
      sent += static_cast<size_t>(n);
    }
    return true;
  }

  /// Read what the client sends until it includes `text`, and return all of
  /// it. Empty if the text never arrives.
  std::string waitFor(
    const std::string & text, std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (received_.find(text) == std::string::npos) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      if (left.count() <= 0 || !readMore(left)) {
        return "";
      }
    }
    std::string all;
    all.swap(received_);
    return all;
  }

  /// True once the client has closed its end of the connection.
  bool waitForClose(std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
      const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      if (left.count() <= 0 || !readable(client_fd_, left)) {
        return false;
      }
      char buffer[256];
      const ssize_t n = recv(client_fd_, buffer, sizeof(buffer), 0);
      if (n <= 0) {
        return true;
      }
      received_.append(buffer, static_cast<size_t>(n));
    }
  }

  /// Drop the client, as a GPSd that exits or restarts would.
  void disconnect()
  {
    if (client_fd_ >= 0) {
      close(client_fd_);
      client_fd_ = -1;
    }
  }

private:
  static bool readable(int fd, std::chrono::milliseconds timeout)
  {
    pollfd poll_fd{fd, POLLIN, 0};
    return poll(&poll_fd, 1, static_cast<int>(timeout.count())) > 0;
  }

  bool readMore(std::chrono::milliseconds timeout)
  {
    if (client_fd_ < 0 || !readable(client_fd_, timeout)) {
      return false;
    }
    char buffer[256];
    const ssize_t n = recv(client_fd_, buffer, sizeof(buffer), 0);
    if (n <= 0) {
      return false;
    }
    received_.append(buffer, static_cast<size_t>(n));
    return true;
  }

  int listen_fd_{-1};
  int client_fd_{-1};
  int port_{0};
  std::string received_;
};

}  // namespace test
}  // namespace gpsd_client

#endif  // FAKE_GPSD_HPP_
