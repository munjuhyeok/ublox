// MIT License
//
// Copyright (c) 2021 Yuming Meng
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef NTRIPLIB_NTRIP_CLIENT_H_
#define NTRIPLIB_NTRIP_CLIENT_H_

#if defined(WIN32) || defined(_WIN32)
#include <winsock2.h>
#endif  // defined(WIN32) || defined(_WIN32)

#include <atomic>
#include <string>
#include <thread>  // NOLINT.
#include <functional>

#include "./thread_raii.h"


namespace libntrip {

using ClientCallback = std::function<void (char const* _buffer, int _size)>;

class NtripClient {
 public:
  NtripClient() = default;
  NtripClient(NtripClient const&) = delete;
  NtripClient(NtripClient&&) = delete;
  NtripClient& operator=(NtripClient const&) = delete;
  NtripClient& operator=(NtripClient&&) = delete;
  NtripClient(std::string server_url, int port,
      std::string const& user, std::string const& passwd,
      std::string const& mountpoint) :
        server_url_(server_url), server_port_(port),
        user_(user), passwd_(passwd),
        mountpoint_(mountpoint) { }
  ~NtripClient() { Stop(); }

  void Init(std::string server_url, int port,
      std::string const& user, std::string const& passwd,
      std::string const& mountpoint) {
    server_url_ = server_url;
    server_port_ = port;
    user_ = user;
    passwd_ = passwd;
    mountpoint_ = mountpoint;
  }

  void set_gga_buffer(std::string const& gga_buffer) {
    gga_buffer_ = gga_buffer;
    gga_is_update_.store(true);
  }

  void set_location(double latitude, double longitude) {
    latitude_ = latitude;
    longitude_ = longitude;
  }
  
  void set_report_interval(int intv) {
    report_interval_ = intv;
  }

  void OnReceived(const ClientCallback &callback) { callback_ = callback; }
  bool Run(void);
  void Stop(void);
  bool service_is_running(void) const {
    return service_is_running_.load();
  }

 private:
  // Thread handler.
  void ThreadHandler(void);

  std::atomic_bool service_is_running_ = {false};
  std::atomic_bool gga_is_update_ = {false};
  int report_interval_ = 1;
  double latitude_ = 36.372;
  double longitude_ = 127.363;
  std::string server_url_;
  int server_port_ = 8090;
  std::string user_;
  std::string passwd_;
  std::string mountpoint_;
  std::string gga_buffer_;
#if defined(WIN32) || defined(_WIN32)
  SOCKET socket_fd_ = INVALID_SOCKET;
#else
  int socket_fd_ = -1;
#endif  // defined(WIN32) || defined(_WIN32)
  Thread thread_;
  ClientCallback callback_ = [] (char const*, int) -> void {};
};

}  // namespace libntrip

#endif  // NTRIPLIB_NTRIP_CLIENT_H_
