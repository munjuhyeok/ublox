#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <ublox_gps/node.hpp>
#include "ntrip/ntrip_client.h"
#include <thread>

bool run_ntrip_client(std::shared_ptr<ublox_node::UbloxNode> ublox_node){
  bool run_rtcm;
  ublox_node->get_parameter("rtcm", run_rtcm);
  if (!run_rtcm)
    return false;


  libntrip::NtripClient ntrip_client;
  // below is for ntrip client
  std::string server_url = "gnssdata.or.kr";
  int port = 2101;
  std::string user = "munjuhyeok@gmail.com";
  std::string passwd = "gnss";
  std::string mountpoint = "SEJN-RTCM32";

  ntrip_client.Init(server_url, port, user, passwd, mountpoint);
  ntrip_client.OnReceived([ublox_node] (const char *buffer, int size) {
    std::vector<uint8_t> message(buffer, buffer + size);
    ublox_node->sendRtcm(message);
  });
  // std::thread ntrip_client_thread(&libntrip::NtripClient::Run, std::ref(ntrip_client));
  ntrip_client.Run();
  std::this_thread::sleep_for(std::chrono::seconds(1));  // Maybe take longer?
  while (ntrip_client.service_is_running()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  std::cout<<"sibal2"<<std::endl;
  return true;
}

int main(int argc, char** argv) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions nodeOptions;
  std::shared_ptr<ublox_node::UbloxNode> ublox_node = std::make_shared<ublox_node::UbloxNode>(nodeOptions);

  std::thread ntrip_client_thread(run_ntrip_client, ublox_node);

  rclcpp::spin(ublox_node);

  rclcpp::shutdown();
  
  return 0;
}
