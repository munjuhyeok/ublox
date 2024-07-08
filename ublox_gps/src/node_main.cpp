#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <ublox_gps/node.hpp>
#include "ntrip/ntrip_client.h"
#include <thread>

int main(int argc, char** argv) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions nodeOptions;
  auto config_directory = ament_index_cpp::get_package_share_directory("ublox_gps") + "/config";
  auto base_params = config_directory + "/zed_f9p_base.yaml";
  auto rover_params = config_directory + "/zed_f9p_rover.yaml";
  nodeOptions.arguments({"--ros-args", "--params-file", base_params, "--ros-args", "-r", "__ns:=/base"});
  std::shared_ptr<ublox_node::UbloxNode> ublox_node_base = std::make_shared<ublox_node::UbloxNode>(nodeOptions);
  nodeOptions.arguments({"--ros-args", "--params-file", rover_params, "--ros-args", "-r", "__ns:=/rover"});
  std::shared_ptr<ublox_node::UbloxNode> ublox_node_rover = std::make_shared<ublox_node::UbloxNode>(nodeOptions);


  // below is for ntrip client
  std::string server_url = "gnssdata.or.kr";
  int port = 2101;
  std::string user = "munjuhyeok@gmail.com";
  std::string passwd = "gnss";
  std::string mountpoint = "SEJN-RTCM32";

  libntrip::NtripClient ntrip_client;
  ntrip_client.Init(server_url, port, user, passwd, mountpoint);
  ntrip_client.OnReceived([ublox_node_base] (const char *buffer, int size) {
    std::vector<uint8_t> message(buffer, buffer + size);
    ublox_node_base->sendRtcm(message);
  });
  std::thread ntrip_client_thread(&libntrip::NtripClient::Run, std::ref(ntrip_client));

  std::this_thread::sleep_for(std::chrono::seconds(1));  // Maybe take longer?
  while (ntrip_client.service_is_running()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ublox_node_base);
  executor.add_node(ublox_node_rover);
  executor.spin();
  rclcpp::shutdown();

  ntrip_client_thread.join();
  
  return 0;
}
