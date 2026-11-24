// =============================================================================
// mqtt_client.hpp — C++ port of execution/mqtt_client.py
// =============================================================================

#ifndef EXECUTION__MQTT_CLIENT_HPP_
#define EXECUTION__MQTT_CLIENT_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

struct mosquitto_message;

namespace execution
{

using AnyMap = std::map<std::string, YAML::Node>;

// Python: execution/mqtt_client.py — class MQTTFleetClient
class MQTTFleetClient
{
public:
  MQTTFleetClient(
    const std::string & broker_host,
    int broker_port,
    const std::string & client_id = "rmf_fleet_adapter");

  MQTTFleetClient(const MQTTFleetClient &) = delete;
  MQTTFleetClient & operator=(const MQTTFleetClient &) = delete;
  MQTTFleetClient(MQTTFleetClient &&) noexcept;
  MQTTFleetClient & operator=(MQTTFleetClient &&) noexcept;
  ~MQTTFleetClient();

  bool start();
  void stop();

  std::map<std::string, AnyMap> get_robot_states();
  std::map<std::string, AnyMap> get_vda_connection_states();
  std::vector<AnyMap> get_and_clear_load_status_events();
  std::vector<AnyMap> get_and_clear_nav_status_events();

  bool publish_vda5050_order(
    const std::string & robot_name,
    const AnyMap & payload,
    const std::string & interface_name = "uagv",
    const std::string & major_version = "v3",
    const std::string & manufacturer = "Armstrong");

  bool publish_vda5050_instant_actions(
    const std::string & robot_name,
    const AnyMap & payload,
    const std::string & interface_name = "uagv",
    const std::string & major_version = "v3",
    const std::string & manufacturer = "Armstrong");

  static AnyMap build_vda5050_instant_actions_payload(
    const std::string & serial_number,
    int header_id,
    const std::vector<AnyMap> & actions,
    const std::string & manufacturer = "Armstrong",
    const std::string & version = "3.0.0");

  bool is_connected() const;

  void handle_mqtt_connect(int rc);
  void handle_mqtt_disconnect(int rc);
  void handle_mqtt_message(const struct mosquitto_message * msg);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  static std::optional<int> loads_to_load_status(const YAML::Node & loads);
  static std::optional<int> vda_operating_mode_to_int(const YAML::Node & operating_mode);
  void append_load_status_change_event(
    const std::string & amr_id,
    const std::optional<int> & load_status,
    const std::string & order_id,
    const YAML::Node & payload);
};

}  // namespace execution

#endif  // EXECUTION__MQTT_CLIENT_HPP_
