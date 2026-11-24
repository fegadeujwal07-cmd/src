// C++ port of execution/mqtt_client.py

#include "execution/mqtt_client.hpp"

#include <mosquitto.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <vector>

namespace execution
{

namespace
{

std::string yaml_scalar_as_string(const YAML::Node & node)
{
  if (!node || node.IsNull()) {
    return "";
  }
  try {
    return node.as<std::string>();
  } catch (...) {
  }
  try {
    return std::to_string(node.as<int>());
  } catch (...) {
  }
  try {
    return std::to_string(node.as<double>());
  } catch (...) {
  }
  return "";
}

// [VELOCITY] Safe numeric read: returns fallback when the node is missing, null,
// not a number, or NaN/inf, so bad data never reaches fleet state.
double yaml_scalar_as_finite_double(const YAML::Node & node, double fallback = 0.0)
{
  if (!node || node.IsNull() || !node.IsScalar()) {
    return fallback;
  }
  try {
    const double value = node.as<double>();
    return std::isfinite(value) ? value : fallback;
  } catch (...) {
  }
  try {
    const double value = std::stod(node.as<std::string>());
    return std::isfinite(value) ? value : fallback;
  } catch (...) {
  }
  return fallback;
}

double unix_timestamp_now()
{
  return std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string utc_iso_timestamp_millis()
{
  const auto now = std::chrono::system_clock::now();
  const auto ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::gmtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%FT%T") << "." << std::setfill('0') << std::setw(3) <<
    (ms.count() % 1000) << "Z";
  return oss.str();
}

std::vector<std::string> split_mqtt_topic(const std::string & topic)
{
  std::vector<std::string> parts;
  std::stringstream ss(topic);
  std::string item;
  while (std::getline(ss, item, '/')) {
    if (!item.empty()) {
      parts.push_back(item);
    }
  }
  return parts;
}

YAML::Node yaml_map_or_empty(const YAML::Node & node)
{
  if (node && node.IsMap()) {
    return node;
  }
  return YAML::Node(YAML::NodeType::Map);
}

std::string yaml_value_to_json(const YAML::Node & node);

std::string yaml_node_to_json_string(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    return "";
  }
  std::ostringstream oss;
  oss << "{";
  bool first = true;
  for (const auto & kv : node) {
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << '"' << kv.first.as<std::string>() << "\": " << yaml_value_to_json(kv.second);
  }
  oss << "}";
  return oss.str();
}

std::string yaml_value_to_json(const YAML::Node & node)
{
  if (!node || node.IsNull()) {
    return "null";
  }
  if (node.IsMap()) {
    return yaml_node_to_json_string(node);
  }
  if (node.IsSequence()) {
    std::ostringstream oss;
    oss << "[";
    bool first = true;
    for (const auto & item : node) {
      if (!first) {
        oss << ", ";
      }
      first = false;
      oss << yaml_value_to_json(item);
    }
    oss << "]";
    return oss.str();
  }
  try {
    const bool value = node.as<bool>();
    return value ? "true" : "false";
  } catch (...) {
  }
  try {
    return std::to_string(node.as<int>());
  } catch (...) {
  }
  try {
    std::ostringstream oss;
    oss << node.as<double>();
    return oss.str();
  } catch (...) {
  }
  try {
    const std::string value = node.as<std::string>();
    std::ostringstream oss;
    oss << '"';
    for (const char c : value) {
      if (c == '"' || c == '\\') {
        oss << '\\';
      }
      oss << c;
    }
    oss << '"';
    return oss.str();
  } catch (...) {
    return "null";
  }
}

std::string anymap_to_json_string(const AnyMap & map)
{
  YAML::Node root(YAML::NodeType::Map);
  for (const auto & [key, value] : map) {
    root[key] = value;
  }
  return yaml_value_to_json(root);
}

}  // namespace

struct MQTTFleetClient::Impl
{
  std::string broker_host;
  int broker_port{1883};
  std::string client_id;
  mosquitto * mosq{nullptr};
  bool connected{false};
  std::mutex events_mutex;
  std::map<std::string, AnyMap> amr_states;
  std::vector<AnyMap> load_status_events;
  std::map<std::string, std::optional<int>> last_vda_load_status_by_robot;
  std::map<std::string, AnyMap> vda_connection_by_robot;
};

void MQTTFleetClient::append_load_status_change_event(
  const std::string & amr_id,
  const std::optional<int> & load_status,
  const std::string & order_id,
  const YAML::Node & payload)
{
  // Python mqtt_client.py lines 57–86 — caller must hold impl_->events_mutex
  if (!impl_) {
    return;
  }
  const auto prev_it = impl_->last_vda_load_status_by_robot.find(amr_id);
  const std::optional<int> previous_status =
    (prev_it != impl_->last_vda_load_status_by_robot.end()) ? prev_it->second : std::nullopt;
  if (previous_status == load_status) {
    return;
  }
  impl_->last_vda_load_status_by_robot[amr_id] = load_status;
  if (!load_status.has_value()) {
    return;
  }

  const YAML::Node header_id = payload["headerId"];
  std::string event_id;
  if (!order_id.empty() && header_id && !header_id.IsNull()) {
    event_id = order_id + "_" + yaml_scalar_as_string(header_id);
  } else if (!order_id.empty()) {
    event_id = order_id;
  } else {
    event_id = amr_id + "_load_state";
  }

  AnyMap event;
  event["robot_id"] = YAML::Node(amr_id);
  event["id"] = YAML::Node(event_id);
  event["status"] = YAML::Node(*load_status);
  if (payload["timestamp"] && !payload["timestamp"].IsNull()) {
    event["timestamp"] = payload["timestamp"];
  } else {
    event["timestamp"] = YAML::Node(unix_timestamp_now());
  }
  if (!order_id.empty()) {
    event["base_request_id"] = YAML::Node(order_id);
  }
  impl_->load_status_events.push_back(std::move(event));
}

std::optional<int> MQTTFleetClient::vda_operating_mode_to_int(const YAML::Node & operating_mode)
{
  // Python mqtt_client.py lines 57–67
  if (!operating_mode || operating_mode.IsNull() || !operating_mode.IsScalar()) {
    return std::nullopt;
  }
  std::string normalized;
  try {
    normalized = operating_mode.as<std::string>();
  } catch (...) {
    return std::nullopt;
  }
  auto is_not_space = [](unsigned char c) { return !std::isspace(c); };
  auto begin = std::find_if(normalized.begin(), normalized.end(), is_not_space);
  if (begin == normalized.end()) {
    return std::nullopt;
  }
  auto end = std::find_if(normalized.rbegin(), normalized.rend(), is_not_space).base();
  normalized = std::string(begin, end);
  std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::toupper);
  if (normalized == "AUTOMATIC") {
    return 1;
  }
  if (normalized == "MANUAL") {
    return 0;
  }
  return std::nullopt;
}

std::optional<int> MQTTFleetClient::loads_to_load_status(const YAML::Node & loads)
{
  // Python mqtt_client.py lines 49–55
  if (!loads || loads.IsNull()) {
    return std::nullopt;
  }
  if (loads.IsSequence()) {
    return loads.size() > 0 ? 4 : 2;
  }
  return std::nullopt;
}

void MQTTFleetClient::handle_mqtt_connect(int rc)
{
  // Python mqtt_client.py lines 88–99
  if (!impl_ || !impl_->mosq) {
    return;
  }
  if (rc == 0) {
    impl_->connected = true;
    mosquitto_subscribe(impl_->mosq, nullptr, "amr/+/state", 0);
    mosquitto_subscribe(impl_->mosq, nullptr, "uagv/v3/Armstrong/+/state", 0);
    mosquitto_subscribe(impl_->mosq, nullptr, "uagv/v3/Armstrong/+/connection", 1);
    std::cout << "[MQTT] Connected and subscribed to state + VDA connection topics" << std::endl;
  } else {
    impl_->connected = false;
    std::cout << "[MQTT] Connect failed: " << rc << std::endl;
  }
}

void MQTTFleetClient::handle_mqtt_disconnect(int rc)
{
  // Python mqtt_client.py lines 101–103
  if (impl_) {
    impl_->connected = false;
  }
  std::cout << "[MQTT] Disconnected: " << rc << std::endl;
}

void MQTTFleetClient::handle_mqtt_message(const struct mosquitto_message * msg)
{
  // Python mqtt_client.py lines 105–215
  if (!impl_ || !msg || !msg->topic) {
    return;
  }

  try {
    const std::string topic = msg->topic;
    std::string payload_str;
    if (msg->payload && msg->payloadlen > 0) {
      payload_str.assign(static_cast<const char *>(msg->payload), static_cast<std::size_t>(msg->payloadlen));
    }

    YAML::Node payload;
    if (!payload_str.empty()) {
      payload = YAML::Load(payload_str);
    }
    if (!payload || !payload.IsMap()) {
      payload = YAML::Node(YAML::NodeType::Map);
    }

    const std::vector<std::string> parts = split_mqtt_topic(topic);
    if (parts.size() < 3) {
      return;
    }

    std::lock_guard<std::mutex> lock(impl_->events_mutex);

    if (parts[0] == "amr" && parts.size() >= 3 && parts[2] == "state") {
      const std::string amr_id = parts[1];
      YAML::Node state = payload["state"];
      if (!state || !state.IsMap()) {
        state = payload;
      }
      const YAML::Node state_map = yaml_map_or_empty(state);

      YAML::Node loads = payload["loads"];
      if (!loads || loads.IsNull()) {
        loads = state_map["loads"];
      }
      const std::optional<int> load_status = loads_to_load_status(loads);

      YAML::Node pose = state_map["pose"];
      if (!pose || !pose.IsMap()) {
        pose = YAML::Node(YAML::NodeType::Map);
      }
      YAML::Node twist = state_map["twist"];
      if (!twist || !twist.IsMap()) {
        twist = YAML::Node(YAML::NodeType::Map);
      }
      YAML::Node battery = state_map["battery"];
      if (!battery || !battery.IsMap()) {
        battery = YAML::Node(YAML::NodeType::Map);
      }

      AnyMap robot_state;
      robot_state["pose"] = pose;
      robot_state["twist"] = twist;
      robot_state["battery"] = battery;
      robot_state["status"] = YAML::Node(yaml_scalar_as_string(state_map["status"]).empty() ?
          "unknown" : yaml_scalar_as_string(state_map["status"]));
      robot_state["current_task"] = YAML::Node(yaml_scalar_as_string(state_map["current_task"]));
      robot_state["tf_data"] = state_map["tf_data"];
      if (load_status.has_value()) {
        robot_state["load_status"] = YAML::Node(*load_status);
      } else {
        robot_state["load_status"] = YAML::Node(YAML::NodeType::Null);
      }
      impl_->amr_states[amr_id] = std::move(robot_state);

      std::string order_id = yaml_scalar_as_string(payload["orderId"]);
      if (order_id.empty()) {
        order_id = yaml_scalar_as_string(state_map["orderId"]);
      }
      if (order_id.empty()) {
        order_id = yaml_scalar_as_string(state_map["current_task"]);
      }
      append_load_status_change_event(amr_id, load_status, order_id, payload);
      return;
    }

    if (parts[0] == "uagv" && parts.size() >= 5 && parts[2] == "Armstrong" && parts.back() == "state") {
      std::string amr_id = yaml_scalar_as_string(payload["serialNumber"]);
      if (amr_id.empty()) {
        amr_id = parts[parts.size() - 2];
      }

      const YAML::Node pos = yaml_map_or_empty(payload["mobileRobotPosition"]);
      const YAML::Node power = payload["powerSupply"];

      YAML::Node battery(YAML::NodeType::Map);
      if (power && power.IsMap() && power["stateOfCharge"]) {
        try {
          battery["percentage"] = power["stateOfCharge"].as<double>();
        } catch (...) {
          try {
            battery["percentage"] = std::stod(yaml_scalar_as_string(power["stateOfCharge"]));
          } catch (...) {
          }
        }
      }

      // [CHARGING] powerSupply.charging -> robot_state["charging"]:
      //   true / false  -> stored as a bool
      //   missing, null or any other value -> null (execution reports it to RMF
      //   as charging_status -1 = unknown; a charge task fails on unknown).
      YAML::Node charging(YAML::NodeType::Null);
      if (power && power.IsMap() && power["charging"] && !power["charging"].IsNull()) {
        std::string charging_text = yaml_scalar_as_string(power["charging"]);
        std::transform(
          charging_text.begin(), charging_text.end(), charging_text.begin(),
          [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (charging_text == "true") {
          charging = YAML::Node(true);
        } else if (charging_text == "false") {
          charging = YAML::Node(false);
        }
      }

      bool driving = false;
      if (payload["driving"] && !payload["driving"].IsNull()) {
        try {
          driving = payload["driving"].as<bool>();
        } catch (...) {
          driving = yaml_scalar_as_string(payload["driving"]) == "true";
        }
      }

      const std::string order_id = yaml_scalar_as_string(payload["orderId"]);
      const std::optional<int> load_status = loads_to_load_status(payload["loads"]);
      const std::optional<int> operating_mode = vda_operating_mode_to_int(payload["operatingMode"]);

      YAML::Node pose(YAML::NodeType::Map);
      pose["x"] = pos["x"] ? pos["x"].as<double>() : 0.0;
      pose["y"] = pos["y"] ? pos["y"].as<double>() : 0.0;
      pose["yaw"] = pos["theta"] ? pos["theta"].as<double>() : 0.0;

      // [VELOCITY] VDA5050 top-level `velocity` published by mqtt_robot_client_node:
      // a SINGLE number, the robot's linear speed in m/s, already converted from the
      // encoder RPM on the robot. Nothing is converted here; the value is only read.
      //
      //   velocity present -> twist = { linear: |velocity| }   one value, never negative
      //                       (reversing counts as speed too). It is 0.0 only when the
      //                       robot really reports a standstill.
      //                       robot_state["velocity"] = velocity (signed, as sent)
      //   velocity missing / not a finite number (robot leaves it out when its encoder
      //   is stale or not configured) -> twist stays EMPTY and robot_state["velocity"]
      //   is null. No 0.0 or -1.0 is written here: "no speed information" must never
      //   look like "standing still". execution turns the empty twist into UNKNOWN.
      const double velocity_in = yaml_scalar_as_finite_double(payload["velocity"], std::nan(""));
      const bool have_velocity = std::isfinite(velocity_in);

      YAML::Node twist(YAML::NodeType::Map);
      if (have_velocity) {
        twist["linear"] = std::fabs(velocity_in);
      }

      AnyMap robot_state;
      robot_state["pose"] = pose;
      robot_state["twist"] = twist;
      if (have_velocity) {
        robot_state["velocity"] = YAML::Node(velocity_in);
      } else {
        robot_state["velocity"] = YAML::Node(YAML::NodeType::Null);
      }
      robot_state["battery"] = battery;
      robot_state["charging"] = charging;
      // Wall time this state was received, so execution can treat an old
      // charging value as unknown when the robot stops sending states.
      robot_state["state_received_at"] = YAML::Node(unix_timestamp_now());
      robot_state["status"] = YAML::Node(driving ? "moving" : "idle");
      robot_state["current_task"] = YAML::Node(order_id);
      robot_state["tf_data"] = YAML::Node(YAML::NodeType::Null);
      if (load_status.has_value()) {
        robot_state["load_status"] = YAML::Node(*load_status);
      } else {
        robot_state["load_status"] = YAML::Node(YAML::NodeType::Null);
      }
      if (operating_mode.has_value()) {
        robot_state["operating_mode"] = YAML::Node(*operating_mode);
      } else {
        robot_state["operating_mode"] = YAML::Node(YAML::NodeType::Null);
      }
      if (payload["orderUpdateId"] && !payload["orderUpdateId"].IsNull()) {
        try {
          robot_state["vda_order_update_id"] = YAML::Node(payload["orderUpdateId"].as<int>());
        } catch (...) {
          try {
            robot_state["vda_order_update_id"] =
              YAML::Node(std::stoi(yaml_scalar_as_string(payload["orderUpdateId"])));
          } catch (...) {
          }
        }
      }
      if (payload["newBaseRequest"] && !payload["newBaseRequest"].IsNull()) {
        try {
          robot_state["vda_new_base_request"] = YAML::Node(payload["newBaseRequest"].as<bool>());
        } catch (...) {
          robot_state["vda_new_base_request"] =
            YAML::Node(yaml_scalar_as_string(payload["newBaseRequest"]) == "true" ||
            yaml_scalar_as_string(payload["newBaseRequest"]) == "1");
        }
      }
      impl_->amr_states[amr_id] = std::move(robot_state);
      append_load_status_change_event(amr_id, load_status, order_id, payload);
      return;
    }

    if (parts[0] == "uagv" && parts.size() >= 5 && parts[2] == "Armstrong" && parts.back() == "connection") {
      if (!payload || !payload.IsMap()) {
        return;
      }
      std::string amr_id = yaml_scalar_as_string(payload["serialNumber"]);
      if (amr_id.empty()) {
        amr_id = parts[parts.size() - 2];
      }
      AnyMap connection_state;
      connection_state["connectionState"] =
        YAML::Node(yaml_scalar_as_string(payload["connectionState"]));
      connection_state["headerId"] = payload["headerId"];
      connection_state["timestamp"] = payload["timestamp"];
      connection_state["version"] = payload["version"];
      impl_->vda_connection_by_robot[amr_id] = std::move(connection_state);
    }
  } catch (const std::exception & e) {
    std::cout << "[MQTT] on_message error: " << e.what() << ", topic=" <<
      (msg->topic ? msg->topic : "") << std::endl;
  }
}
namespace
{

void on_mqtt_connect(struct mosquitto * /*mosq*/, void * obj, int rc)
{
  auto * client = static_cast<MQTTFleetClient *>(obj);
  if (client) {
    client->handle_mqtt_connect(rc);
  }
}

void on_mqtt_disconnect(struct mosquitto * /*mosq*/, void * obj, int rc)
{
  auto * client = static_cast<MQTTFleetClient *>(obj);
  if (client) {
    client->handle_mqtt_disconnect(rc);
  }
}

void on_mqtt_message(
  struct mosquitto * /*mosq*/,
  void * obj,
  const struct mosquitto_message * msg)
{
  auto * client = static_cast<MQTTFleetClient *>(obj);
  if (client) {
    client->handle_mqtt_message(msg);
  }
}

}  // namespace
MQTTFleetClient::MQTTFleetClient(
  const std::string & broker_host,
  int broker_port,
  const std::string & client_id)
: impl_(std::make_unique<Impl>())
{
  impl_->broker_host = broker_host;
  impl_->broker_port = broker_port;
  impl_->client_id = client_id;
}

MQTTFleetClient::MQTTFleetClient(MQTTFleetClient &&) noexcept = default;
MQTTFleetClient & MQTTFleetClient::operator=(MQTTFleetClient &&) noexcept = default;

MQTTFleetClient::~MQTTFleetClient()
{
  stop();
}

bool MQTTFleetClient::start()
{
  // Python mqtt_client.py lines 217–236
  if (!impl_) {
    return false;
  }
  stop();

  static std::once_flag mosquitto_init_once;
  std::call_once(mosquitto_init_once, []() { mosquitto_lib_init(); });

  impl_->mosq = mosquitto_new(impl_->client_id.c_str(), true, this);
  if (!impl_->mosq) {
    std::cout << "[MQTT] start failed: mosquitto_new returned null" << std::endl;
    return false;
  }

  mosquitto_connect_callback_set(impl_->mosq, on_mqtt_connect);
  mosquitto_disconnect_callback_set(impl_->mosq, on_mqtt_disconnect);
  mosquitto_message_callback_set(impl_->mosq, on_mqtt_message);

  const int connect_rc = mosquitto_connect(
    impl_->mosq, impl_->broker_host.c_str(), impl_->broker_port, 60);
  if (connect_rc != MOSQ_ERR_SUCCESS) {
    std::cout << "[MQTT] start failed: connect rc=" << connect_rc << std::endl;
    mosquitto_destroy(impl_->mosq);
    impl_->mosq = nullptr;
    return false;
  }

  const int loop_rc = mosquitto_loop_start(impl_->mosq);
  if (loop_rc != MOSQ_ERR_SUCCESS) {
    std::cout << "[MQTT] start failed: loop_start rc=" << loop_rc << std::endl;
    mosquitto_disconnect(impl_->mosq);
    mosquitto_destroy(impl_->mosq);
    impl_->mosq = nullptr;
    return false;
  }

  return true;
}

void MQTTFleetClient::stop()
{
  // Python mqtt_client.py lines 238–247
  if (!impl_ || !impl_->mosq) {
    if (impl_) {
      impl_->connected = false;
    }
    return;
  }

  try {
    mosquitto_loop_stop(impl_->mosq, true);
    mosquitto_disconnect(impl_->mosq);
  } catch (...) {
  }
  mosquitto_destroy(impl_->mosq);
  impl_->mosq = nullptr;
  impl_->connected = false;
}

std::map<std::string, AnyMap> MQTTFleetClient::get_robot_states()
{
  // Python mqtt_client.py lines 249–253
  if (!impl_) {
    return {};
  }
  std::lock_guard<std::mutex> lock(impl_->events_mutex);
  return impl_->amr_states;
}

std::map<std::string, AnyMap> MQTTFleetClient::get_vda_connection_states()
{
  // Python mqtt_client.py lines 255–258
  if (!impl_) {
    return {};
  }
  std::lock_guard<std::mutex> lock(impl_->events_mutex);
  return impl_->vda_connection_by_robot;
}

std::vector<AnyMap> MQTTFleetClient::get_and_clear_load_status_events()
{
  // Python mqtt_client.py lines 260–265
  if (!impl_) {
    return {};
  }
  std::lock_guard<std::mutex> lock(impl_->events_mutex);
  std::vector<AnyMap> events = std::move(impl_->load_status_events);
  impl_->load_status_events.clear();
  return events;
}

std::vector<AnyMap> MQTTFleetClient::get_and_clear_nav_status_events()
{
  // Python mqtt_client.py lines 267–269
  return {};
}

bool MQTTFleetClient::publish_vda5050_order(
  const std::string & serial_number,
  const AnyMap & order_payload,
  const std::string & interface_name,
  const std::string & major_version,
  const std::string & manufacturer)
{
  // Python mqtt_client.py lines 271–288
  if (!impl_ || !impl_->mosq || !impl_->connected) {
    return false;
  }
  try {
    const std::string topic =
      interface_name + "/" + major_version + "/" + manufacturer + "/" +
      serial_number + "/order";
    const std::string payload = anymap_to_json_string(order_payload);
    const int rc = mosquitto_publish(
      impl_->mosq, nullptr, topic.c_str(),
      static_cast<int>(payload.size()), payload.c_str(), 1, false);
    return rc == MOSQ_ERR_SUCCESS;
  } catch (...) {
    return false;
  }
}

bool MQTTFleetClient::publish_vda5050_instant_actions(
  const std::string & serial_number,
  const AnyMap & instant_actions_payload,
  const std::string & interface_name,
  const std::string & major_version,
  const std::string & manufacturer)
{
  // Python mqtt_client.py lines 290–307
  if (!impl_ || !impl_->mosq || !impl_->connected) {
    return false;
  }
  try {
    const std::string topic =
      interface_name + "/" + major_version + "/" + manufacturer + "/" +
      serial_number + "/instantActions";
    const std::string payload = anymap_to_json_string(instant_actions_payload);
    const int rc = mosquitto_publish(
      impl_->mosq, nullptr, topic.c_str(),
      static_cast<int>(payload.size()), payload.c_str(), 1, false);
    return rc == MOSQ_ERR_SUCCESS;
  } catch (...) {
    return false;
  }
}

AnyMap MQTTFleetClient::build_vda5050_instant_actions_payload(
  const std::string & serial_number,
  int header_id,
  const std::vector<AnyMap> & actions,
  const std::string & manufacturer,
  const std::string & version)
{
  // Python mqtt_client.py lines 309–325
  YAML::Node actions_node(YAML::NodeType::Sequence);
  for (const auto & action : actions) {
    YAML::Node item(YAML::NodeType::Map);
    for (const auto & [key, value] : action) {
      item[key] = value;
    }
    actions_node.push_back(item);
  }
  return {
    {"headerId", YAML::Node(header_id)},
    {"timestamp", YAML::Node(utc_iso_timestamp_millis())},
    {"version", YAML::Node(version)},
    {"manufacturer", YAML::Node(manufacturer)},
    {"serialNumber", YAML::Node(serial_number)},
    {"actions", actions_node},
  };
}

bool MQTTFleetClient::is_connected() const
{
  // Python mqtt_client.py lines 327–328
  return impl_ && impl_->connected;
}
}  // namespace execution
