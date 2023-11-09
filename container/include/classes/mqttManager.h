#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <mosquittopp.h>
#include <pwd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <vector>

#include "libraries/json.hpp"

#include <container/mqtt_publishers_msg.h>
#include <container/std_string.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include "mobile_payment_interface/WorldpayAccount.h"

using namespace std;
using json = nlohmann::json;

vector<string> split(string data, char spliter);

class MQTTManager : public mosqpp::mosquittopp {
 public:
  MQTTManager(ros::NodeHandle);
  ~MQTTManager();

 private:
  const char* kMockTransResponseServiceName =
      ("/payment_reader"
       "/mock_transaction_response");
  const char* kTransEnableServiceName = "/payment_reader/transaction_enable";
  const char* kProdMqttHost = "vehicle.tortops.com";
  const char* kDevMqttHost = "dev3.vehicle.tortops.com";

  ros::NodeHandle nh;
  ros::ServiceClient skuClient;
  ros::Subscriber mqttPublicationsSub;
  ros::Publisher lockOpenPub, amountPub, timeoutPub, accountPub, mqtt_msg_pub;
  ros::Publisher pay_reader_test_mode_ros_pub_;
  ros::ServiceClient mock_trans_response_srv_client_;
  ros::ServiceClient trans_enable_srv_client_;

  std_msgs::Bool openMsg;
  std_msgs::Int32 timeoutMsg;
  std_msgs::Float32 amountMsg;
  // mobile_payment_interface::WorldpayAccount accountMsg;

  bool securemqtt;
  json subscribers;
  int mqtt_port, mqtt_keepalive, macqtt;
  string mqtt_host, mqtt_password, mqtt_id, node_name, sku, encrypt_file;
  const char* homedir;

  void getSKU();
  void Reinit();
  void connect();
  void Reconnect();
  void Publishers_cb(const container::mqtt_publishers_msg& publisher_msg);
  void processContainerControl(json msg);

  void on_connect(int rc);
  void on_disconnect(int rc);
  void on_publish(int mid);
  void on_message(const struct mosquitto_message* message);
  void on_subscribe(int mid, int qos_count, const int* granted_qos);
};

MQTTManager::MQTTManager(ros::NodeHandle nh_priv) : mosquittopp() {
  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  mosqpp::lib_init();
  node_name = ros::this_node::getName();
  int log_level;
  nh_priv.param("log_level", log_level, 0);
  ros::console::levels::Level console_level;
  console_level = (ros::console::levels::Level)log_level;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  mqttPublicationsSub =
      nh.subscribe("/mqtt_publishers", 10, &MQTTManager::Publishers_cb, this);

  lockOpenPub = nh.advertise<std_msgs::Bool>("/lock_open_topic", 1);
  amountPub =
      nh.advertise<std_msgs::Float32>("/payment_reader/transaction_amount", 1);
  timeoutPub =
      nh.advertise<std_msgs::Int32>("/payment_reader/transaction_timeout", 1);
  accountPub = nh.advertise<mobile_payment_interface::WorldpayAccount>(
                                      "/payment_reader/transaction_account", 1);
  mqtt_msg_pub = nh.advertise<std_msgs::String>("mqtt_on_msg", 2);
  pay_reader_test_mode_ros_pub_ = nh.advertise<std_msgs::Bool>(
                                    "/payment_reader/test_transaction_mode", 1);

  skuClient = nh.serviceClient<container::std_string>("/sku");
  mock_trans_response_srv_client_ =
      nh.serviceClient<std_srvs::Empty>(kMockTransResponseServiceName);
  trans_enable_srv_client_ =
      nh.serviceClient<std_srvs::SetBool>(kTransEnableServiceName);

  nh_priv.param("macqtt", macqtt, 0);
  nh_priv.param<string>("mqtt_host", mqtt_host, kProdMqttHost);
  nh_priv.param("mqtt_port", mqtt_port, 8883);
  nh_priv.param<string>("mqtt_password", mqtt_password, "6HPrmTxzM6Sh6hvnEERd");
  nh_priv.param("mqtt_secure", securemqtt, true);
  nh_priv.param("mqtt_keepalive", mqtt_keepalive, 30);

  encrypt_file =
      ros::package::getPath("container") + string{"/config/letsencrypt.pem"};

  getSKU();
  mqtt_id = sku;
  if (macqtt) {
    mqtt_password = sku;
  }

  if (!mqtt_id.empty()) {
    connect();
  }
}

void MQTTManager::getSKU() {
  ros::service::waitForService("/sku");
  container::std_string sku_srv;
  if (!skuClient.call(sku_srv)) {
    ROS_ERROR_STREAM(node_name << " --- error in sku service");
  } else {
    sku = sku_srv.response.data;
  }
}

void MQTTManager::Reinit() {
  int nRet;
  reinitialise(mqtt_id.c_str(), true);

  if (securemqtt) {
    nRet = tls_set(encrypt_file.c_str());
    nRet = tls_insecure_set(false);
    nRet = tls_opts_set(0, "tlsv1.2", "ECDHE-RSA-AES256-GCM-SHA384");
    nRet = username_pw_set(mqtt_id.c_str(), mqtt_password.c_str());
  }
}

void MQTTManager::connect() {
  Reinit();

  int nRet = connect_async(mqtt_host.c_str(), mqtt_port, mqtt_keepalive);
  nRet = loop_start();
}

void MQTTManager::Reconnect() {
  Reinit();

  ROS_DEBUG("Connecting to %s, port: %d and keepalive: %d...",
            mqtt_host.c_str(), mqtt_port, mqtt_keepalive);
  int nRet = connect_async(mqtt_host.c_str(), mqtt_port, mqtt_keepalive);
  if (nRet != MOSQ_ERR_SUCCESS) {
    ROS_ERROR("MQTT error: connect_async (%d)", nRet);
  }
}

void MQTTManager::Publishers_cb(
    const container::mqtt_publishers_msg& mqtt_publisher) {
  string topic = "containers/" + sku + "/" + mqtt_publisher.mqtt_topic;
  int x = publish(NULL, topic.c_str(), strlen(mqtt_publisher.raw_msg.c_str()),
                  mqtt_publisher.raw_msg.c_str(), mqtt_publisher.qos, false);
}

void MQTTManager::on_disconnect(int rc) {
  ROS_ERROR_STREAM(node_name << " ---  disconnection(" << rc
                             << "), error: " << string{strerror(rc)});
  Reconnect();
}

void MQTTManager::on_connect(int rc) {
  if (rc == 0) {
    ROS_DEBUG_STREAM(node_name << " --- connected with server");

  } else {
    ROS_ERROR_STREAM(node_name << " --- Impossible to connect with server("
                               << rc << ")");
  }
}

void MQTTManager::on_publish(int mid) {
  // ROS_DEBUG_STREAM (node_name << " --- Message (" << mid << ") succeed to be
  // published " );
}

void MQTTManager::on_subscribe(int mid, int qos_count, const int* granted_qos) {
  ROS_DEBUG_STREAM(node_name << " --- subscription succeeded (" << mid << ") ");
}

vector<string> split(string data, char spliter) {
  vector<string> tokens;
  string temp;
  stringstream check1(data);
  while (getline(check1, temp, spliter)) {
    tokens.push_back(temp);
  }
  return tokens;
}

void MQTTManager::on_message(const struct mosquitto_message* message) {
  string topicFull = message->topic;

  ROS_DEBUG_STREAM(
      "==================== MQTT message received ====================");
  ROS_DEBUG_STREAM("Subscriber: " << mqtt_id);
  ROS_DEBUG_STREAM("Destiny topic: " << topicFull);
  ROS_DEBUG_STREAM("Data: " << reinterpret_cast<char*>(message->payload));

  vector<string> topic = split(topicFull, '/');
  string raw_msg = reinterpret_cast<char*>(message->payload);
  // Debug topic
  std_msgs::String str;
  str.data = "Topic: /" + topic.at(0) + ", msg: " + raw_msg;
  mqtt_msg_pub.publish(str);

  try {
    json msg = json::parse(raw_msg);
    if (topic.at(0) == "containercontrol") {
      processContainerControl(msg);
    }
  } catch (const exception& ex) {
    ROS_ERROR_STREAM(node_name << " --- Error parsing mqtt json: " << raw_msg);
    ROS_ERROR_STREAM(ex.what());
  }
}

void MQTTManager::processContainerControl(json msg) {
  if (!msg.contains("command")) {
    ROS_WARN_STREAM(node_name << "--- Error in MQTT containercontrol message");
    return;
  }

  string cmd = msg["command"];
  int nRet = 0;
  if (cmd.compare("config") == 0) {
    std_srvs::SetBool bool_msg;

    amountMsg.data = msg["amount"];
    amountPub.publish(amountMsg);

    bool_msg.request.data = msg["enable"];

    if (!trans_enable_srv_client_.call(bool_msg)) {
      ROS_ERROR(
          "Couldn't enable/disable payment transactions: service"
          " might be down");
    }
  } else if (cmd.compare("env") == 0) {
    /* Selecting broker host */
    if (mqtt_host.compare(kProdMqttHost) == 0) {
      mqtt_host = kDevMqttHost;
    } else if (mqtt_host.compare(kDevMqttHost) == 0) {
      mqtt_host = kProdMqttHost;
    }

    /* disconecting from the current broker (then reconnect the new one in the
     * disconnect callback) */
    disconnect();
  } else if (cmd.compare("use_test_transactions") == 0) {
    bool enable_test_trans = msg["enable"];
    mobile_payment_interface::WorldpayAccount account_msg;
    std_msgs::Bool test_mode_msg;

    if(enable_test_trans)
    {
        account_msg.use_test_account =  true;
        account_msg.use_tortoise_account = false;
        test_mode_msg.data = true;
    }
    else
    {
        account_msg.use_test_account =  false;
        account_msg.use_tortoise_account = true;
        test_mode_msg.data = false;
    }
    accountPub.publish(account_msg);
    pay_reader_test_mode_ros_pub_.publish(test_mode_msg);
  } else if (cmd.compare("send_mock_transaction") == 0) {
    std_srvs::Empty empty_msg;

    if (!mock_trans_response_srv_client_.call(empty_msg)) {
      ROS_ERROR(
          "Couldn't trigger a mock payment transaction response:"
          " service might be down");
    }
  } else if (cmd.compare("price") == 0) {
    amountMsg.data = msg["amount"];
    amountPub.publish(amountMsg);
  } else if (cmd.compare("enable") == 0) {
    std_srvs::SetBool bool_msg;

    bool_msg.request.data = msg["enable"];

    if (!trans_enable_srv_client_.call(bool_msg)) {
      ROS_ERROR(
          "Couldn't enable/disable payment transactions: service"
          " might be down");
    }
  } else if (cmd.compare("open") == 0) {
    openMsg.data = true;
    lockOpenPub.publish(openMsg);
  } else if (cmd.compare("account") == 0) {
    /* accountMsg.id = msg["id"];
    accountMsg.token = msg["token"];
    accountMsg.acceptor_id = msg["acceptor"];
    accountMsg.use_test_account = false; */
  } else if (cmd.compare("toggle_provider") == 0) {
    string switch_network_command = "sudo /usr/bin/switch-interface.sh";

    Reconnect(); /* Neccessary to catch the disconnection quickly */
    ROS_DEBUG("Switching network ...");
    nRet = system(switch_network_command.c_str());
    if (nRet == 0) {
      ROS_INFO("Switch network script executed successfully");
      Reconnect(); /* Neccessary to reconnect with the new interface */
    }
  } else if (cmd.compare("reboot") == 0) {
    ROS_INFO_STREAM(node_name << "--- Stopping mss & rebooting");
    nRet = system(
        "sudo shutdown -r +1 'The vehicle is about to restart (UI request)'");
    if (nRet == 0)
      nRet = system("systemctl --user stop mss_ros_launcher.service");
  } else if (cmd.compare("shutdown") == 0) {
    ROS_INFO_STREAM(node_name << "--- Stopping mss & poweroff");
    nRet = system(
        "sudo shutdown -P +1 'The vehicle is about to shutdown (UI request)'");
    if (nRet == 0)
      nRet = system("systemctl --user stop mss_ros_launcher.service");
  } else {
    ROS_WARN_STREAM(node_name << "--- containercontrol unkown command");
  }
}

MQTTManager::~MQTTManager() {
  disconnect();
  loop_stop();
  mosqpp::lib_cleanup();
}
