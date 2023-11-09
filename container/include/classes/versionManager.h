/*
The container software version is obtained from the package.xml (the manifest of
the project) that is processed by catkin and exposed by CMAKE
${${PROJECT_NAME}_VERSION} variable, it is passed to the node via the compile
definitions function in the CMAKELISTS.txt
*/
#ifdef CONTAINER_VERSION
# define SW_VERSION CONTAINER_VERSION
#else
# define SW_VERSION "x.x.x"
#endif

#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <container/std_string.h>

#include <pwd.h>
#include <fstream>

#include "libraries/json.hpp"

using namespace std;
using json = nlohmann::json;

class VersionManager {
 public:
  VersionManager(ros::NodeHandle nh_priv);
  ~VersionManager();

 private:
  ros::NodeHandle nh;
  ros::ServiceServer sku_serv, version_serv, provider_serv;

  int log_level;
  const char* homedir;
  string node_name, sku;

  void GetSku();
  string GetProvider();

  bool SkuService(container::std_string::Request& request,
                  container::std_string::Response& response);
  bool VersionService(container::std_string::Request& request,
                      container::std_string::Response& response);
  bool ProviderService(container::std_string::Request& request,
                      container::std_string::Response& response);
};

VersionManager::VersionManager(ros::NodeHandle nh_priv) {
  node_name = ros::this_node::getName();
  nh_priv.param("log_level", log_level, 0);
  ros::console::levels::Level console_level = (ros::console::levels::Level)log_level;
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  if ((homedir = getenv("HOME")) == NULL) {
    homedir = getpwuid(getuid())->pw_dir;
  }

  GetSku();

  sku_serv = nh.advertiseService("/sku", &VersionManager::SkuService, this);
  version_serv =
      nh.advertiseService("/version", &VersionManager::VersionService, this);
  provider_serv = nh.advertiseService("/provider", &VersionManager::ProviderService, this);
}

void VersionManager::GetSku() {
  string sku_path(homedir);
  sku_path.append("/temp/id.txt");
  ifstream sku_file(sku_path);

  if (sku_file.good()) {
    try {
      json j = json::parse(sku_file);
      if (j.contains("sku")) {
        sku = j.at("sku");
      }
    } catch (...) {
      ROS_ERROR_STREAM(node_name << " -- Error reading sku.");
    }
  } else {
    ROS_WARN_STREAM(node_name << " -- Error opening " << sku_path);
  }
}

string VersionManager::GetProvider() {
  string active_interface{""};
  string provider{"Error"};
  string active_iface_path(homedir);
  active_iface_path.append("/temp/active-interface.txt");
  ifstream active_iface_file(active_iface_path.c_str());

  if (!active_iface_file.good()) {
    ROS_WARN_STREAM(node_name << " -- Error opening " << active_iface_path);
    return provider;
  }

  getline(active_iface_file, active_interface);
  if (active_interface.empty()) {
    ROS_WARN_STREAM(node_name << " -- Error, no active interface registered");
    return provider;
  }

  string provider_path(homedir);
  provider_path.append("/temp/carrier-")
      .append(active_interface)
      .append(".txt");

  ifstream provider_file(provider_path.c_str());
  if (!provider_file.good()) {
    ROS_WARN_STREAM(node_name << " -- Error opening " << provider_path);
    return provider;
  }

  getline(provider_file, provider);
  if (provider.empty()) {
    ROS_WARN_STREAM(node_name << " -- Error, current provider not registered");
    provider = "Error";
  }

  return provider;
}

bool VersionManager::SkuService(container::std_string::Request& request,
                                container::std_string::Response& response) {
  response.data = sku;
  return true;
}

bool VersionManager::VersionService(container::std_string::Request& request,
                                    container::std_string::Response& response) {
  response.data = SW_VERSION;
  return true;
}

bool VersionManager::ProviderService(container::std_string::Request& request,
                                    container::std_string::Response& response) {
  response.data = GetProvider();
  return true;
}

VersionManager::~VersionManager() {}