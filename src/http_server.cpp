#include "http/http_server.h"

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace indooruav_http {

namespace {
const char* kJsonContentType = "application/json";

std::string ReadYamlValue(const std::string& file_path, const std::string& key) {
    try {
        YAML::Node root = YAML::LoadFile(file_path);
        if (root[key]) {
            return root[key].as<std::string>();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to read YAML key '%s' from %s: %s",
                  key.c_str(), file_path.c_str(), e.what());
    }
    return "";
}

bool UpdateYamlValue(const std::string& file_path, const std::string& key,
                     const std::string& new_value) {
    std::ifstream in(file_path);
    if (!in.is_open()) {
        ROS_ERROR("Failed to open %s for update", file_path.c_str());
        return false;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        lines.push_back(line);
    }
    in.close();

    const std::string needle = key + ":";
    bool found = false;
    for (auto& l : lines) {
        if (l.find(needle) != std::string::npos) {
            size_t colon = l.find(':');
            if (colon != std::string::npos) {
                size_t val_start = l.find_first_not_of(" \t", colon + 1);
                if (val_start == std::string::npos) val_start = colon + 1;
                l = l.substr(0, val_start) + new_value;
                found = true;
                break;
            }
        }
    }

    if (!found) {
        ROS_ERROR("Key '%s' not found in %s", key.c_str(), file_path.c_str());
        return false;
    }

    std::ofstream out(file_path, std::ios::trunc);
    if (!out.is_open()) {
        ROS_ERROR("Failed to write %s", file_path.c_str());
        return false;
    }
    for (size_t i = 0; i < lines.size(); ++i) {
        out << lines[i];
        if (i + 1 < lines.size()) out << "\n";
    }
    out.close();
    return true;
}

}

// 构造HTTP服务端并初始化发布者/服务客户端/路由
HttpServer::HttpServer(ros::NodeHandle& nh, int port)
    : port_(port) {
    server_ = std::make_unique<httplib::Server>();
    SetupPublishers(nh);
    SetupCommandClients(nh);
    SetupRoutes();
}

// 析构时停止HTTP服务
HttpServer::~HttpServer() {
    Stop();
}

// 启动HTTP监听线程
bool HttpServer::Start() {
    if (is_running_) {
        ROS_WARN("HTTP server already running");
        return false;
    }

    if (!server_->bind_to_port("0.0.0.0", port_)) {
        ROS_ERROR("Failed to bind HTTP server to 0.0.0.0:%d", port_);
        return false;
    }

    is_running_ = true;
    server_thread_ = std::make_unique<std::thread>([this]() {
        try {
            ROS_INFO("HTTP server listening on 0.0.0.0:%d", port_);
            const bool listen_ok = server_->listen_after_bind();
            if (is_running_ && !listen_ok) {
                ROS_ERROR("HTTP server stopped listening unexpectedly on 0.0.0.0:%d", port_);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("HTTP server exception: %s", e.what());
        }
        is_running_ = false;
    });

    return true;
}

// 停止HTTP监听并回收线程
void HttpServer::Stop() {
    if (server_ && is_running_) {
        server_->stop();
    }
    if (server_thread_ && server_thread_->joinable()) {
        server_thread_->join();
    }
    is_running_ = false;
}

// 初始化航线信息发布话题
void HttpServer::SetupPublishers(ros::NodeHandle& nh) {
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("airline_info_topic", airline_info_topic_, "/indooruav_http/airline_info");
    nh_private.param<std::string>("airline_key_topic", airline_key_topic_, "/indooruav_http/airline_key");
    nh_private.param<std::string>("airline_key_param", airline_key_param_, "/indooruav_http/current_airline_key");

    airline_info_pub_ = nh.advertise<std_msgs::String>(airline_info_topic_, 10);
    airline_key_pub_ = nh.advertise<std_msgs::String>(airline_key_topic_, 10, true);
}

// 初始化控制命令对应的ROS服务客户端
void HttpServer::SetupCommandClients(ros::NodeHandle& nh) {
    ros::NodeHandle nh_private("~");
    nh_private.param<double>("command_service_wait_timeout_sec", command_service_wait_timeout_sec_, 1.0);

    command_service_names_.clear();
    for (int mode = 1; mode <= 6; ++mode) {
        std::string param_name = "command_mode_" + std::to_string(mode) + "_service";
        std::string default_service;
        if (mode == 1) {
            default_service = "indooruav_core/state_machine_event/takeoff_command";
        }

        std::string service_name;
        nh_private.param<std::string>(param_name, service_name, default_service);
        if (!service_name.empty()) {
            command_service_names_[mode] = service_name;
            command_clients_[mode] = nh.serviceClient<std_srvs::Empty>(service_name);
        }
    }

    if (command_service_names_.empty()) {
        ROS_WARN("No command services configured. /sendCommand will return resultCode=2.");
    }
}

// 注册HTTP路由与回调
void HttpServer::SetupRoutes() {
    server_->Get("/airlineInfo", [this](const httplib::Request& req, httplib::Response& res) {
        HandleAirlineInfo(req, res);
    });
    server_->Post("/airlineInfo", [this](const httplib::Request& req, httplib::Response& res) {
        HandleAirlineInfo(req, res);
    });
    server_->Get("/sendCommand", [this](const httplib::Request& req, httplib::Response& res) {
        HandleCommand(req, res);
    });
    server_->Post("/sendCommand", [this](const httplib::Request& req, httplib::Response& res) {
        HandleCommand(req, res);
    });
}

// 处理/airlineInfo航线选择请求并发布到ROS
void HttpServer::HandleAirlineInfo(const httplib::Request& req, httplib::Response& res) {
    int site_id = 0;
    int device_id = 0;
    std::string airline_key;
    std::string detect_time_cur;

    bool ok = GetIntParam(req, "siteId", &site_id) &&
              GetIntParam(req, "deviceId", &device_id) &&
              GetStringParam(req, "airlineKey", &airline_key) &&
              GetStringParam(req, "detectTimeCur", &detect_time_cur);
    std::cout << "Site ID: " << site_id << ", Device ID: " << device_id << ", Airline Key: " << airline_key << ", Detect Time: " << detect_time_cur << std::endl;
    if (!ok && !req.body.empty()) {
        try {
            nlohmann::json body = nlohmann::json::parse(req.body);
            if (site_id <= 0) {
                site_id = body.value("siteId", 0);
            }
            if (device_id <= 0) {
                device_id = body.value("deviceId", 0);
            }
            if (airline_key.empty()) {
                airline_key = body.value("airlineKey", "");
            }
            if (detect_time_cur.empty()) {
                detect_time_cur = body.value("detectTimeCur", "");
            }
            ok = (site_id > 0 && device_id > 0 &&
                  !airline_key.empty() && !detect_time_cur.empty());
        } catch (const std::exception&) {
            ok = false;
        }
    }

    if (!ok) {
        SendResult(res, 3);
        return;
    }

    // 根据 airlineKey 查找对应的航线文件并更新配置
    const std::string waypoint_pkg = ros::package::getPath("indooruav_waypoint");
    const std::string waypoint_file = waypoint_pkg + "/waypoints/" + airline_key + ".yaml";
    {
        std::ifstream check(waypoint_file);
        if (!check.is_open()) {
            ROS_ERROR("Waypoint file not found: %s", waypoint_file.c_str());
            SendResult(res, 2);
            return;
        }
    }

    const std::string map3d_name = ReadYamlValue(waypoint_file, "map3d_name");
    if (map3d_name.empty()) {
        ROS_ERROR("map3d_name not found in %s", waypoint_file.c_str());
        SendResult(res, 2);
        return;
    }

    const std::string config_yaml = waypoint_pkg + "/config/config.yaml";
    const std::string localize_yaml = ros::package::getPath("fastlio") + "/config/localize.yaml";

    if (!UpdateYamlValue(config_yaml, "waypoints_file_path", "waypoints/" + airline_key + ".yaml")) {
        ROS_ERROR("Failed to update waypoints_file_path in %s", config_yaml.c_str());
        SendResult(res, 2);
        return;
    }
    ROS_INFO("Updated waypoints_file_path -> waypoints/%s.yaml", airline_key.c_str());

    if (!UpdateYamlValue(localize_yaml, "pcd_name", map3d_name + ".pcd")) {
        ROS_ERROR("Failed to update pcd_name in %s", localize_yaml.c_str());
        SendResult(res, 2);
        return;
    }
    ROS_INFO("Updated pcd_name -> %s.pcd", map3d_name.c_str());

    nlohmann::json payload = {
        {"siteId", site_id},
        {"deviceId", device_id},
        {"airlineKey", airline_key},
        {"detectTimeCur", detect_time_cur}
    };

    std_msgs::String info_msg;
    info_msg.data = payload.dump();
    airline_info_pub_.publish(info_msg);

    std_msgs::String key_msg;
    key_msg.data = airline_key;
    airline_key_pub_.publish(key_msg);
    ros::param::set(airline_key_param_, airline_key);

    SendResult(res, 1);
}

// 处理/sendCommand控制命令请求并转发到ROS服务
void HttpServer::HandleCommand(const httplib::Request& req, httplib::Response& res) {
    int site_id = 0;
    int device_id = 0;
    int command_mode = 0;

    bool ok = GetIntParam(req, "siteId", &site_id) &&
              GetIntParam(req, "deviceId", &device_id) &&
              GetIntParam(req, "commandMode", &command_mode);

    if (!ok && !req.body.empty()) {
        try {
            nlohmann::json body = nlohmann::json::parse(req.body);
            if (!GetIntParam(req, "siteId", &site_id)) {
                site_id = body.value("siteId", 0);
            }
            if (!GetIntParam(req, "deviceId", &device_id)) {
                device_id = body.value("deviceId", 0);
            }
            if (!GetIntParam(req, "commandMode", &command_mode)) {
                command_mode = body.value("commandMode", 0);
            }
            ok = (site_id > 0 && device_id > 0 && command_mode > 0);
        } catch (const std::exception&) {
            ok = false;
        }
    }

    if (!ok) {
        ROS_WARN("Rejected /sendCommand request from %s:%d due to invalid parameters",
                 req.remote_addr.c_str(),
                 req.remote_port);
        SendResult(res, 3);
        return;
    }

    if (command_mode < 1 || command_mode > 6) {
        ROS_WARN("Rejected /sendCommand request from %s:%d due to unsupported commandMode=%d",
                 req.remote_addr.c_str(),
                 req.remote_port,
                 command_mode);
        SendResult(res, 3);
        return;
    }

    std::string service_name = "<unmapped>";
    const auto command_name_it = command_service_names_.find(command_mode);
    if (command_name_it != command_service_names_.end()) {
        service_name = command_name_it->second;
    }

    if (command_mode == 1) {
        ROS_INFO("Received /sendCommand from %s:%d on %s, siteId=%d, deviceId=%d, commandMode=%d, forwarding to service [%s]",
                 req.remote_addr.c_str(),
                 req.remote_port,
                 req.local_addr.c_str(),
                 site_id,
                 device_id,
                 command_mode,
                 service_name.c_str());
    }

    const bool success = CallCommandService(command_mode);

    if (command_mode == 1) {
        if (success) {
            ROS_INFO("Frontend takeoff command forwarded successfully to state machine service [%s]",
                     service_name.c_str());
        } else {
            ROS_WARN("Failed to forward frontend takeoff command to state machine service [%s]",
                     service_name.c_str());
        }
    }

    SendResult(res, success ? 1 : 2);
}

// 根据commandMode调用对应的ROS服务
bool HttpServer::CallCommandService(int command_mode) {
    auto name_it = command_service_names_.find(command_mode);
    if (name_it == command_service_names_.end()) {
        ROS_WARN("Command mode %d has no service mapping", command_mode);
        return false;
    }

    auto client_it = command_clients_.find(command_mode);
    if (client_it == command_clients_.end()) {
        return false;
    }

    ros::ServiceClient& client = client_it->second;
    if (command_service_wait_timeout_sec_ > 0.0) {
        if (!client.waitForExistence(ros::Duration(command_service_wait_timeout_sec_))) {
            ROS_WARN("Command service not available: %s", name_it->second.c_str());
            return false;
        }
    } else if (!client.exists()) {
        return false;
    }

    std_srvs::Empty srv;
    return client.call(srv);
}

// 从HTTP参数解析整数
bool HttpServer::GetIntParam(const httplib::Request& req, const std::string& key, int* value) const {
    if (!req.has_param(key.c_str())) {
        return false;
    }
    try {
        *value = std::stoi(req.get_param_value(key.c_str()));
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// 从HTTP参数解析字符串
bool HttpServer::GetStringParam(const httplib::Request& req, const std::string& key, std::string* value) const {
    if (!req.has_param(key.c_str())) {
        return false;
    }
    *value = req.get_param_value(key.c_str());
    return !value->empty();
}

// 统一返回resultCode响应
void HttpServer::SendResult(httplib::Response& res, int result_code) const {
    nlohmann::json body = {{"resultCode", result_code}};
    res.set_content(body.dump(), kJsonContentType);
    res.status = 200;
}

}  // namespace indooruav_http
