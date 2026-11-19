#include "http/http_server.h"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
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

// 按候选参数名依次查找请求参数（兼容文档 URL 中 siteId/deviceId/pointId 的
// siteld/deviceld/pointid 写法），未找到返回空串
std::string FindParam(const httplib::Request& req,
                      std::initializer_list<const char*> keys) {
    for (const char* key : keys) {
        if (req.has_param(key)) {
            return req.get_param_value(key);
        }
    }
    return "";
}

bool ParseIntValue(const std::string& raw, int* value) {
    if (raw.empty() || value == nullptr) {
        return false;
    }
    try {
        size_t consumed = 0;
        const int parsed = std::stoi(raw, &consumed);
        if (consumed != raw.size()) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool ParseDoubleValue(const std::string& raw, double* value) {
    if (raw.empty() || value == nullptr) {
        return false;
    }
    try {
        size_t consumed = 0;
        const double parsed = std::stod(raw, &consumed);
        if (consumed != raw.size() || !std::isfinite(parsed)) {
            return false;
        }
        *value = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// 将一行中 field_key（形如 "gimbal_pitch:"）对应的数值替换为 new_value
bool ReplaceFieldValue(std::string& line, const std::string& field_key,
                       const std::string& new_value) {
    const size_t first_non_space = line.find_first_not_of(" \t");
    if (first_non_space == std::string::npos || line[first_non_space] == '#') {
        return false;
    }
    const size_t field_pos = line.find(field_key);
    if (field_pos == std::string::npos) {
        return false;
    }
    const size_t val_begin = line.find_first_not_of(" \t", field_pos + field_key.size());
    if (val_begin == std::string::npos) {
        return false;
    }
    size_t val_end = line.find_first_of(",}#", val_begin);
    if (val_end == std::string::npos) {
        val_end = line.size();
    }
    // 保留数值与注释之间的空白
    while (val_end > val_begin && (line[val_end - 1] == ' ' || line[val_end - 1] == '\t')) {
        --val_end;
    }
    line = line.substr(0, val_begin) + new_value + line.substr(val_end);
    return true;
}

// 找到包含第 line_index 行的块式（或以 "- " 起始的多行）序列项的行范围 [start, end)
bool FindSequenceEntryRange(const std::vector<std::string>& lines, size_t line_index,
                            size_t* start, size_t* end) {
    size_t dash = 0;
    size_t dash_indent = 0;
    bool found = false;
    for (size_t i = line_index + 1; i-- > 0; ) {
        const std::string& l = lines[i];
        const size_t first = l.find_first_not_of(" \t");
        if (first == std::string::npos || l[first] == '#') {
            continue;
        }
        if (l[first] == '-' && first + 1 < l.size() &&
            (l[first + 1] == ' ' || l[first + 1] == '\t')) {
            dash = i;
            dash_indent = first;
            found = true;
            break;
        }
        if (first == 0) {
            break;  // 已到顶层键仍未找到序列项
        }
    }
    if (!found) {
        return false;
    }

    size_t stop = lines.size();
    for (size_t i = dash + 1; i < lines.size(); ++i) {
        const std::string& l = lines[i];
        const size_t first = l.find_first_not_of(" \t");
        if (first == std::string::npos || l[first] == '#') {
            continue;
        }
        if (first <= dash_indent) {
            stop = i;
            break;
        }
    }
    *start = dash;
    *end = stop;
    return true;
}

// 将航点文件中的第 point_id 个停留航点的云台俯仰角 gimbal_pitch 修改为 angle_deg。
// point_id 与 http_client 上报给平台的 waypointList 编号保持一致：
// 只统计 stop: true 的航点，按文件中的出现顺序从 1 开始编号。
// 优先逐行改写，尽量保留录制文件的注释与排版；航点没有 gimbal_pitch 时补写该字段；
// 行内无法定位时退回整文件重写。
bool UpdateWaypointGimbalPitch(const std::string& file_path, int point_id,
                               double angle_deg, std::string* error) {
    if (error != nullptr) {
        error->clear();
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(file_path);
    } catch (const YAML::Exception& e) {
        if (error != nullptr) {
            *error = std::string("YAML parse error: ") + e.what();
        }
        return false;
    }

    const YAML::Node waypoints = root["waypoints"];
    if (!waypoints || !waypoints.IsSequence()) {
        if (error != nullptr) {
            *error = "missing or non-sequence 'waypoints' field";
        }
        return false;
    }

    int target_index = -1;
    int stop_index = 0;
    try {
        for (std::size_t i = 0; i < waypoints.size(); ++i) {
            const bool stop = waypoints[i]["stop"] ? waypoints[i]["stop"].as<bool>() : false;
            if (!stop) {
                continue;
            }
            if (++stop_index == point_id) {
                target_index = static_cast<int>(i);
                break;
            }
        }
    } catch (const YAML::Exception& e) {
        if (error != nullptr) {
            *error = std::string("malformed waypoint entry: ") + e.what();
        }
        return false;
    }

    if (target_index < 0) {
        if (error != nullptr) {
            *error = "pointId " + std::to_string(point_id) + " not found";
        }
        return false;
    }

    bool target_flow = false;
    bool field_exists = false;
    try {
        const YAML::Node target_node = waypoints[static_cast<std::size_t>(target_index)];
        target_flow = (target_node.Style() == YAML::EmitterStyle::Flow);
        field_exists = static_cast<bool>(target_node["gimbal_pitch"]);
    } catch (const YAML::Exception& e) {
        if (error != nullptr) {
            *error = std::string("malformed target waypoint: ") + e.what();
        }
        return false;
    }

    std::ifstream in(file_path);
    if (!in.is_open()) {
        if (error != nullptr) {
            *error = "cannot open file for update";
        }
        return false;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        lines.push_back(line);
    }
    in.close();

    // getline 会丢掉文件末尾的换行，记录后原样写回
    bool has_trailing_newline = false;
    {
        std::ifstream tail(file_path, std::ios::binary | std::ios::ate);
        if (tail.is_open() && tail.tellg() > 0) {
            tail.seekg(-1, std::ios::end);
            char last_char = 0;
            if (tail.get(last_char)) {
                has_trailing_newline = (last_char == '\n');
            }
        }
    }

    const std::string yaw_key = "yaw_deg:";
    const std::string field_key = "gimbal_pitch:";
    std::ostringstream value_stream;
    value_stream << std::fixed << std::setprecision(2) << angle_deg;
    const std::string new_value = value_stream.str();

    int occurrence = 0;
    bool updated = false;
    for (size_t i = 0; i < lines.size(); ++i) {
        std::string& l = lines[i];
        const size_t first_non_space = l.find_first_not_of(" \t");
        if (first_non_space == std::string::npos || l[first_non_space] == '#') {
            continue;  // 跳过空行与注释行
        }
        const size_t key_pos = l.find(yaw_key);
        if (key_pos == std::string::npos) {
            continue;
        }
        if (occurrence++ != target_index) {
            continue;
        }

        if (field_exists) {
            // 字段可能与 yaw_deg 同行，也可能写在同一航点的后续行
            if (ReplaceFieldValue(l, field_key, new_value)) {
                updated = true;
                break;
            }
            size_t entry_start = 0;
            size_t entry_end = 0;
            if (FindSequenceEntryRange(lines, i, &entry_start, &entry_end)) {
                for (size_t j = entry_start; j < entry_end; ++j) {
                    if (j != i && ReplaceFieldValue(lines[j], field_key, new_value)) {
                        updated = true;
                        break;
                    }
                }
            }
            break;  // 找不到则退回整文件重写
        }

        // 字段不存在：补写 gimbal_pitch，保持流式/块式原有排版
        if (target_flow) {
            const size_t brace = l.rfind('}');
            if (brace == std::string::npos) {
                break;
            }
            size_t insert_pos = brace;
            while (insert_pos > 0 && (l[insert_pos - 1] == ' ' || l[insert_pos - 1] == '\t')) {
                --insert_pos;
            }
            const bool has_comma = insert_pos > 0 && l[insert_pos - 1] == ',';
            l.insert(insert_pos, (has_comma ? " " : ", ") + field_key + " " + new_value);
            updated = true;
            break;
        }

        // 块式：在 yaw_deg 行后插入同列的新行
        const std::string indent(key_pos, ' ');
        lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(i) + 1,
                     indent + field_key + " " + new_value);
        updated = true;
        break;
    }

    if (!updated) {
        // 行内定位失败（例如字段跨行书写）：整文件重写
        try {
            root["waypoints"][static_cast<std::size_t>(target_index)]["gimbal_pitch"] = angle_deg;
        } catch (const YAML::Exception& e) {
            if (error != nullptr) {
                *error = std::string("failed to set gimbal_pitch: ") + e.what();
            }
            return false;
        }
        std::ofstream fallback_out(file_path, std::ios::trunc);
        if (!fallback_out.is_open()) {
            if (error != nullptr) {
                *error = "cannot write file";
            }
            return false;
        }
        fallback_out << YAML::Dump(root) << "\n";
        const bool fallback_ok = fallback_out.good();
        fallback_out.close();
        if (!fallback_ok && error != nullptr) {
            *error = "write error while rewriting file";
        }
        return fallback_ok;
    }

    std::ofstream out(file_path, std::ios::trunc);
    if (!out.is_open()) {
        if (error != nullptr) {
            *error = "cannot write file";
        }
        return false;
    }
    for (size_t i = 0; i < lines.size(); ++i) {
        out << lines[i];
        if (i + 1 < lines.size()) out << "\n";
    }
    if (has_trailing_newline && !lines.empty()) {
        out << "\n";
    }
    out.close();
    return out.good();
}

}

// 构造HTTP服务端并初始化发布者/服务客户端/路由
HttpServer::HttpServer(ros::NodeHandle& nh, int port)
    : port_(port), nh_(nh) {
    server_ = std::make_unique<httplib::Server>();
    SetupPublishers(nh);
    SetupCommandClients(nh);
    SetupRoutes();

    // 订阅状态机状态，防止重复起飞
    state_sub_ = nh_.subscribe("/indooruav_core/state_machine/state", 1,
                                &HttpServer::StateCallback, this);
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
    server_->Get("/pointAngleSet", [this](const httplib::Request& req, httplib::Response& res) {
        HandlePointAngleSet(req, res);
    });
    server_->Post("/pointAngleSet", [this](const httplib::Request& req, httplib::Response& res) {
        HandlePointAngleSet(req, res);
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
    const std::string localize_yaml = ros::package::getPath("indooruav_localize") + "/config/localize.yaml";

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
        ROS_INFO("Received /sendCommand from %s:%d on %s, siteId=%d, deviceId=%d, commandMode=%d, launching all nodes for takeoff",
                 req.remote_addr.c_str(),
                 req.remote_port,
                 req.local_addr.c_str(),
                 site_id,
                 device_id,
                 command_mode);
        LaunchNodesForTakeoff();
        SendResult(res, 1);
        return;
    }

    const bool success = CallCommandService(command_mode);

    SendResult(res, success ? 1 : 2);
}

// 处理/pointAngleSet角度设置请求：修改指定航线中对应航点的云台俯仰角 gimbal_pitch
void HttpServer::HandlePointAngleSet(const httplib::Request& req, httplib::Response& res) {
    int site_id = 0;
    int device_id = 0;
    int point_id = 0;
    double angle_deg = 0.0;
    std::string airline_key = FindParam(req, {"airlineKey"});

    bool angle_ok = ParseDoubleValue(FindParam(req, {"angle"}), &angle_deg);
    bool ok = ParseIntValue(FindParam(req, {"siteId", "siteld"}), &site_id) &&
              ParseIntValue(FindParam(req, {"deviceId", "deviceld"}), &device_id) &&
              ParseIntValue(FindParam(req, {"pointId", "pointid"}), &point_id) &&
              angle_ok && !airline_key.empty() &&
              site_id > 0 && device_id > 0 && point_id > 0;

    if (!ok && !req.body.empty()) {
        try {
            nlohmann::json body = nlohmann::json::parse(req.body);
            if (site_id <= 0) {
                site_id = body.value("siteId", body.value("siteld", 0));
            }
            if (device_id <= 0) {
                device_id = body.value("deviceId", body.value("deviceld", 0));
            }
            if (point_id <= 0) {
                point_id = body.value("pointId", body.value("pointid", 0));
            }
            if (airline_key.empty()) {
                airline_key = body.value("airlineKey", "");
            }
            if (!angle_ok && body.contains("angle") && body["angle"].is_number()) {
                angle_deg = body["angle"].get<double>();
                angle_ok = std::isfinite(angle_deg);
            }
            ok = (site_id > 0 && device_id > 0 && point_id > 0 &&
                  !airline_key.empty() && angle_ok);
        } catch (const std::exception&) {
            ok = false;
        }
    }

    if (!ok) {
        ROS_WARN("Rejected /pointAngleSet request from %s:%d due to invalid parameters",
                 req.remote_addr.c_str(),
                 req.remote_port);
        SendResult(res, 3);
        return;
    }

    const std::string waypoint_pkg = ros::package::getPath("indooruav_waypoint");
    const std::string waypoint_file = waypoint_pkg + "/waypoints/" + airline_key + ".yaml";
    {
        std::ifstream check(waypoint_file);
        if (!check.is_open()) {
            ROS_ERROR("Waypoint file not found for /pointAngleSet: %s", waypoint_file.c_str());
            SendResult(res, 2);
            return;
        }
    }

    std::string error;
    if (!UpdateWaypointGimbalPitch(waypoint_file, point_id, angle_deg, &error)) {
        ROS_ERROR("Failed to set gimbal pitch for airline=%s pointId=%d in %s: %s",
                  airline_key.c_str(), point_id, waypoint_file.c_str(), error.c_str());
        SendResult(res, 2);
        return;
    }

    ROS_INFO("Received /pointAngleSet from %s:%d, siteId=%d, deviceId=%d, airlineKey=%s, pointId=%d, gimbal_pitch=%.2f",
             req.remote_addr.c_str(),
             req.remote_port,
             site_id,
             device_id,
             airline_key.c_str(),
             point_id,
             angle_deg);
    SendResult(res, 1);
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

// 状态机状态回调，缓存当前状态
void HttpServer::StateCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_ = msg->data;
}

// 收到起飞命令时启动所有节点，30秒后自动发起飞指令
void HttpServer::LaunchNodesForTakeoff() {
    // 检查状态机状态，只有待机状态才允许起飞
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (current_state_ != "Await") {
            ROS_WARN("[Launch] State machine is '%s', not Await, ignoring takeoff",
                     current_state_.c_str());
            return;
        }
    }

    // 清理之前可能残留的定时器
    takeoff_timer_.stop();

    const std::string ws = ros::package::getPath("indooruav_http") + "/../..";
    const std::string shell_dir = ws + "/src/indooruav_core/shell";
    const std::string three_d_ws = std::string(std::getenv("HOME")) + "/Project/3D/catkin_ws";

    // 1. 启动雷达节点 (MID360)
    ROS_INFO("[Launch] Starting MID360 lidar...");
    system(("roslaunch " + three_d_ws + "/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch &").c_str());

    // 2. 启动定位节点
    ROS_INFO("[Launch] Starting localization...");
    system(("bash " + shell_dir + "/bringup_localize.sh &").c_str());

    // 3. 启动状态机节点
    ROS_INFO("[Launch] Starting state machine...");
    system(("bash " + shell_dir + "/bringup_indooruav_core.sh &").c_str());

    // 4. [已禁用] 实物控制节点由外部管理，启动时不自动启动
    //    设置 I2C 和串口权限（若实物控制节点已外部启动，权限已由外部设置）
    // ROS_INFO("[Launch] Setting device permissions...");
    // system("echo \"888888\" | sudo -S chmod 666 /dev/i2c-7");
    // system("echo \"888888\" | sudo -S chmod 777 /dev/ttyTHS0");
    // ROS_INFO("[Launch] Starting hardware controller...");
    // system(("bash " + shell_dir + "/bringup_controller_hardware.sh &").c_str());

    // 5. 启动航线跟踪节点
    ROS_INFO("[Launch] Starting waypoint tracker...");
    system(("bash " + shell_dir + "/bringup_waypoint_tracker.sh &").c_str());

    // 6. 启动降落节点
    ROS_INFO("[Launch] Starting landing node...");
    system("roslaunch indooruav_mission bringup_mission.launch &");

    // 7. 启动像素坐标发布脚本（用硬编码路径，直接指向 indooruav_localize/scripts）
    const std::string pixel_script = std::string(getenv("HOME")) + "/Project/IndoorUavInspection2/catkin_ws/src/indooruav_localize/scripts/odometry_to_pixel.py";
    ROS_INFO("[Launch] Starting pixel coordinate publisher: %s", pixel_script.c_str());
    system(("python3 " + pixel_script + " &").c_str());

    // 8. 30秒后自动发起飞指令
    ROS_INFO("[Launch] Will send takeoff command in 30 seconds...");
    takeoff_timer_ = nh_.createTimer(ros::Duration(30.0), &HttpServer::DelayedTakeoffCommand, this, /*oneshot=*/true);
}

void HttpServer::DelayedTakeoffCommand(const ros::TimerEvent& event) {
    ROS_INFO("[Launch] 30s delay elapsed, sending takeoff command...");
    CallCommandService(1);
}

}  // namespace indooruav_http
