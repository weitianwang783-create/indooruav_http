#include "http/http_client.h"

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#include <tf/transform_datatypes.h>

namespace indooruav_http {

namespace {
const char* kJsonContentType = "application/json";
const char* kRunPostLandWorkflowService = "/indooruav_http/run_post_land_workflow";
const char* kUploadImageBytesService = "/indooruav_http/upload_image_bytes";
std::mutex g_upload_sequence_mutex;
std::unordered_map<std::string, int> g_next_upload_sequence_by_detect_time;

std::string ToLowerCopy(const std::string& input) {
    std::string output = input;
    std::transform(output.begin(), output.end(), output.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return output;
}

int NextUploadSequenceForDetectTime(const std::string& detect_time_cur) {
    std::lock_guard<std::mutex> lock(g_upload_sequence_mutex);
    int& next_sequence = g_next_upload_sequence_by_detect_time[detect_time_cur];
    if (next_sequence <= 0) {
        next_sequence = 1;
    }
    return next_sequence++;
}

std::string JoinPath(const std::string& left, const std::string& right) {
    if (left.empty()) {
        return right;
    }
    if (left.back() == '/') {
        return left + right;
    }
    return left + "/" + right;
}
}  // namespace

HttpClient::HttpClient(ros::NodeHandle& nh,
                       const std::string& server_ip,
                       int server_port,
                       int site_id,
                       int device_id,
                       double device_interval,
                       double flight_interval,
                       int sample_rate,
                       double takeoff_interval)
    : nh_(nh)
    , server_ip_(server_ip)
    , server_port_(server_port)
    , site_id_(site_id)
    , device_id_(device_id)
    , device_state_interval_(device_interval)
    , flight_state_interval_(flight_interval)
    , flight_state_sample_rate_(sample_rate)
    , takeoff_state_interval_(takeoff_interval) {
    std::string base_url = "http://" + server_ip_ + ":" + std::to_string(server_port_);
    client_ = std::make_unique<httplib::Client>(base_url.c_str());

    client_->set_connection_timeout(5, 0);
    client_->set_read_timeout(10, 0);
    client_->set_write_timeout(10, 0);

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("airline_key", airline_key_, "AAAA");
    nh_private.param<std::string>("airline_info_topic", airline_info_topic_, "/indooruav_http/airline_info");
    nh_private.param<std::string>("airline_key_topic", airline_key_topic_, "/indooruav_http/airline_key");
    nh_private.param<std::string>("device_state_info_topic",
                                  device_state_info_topic_,
                                  "/indooruav_controller/http/device_state");
    nh_private.param<std::string>("takeoff_state_topic", takeoff_state_topic_, "/indooruav_http/takeoff_state");
    nh_private.param<std::string>("battery_topic", battery_topic_, "/battery_state");
    nh_private.param<std::string>("odom_topic", odom_topic_, "/Odometry_rotate");
    nh_private.param<std::string>("odom_fallback_topic", odom_fallback_topic_, "/Odometry");
    nh_private.param<std::string>("gimbal_topic", gimbal_topic_, "/gimbal/attitude");
    nh_private.param<std::string>("detection_topic", detection_topic_, "/detection/result");
    nh_private.param<bool>("enable_detection_error", enable_detection_error_, true);
    nh_private.param<double>("uav_online_timeout_sec", uav_online_timeout_sec_, 5.0);
    nh_private.param<std::string>("post_land_image_root_dir",
                                  post_land_image_root_dir_,
                                  "/tmp/indooruav_post_land_images");
    nh_private.param<std::string>("post_land_image_source_mode",
                                  post_land_image_source_mode_,
                                  "local_fs");
    nh_private.param<std::string>("controller_upload_mission_media_service",
                                  controller_upload_mission_media_service_,
                                  "/indooruav_controller/controller_hardware/upload_mission_photos_from_sd");

    nh_private.param<int>("uav_state", current_device_state_.uav_state, 1);
    nh_private.param<int>("control_state", current_device_state_.control_state, 1);
    nh_private.param<double>("control_soc", current_device_state_.control_soc, 0.0);
    nh_private.param<double>("control_rssi", current_device_state_.control_rssi, 0.0);
    nh_private.param<double>("battery_rssi", current_device_state_.battery_rssi, 0.0);
    nh_private.param<int>("battery_cycle_num", current_device_state_.battery_cycle_num, 0);
    nh_private.param<int>("takeoff_state", takeoff_state_, 1);

    SetupSubscribers(nh);
    SetupServices(nh);
    transfer_mission_media_client_ =
        nh.serviceClient<indooruav_msgs::TransferMissionMedia>(controller_upload_mission_media_service_);
}

HttpClient::~HttpClient() {
    if (post_land_workflow_thread_.joinable()) {
        post_land_workflow_thread_.join();
    }
}

void HttpClient::StartTimers() {
    device_state_timer_ = nh_.createTimer(
        ros::Duration(device_state_interval_),
        &HttpClient::DeviceStateTimerCallback,
        this);

    flight_state_timer_ = nh_.createTimer(
        ros::Duration(flight_state_interval_),
        &HttpClient::FlightStateTimerCallback,
        this);

    takeoff_state_timer_ = nh_.createTimer(
        ros::Duration(takeoff_state_interval_),
        &HttpClient::TakeoffStateTimerCallback,
        this);
}

void HttpClient::SetupSubscribers(ros::NodeHandle& nh) {
    battery_sub_ = nh.subscribe(battery_topic_, 10, &HttpClient::BatteryCallback, this);
    odom_sub_ = nh.subscribe(odom_topic_, 10, &HttpClient::OdomCallback, this);
    if (!odom_fallback_topic_.empty() && odom_fallback_topic_ != odom_topic_) {
        odom_fallback_sub_ = nh.subscribe(odom_fallback_topic_, 10, &HttpClient::OdomCallback, this);
    }
    gimbal_sub_ = nh.subscribe(gimbal_topic_, 10, &HttpClient::GimbalCallback, this);
    detection_sub_ = nh.subscribe(detection_topic_, 10, &HttpClient::DetectionCallback, this);
    airline_info_sub_ = nh.subscribe(airline_info_topic_, 10, &HttpClient::AirlineInfoCallback, this);
    airline_key_sub_ = nh.subscribe(airline_key_topic_, 10, &HttpClient::AirlineKeyCallback, this);
    device_state_info_sub_ = nh.subscribe(device_state_info_topic_, 10, &HttpClient::DeviceStateInfoCallback, this);
    takeoff_state_sub_ = nh.subscribe(takeoff_state_topic_, 10, &HttpClient::TakeoffStateTopicCallback, this);
}

void HttpClient::SetupServices(ros::NodeHandle& nh) {
    send_airline_service_ = nh.advertiseService(
        "/indooruav_http/send_airline",
        &HttpClient::HandleSendAirline,
        this);
    send_pic_service_ = nh.advertiseService(
        "/indooruav_http/send_pic",
        &HttpClient::HandleSendPic,
        this);
    upload_image_bytes_service_ = nh.advertiseService(
        kUploadImageBytesService,
        &HttpClient::HandleUploadImageBytes,
        this);
    set_takeoff_state_service_ = nh.advertiseService(
        "/indooruav_http/set_takeoff_state",
        &HttpClient::HandleSetTakeoffState,
        this);
    send_error_data_service_ = nh.advertiseService(
        "/indooruav_http/send_error_data",
        &HttpClient::HandleSendErrorData,
        this);
    send_fly_over_service_ = nh.advertiseService(
        "/indooruav_http/send_fly_over",
        &HttpClient::HandleSendFlyOver,
        this);
    send_pic_over_service_ = nh.advertiseService(
        "/indooruav_http/send_pic_over",
        &HttpClient::HandleSendPicOver,
        this);
    airline_sync_service_ = nh.advertiseService(
        "/indooruav_http/airline_sync",
        &HttpClient::HandleAirlineSync,
        this);
    run_post_land_workflow_service_ = nh.advertiseService(
        kRunPostLandWorkflowService,
        &HttpClient::HandleRunPostLandWorkflow,
        this);
}

void HttpClient::MarkTelemetryReceived() {
    last_telemetry_time_ = ros::Time::now();
}

void HttpClient::UpdateDerivedDeviceState() {
    if (HasFreshDeviceStateInfo()) {
        return;
    }

    int next_uav_state = 0;
    if (uav_online_timeout_sec_ <= 0.0) {
        next_uav_state = 1;
    } else if (!last_telemetry_time_.isZero()) {
        const double silence_sec = (ros::Time::now() - last_telemetry_time_).toSec();
        next_uav_state = silence_sec <= uav_online_timeout_sec_ ? 1 : 0;
    }

    if (current_device_state_.uav_state != next_uav_state) {
        ROS_INFO("uavState changed from %d to %d",
                 current_device_state_.uav_state,
                 next_uav_state);
        current_device_state_.uav_state = next_uav_state;
    }
}

bool HttpClient::HasFreshDeviceStateInfo() const {
    if (!has_device_state_info_) {
        return false;
    }

    if (uav_online_timeout_sec_ <= 0.0) {
        return true;
    }

    if (last_device_state_info_time_.isZero()) {
        return false;
    }

    const double silence_sec = (ros::Time::now() - last_device_state_info_time_).toSec();
    return silence_sec <= uav_online_timeout_sec_;
}

void HttpClient::BatteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg) {
    MarkTelemetryReceived();
    if (HasFreshDeviceStateInfo()) {
        return;
    }

    current_device_state_.battery_soc = msg->percentage * 100.0;
    current_device_state_.battery_volt = msg->voltage;
    current_device_state_.battery_temp = msg->temperature;
}

void HttpClient::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    MarkTelemetryReceived();

    const ros::Time now = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    const double min_interval = flight_state_sample_rate_ > 0
        ? 1.0 / static_cast<double>(flight_state_sample_rate_)
        : 1.0;

    if (!last_sample_time_.isZero() && (now - last_sample_time_).toSec() < min_interval) {
        return;
    }
    last_sample_time_ = now;

    FlightState state;
    state.time_stamp = GetTimeStamp();

    state.positionx = msg->pose.pose.position.x;
    state.positiony = msg->pose.pose.position.y;
    state.positionz = msg->pose.pose.position.z;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    m.getRPY(roll, pitch, yaw);

    state.attitude_roll = roll * 180.0 / M_PI;
    state.attitude_pitch = pitch * 180.0 / M_PI;
    state.attitude_yaw = yaw * 180.0 / M_PI;

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double vz = msg->twist.twist.linear.z;
    state.horizontal_speed = std::sqrt(vx * vx + vy * vy);
    state.vertical_speed = vz;

    state.line_type = 1;
    state.pose_angle_roll = gimbal_roll_;
    state.pose_angle_pitch = gimbal_pitch_;
    state.pose_angle_yaw = gimbal_yaw_;

    state.pantograph_is = pantograph_is_;
    state.abnormal_is = abnormal_is_;
    state.pantograph_locx = pantograph_locx_;
    state.pantograph_locy = pantograph_locy_;
    state.pantograph_locz = pantograph_locz_;
    state.abnormal_locx = abnormal_locx_;
    state.abnormal_locy = abnormal_locy_;
    state.abnormal_locz = abnormal_locz_;

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    flight_state_buffer_.push_back(state);

    const size_t max_size = static_cast<size_t>(
        std::max(1, flight_state_sample_rate_) * std::max(1.0, flight_state_interval_));
    while (flight_state_buffer_.size() > max_size) {
        flight_state_buffer_.pop_front();
    }
}

void HttpClient::GimbalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    MarkTelemetryReceived();
    tf::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(gimbal_roll_, gimbal_pitch_, gimbal_yaw_);
    gimbal_roll_ = gimbal_roll_ * 180.0 / M_PI;
    gimbal_pitch_ = gimbal_pitch_ * 180.0 / M_PI;
    gimbal_yaw_ = gimbal_yaw_ * 180.0 / M_PI;
}

void HttpClient::DetectionCallback(const std_msgs::String::ConstPtr& msg) {
    try {
        json j = json::parse(msg->data);
        if (j.contains("pantograph")) {
            pantograph_is_ = j["pantograph"].value("detected", false) ? 1 : 0;
            pantograph_locx_ = j["pantograph"].value("x", 0.0);
            pantograph_locy_ = j["pantograph"].value("y", 0.0);
            pantograph_locz_ = j["pantograph"].value("z", 0.0);

            if (enable_detection_error_ && pantograph_is_ == 1) {
                ErrorData error;
                error.time_stamp = GetTimeStamp();
                error.error_type = 11;
                error.error_info = json::array({pantograph_locx_, pantograph_locy_, pantograph_locz_}).dump();
                SendErrorData(error);
            }
        }

        if (j.contains("abnormal")) {
            abnormal_is_ = j["abnormal"].value("detected", false) ? 1 : 0;
            abnormal_locx_ = j["abnormal"].value("x", 0.0);
            abnormal_locy_ = j["abnormal"].value("y", 0.0);
            abnormal_locz_ = j["abnormal"].value("z", 0.0);

            if (enable_detection_error_ && abnormal_is_ == 1) {
                ErrorData error;
                error.time_stamp = GetTimeStamp();
                error.error_type = 12;
                error.error_info = json::array({abnormal_locx_, abnormal_locy_, abnormal_locz_}).dump();
                SendErrorData(error);
            }
        }
    } catch (const std::exception& e) {
        ROS_WARN("Failed to parse detection JSON: %s", e.what());
    }
}

void HttpClient::AirlineInfoCallback(const std_msgs::String::ConstPtr& msg) {
    try {
        json j = json::parse(msg->data);
        const int site_id = j.value("siteId", 0);
        const int device_id = j.value("deviceId", 0);
        const std::string airline_key = j.value("airlineKey", "");
        const std::string detect_time_cur = j.value("detectTimeCur", "");
        if (site_id <= 0 || device_id <= 0 || airline_key.empty() || detect_time_cur.empty()) {
            ROS_WARN("Ignored airline info update due to missing siteId, deviceId, airlineKey, or detectTimeCur");
            return;
        }

        std::lock_guard<std::mutex> lock(airline_mutex_);
        airline_info_site_id_ = site_id;
        airline_info_device_id_ = device_id;
        airline_info_airline_key_ = airline_key;
        airline_info_detect_time_cur_ = detect_time_cur;
        has_airline_info_context_ = true;
        airline_key_ = airline_key;
        detect_time_cur_ = detect_time_cur;
    } catch (const std::exception& e) {
        ROS_WARN("Failed to parse airline info JSON: %s", e.what());
    }
}

void HttpClient::AirlineKeyCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(airline_mutex_);
    airline_key_ = msg->data;
}

void HttpClient::DeviceStateInfoCallback(const std_msgs::String::ConstPtr& msg) {
    try {
        const json j = json::parse(msg->data);
        MarkTelemetryReceived();
        has_device_state_info_ = true;
        last_device_state_info_time_ = ros::Time::now();

        if (j.contains("uavState")) {
            current_device_state_.uav_state = j.value("uavState", current_device_state_.uav_state);
        }
        if (j.contains("controlState")) {
            current_device_state_.control_state = j.value("controlState", current_device_state_.control_state);
        }
        if (j.contains("controlSoc")) {
            current_device_state_.control_soc = j.value("controlSoc", current_device_state_.control_soc);
        }
        if (j.contains("controlRssi")) {
            current_device_state_.control_rssi = j.value("controlRssi", current_device_state_.control_rssi);
        }
        if (j.contains("batteryTemp")) {
            current_device_state_.battery_temp = j.value("batteryTemp", current_device_state_.battery_temp);
        }
        if (j.contains("batterySoc")) {
            current_device_state_.battery_soc = j.value("batterySoc", current_device_state_.battery_soc);
        }
        if (j.contains("batteryRssi")) {
            current_device_state_.battery_rssi = j.value("batteryRssi", current_device_state_.battery_rssi);
        }
        if (j.contains("batteryVolt")) {
            current_device_state_.battery_volt = j.value("batteryVolt", current_device_state_.battery_volt);
        }
        if (j.contains("batteryCycleNum")) {
            current_device_state_.battery_cycle_num = j.value("batteryCycleNum", current_device_state_.battery_cycle_num);
        }
    } catch (const std::exception& e) {
        ROS_WARN("Failed to parse device state info JSON: %s", e.what());
    }
}

void HttpClient::GetCurrentTargetIds(int* site_id, int* device_id) {
    if (site_id == nullptr || device_id == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(airline_mutex_);
    if (has_airline_info_context_) {
        *site_id = airline_info_site_id_;
        *device_id = airline_info_device_id_;
        return;
    }

    *site_id = site_id_;
    *device_id = device_id_;
}

bool HttpClient::GetAirlineInfoTargetIds(int* site_id, int* device_id) {
    if (site_id == nullptr || device_id == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(airline_mutex_);
    if (!has_airline_info_context_ ||
        airline_info_site_id_ <= 0 ||
        airline_info_device_id_ <= 0 ||
        airline_info_airline_key_.empty() ||
        airline_info_detect_time_cur_.empty()) {
        return false;
    }

    *site_id = airline_info_site_id_;
    *device_id = airline_info_device_id_;
    return true;
}

bool HttpClient::GetCurrentMissionContext(int* site_id,
                                          int* device_id,
                                          std::string* airline_key,
                                          std::string* detect_time_cur) {
    if (site_id == nullptr || device_id == nullptr ||
        airline_key == nullptr || detect_time_cur == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(airline_mutex_);
    if (!has_airline_info_context_ ||
        airline_info_site_id_ <= 0 ||
        airline_info_device_id_ <= 0 ||
        airline_info_airline_key_.empty() ||
        airline_info_detect_time_cur_.empty()) {
        return false;
    }

    *site_id = airline_info_site_id_;
    *device_id = airline_info_device_id_;
    *airline_key = airline_info_airline_key_;
    *detect_time_cur = airline_info_detect_time_cur_;
    return true;
}

bool HttpClient::ResolveTargetIdsForMission(const std::string& airline_key,
                                            const std::string& detect_time_cur,
                                            int* site_id,
                                            int* device_id) {
    if (site_id == nullptr || device_id == nullptr) {
        return false;
    }

    std::lock_guard<std::mutex> lock(airline_mutex_);
    if (has_airline_info_context_ &&
        airline_info_airline_key_ == airline_key &&
        airline_info_detect_time_cur_ == detect_time_cur) {
        *site_id = airline_info_site_id_;
        *device_id = airline_info_device_id_;
        return airline_info_site_id_ > 0 && airline_info_device_id_ > 0;
    }

    return false;
}

void HttpClient::TakeoffStateTopicCallback(const std_msgs::Int32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(takeoff_mutex_);
    takeoff_state_ = msg->data;
}

void HttpClient::DeviceStateTimerCallback(const ros::TimerEvent& event) {
    (void)event;
    UpdateDerivedDeviceState();
    HttpResult result = SendDeviceState(current_device_state_);
    if (result.result_code != 1) {
        ROS_WARN("sendDeviceData failed with resultCode=%d", result.result_code);
    }
}

void HttpClient::FlightStateTimerCallback(const ros::TimerEvent& event) {
    (void)event;

    std::vector<FlightState> batch;
    const size_t sample_count = static_cast<size_t>(
        std::max(1, flight_state_sample_rate_) * std::max(1.0, flight_state_interval_));

    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (flight_state_buffer_.size() < sample_count) {
            ROS_WARN_THROTTLE(5.0, "Not enough flight samples: %zu/%zu", flight_state_buffer_.size(), sample_count);
            return;
        }

        auto it = flight_state_buffer_.end() - static_cast<long>(sample_count);
        for (; it != flight_state_buffer_.end(); ++it) {
            batch.push_back(*it);
        }
    }

    HttpResult result = SendFlightStates(batch);
    if (result.result_code != 1) {
        ROS_WARN("sendFlyData failed with resultCode=%d", result.result_code);
    }
}

void HttpClient::TakeoffStateTimerCallback(const ros::TimerEvent& event) {
    (void)event;
    int state = 1;
    {
        std::lock_guard<std::mutex> lock(takeoff_mutex_);
        state = takeoff_state_;
    }

    HttpResult result = SendTakeoffState(state);
    if (result.result_code != 1) {
        ROS_WARN("sendTakeoffState failed with resultCode=%d", result.result_code);
    }
}

HttpResult HttpClient::SendAirline(const Airline& airline) {
    if (airline.airline_key.empty()) {
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    int site_id = 0;
    int device_id = 0;
    GetCurrentTargetIds(&site_id, &device_id);

    std::string path = "/sendAirline?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id);

    const std::string file_body = airline.ToJson().dump();
    httplib::UploadFormDataItems items;
    items.push_back({"file", file_body, "file.json", kJsonContentType});

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Post(path.c_str(), items));
}

HttpResult HttpClient::SendPic(const std::string& image_path) {
    int site_id = 0;
    int device_id = 0;
    std::string airline_key;
    std::string detect_time_cur;
    GetCurrentMissionContext(&site_id, &device_id, &airline_key, &detect_time_cur);

    return SendPicWithMission(site_id, device_id, image_path, airline_key, detect_time_cur);
}

HttpResult HttpClient::SendPicBytesWithMission(int site_id,
                                               int device_id,
                                               const std::string& image_extension,
                                               const std::vector<uint8_t>& image_bytes,
                                               const std::string& airline_key,
                                               const std::string& detect_time_cur) {
    if (image_bytes.empty()) {
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    if (site_id <= 0 || device_id <= 0 ||
        airline_key.empty() || detect_time_cur.empty()) {
        ROS_WARN("sendPic skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    const std::string extension = NormalizeImageExtension(image_extension);
    const std::string mime_type = GetImageMimeTypeByExtension(extension);
    const int upload_sequence = NextUploadSequenceForDetectTime(detect_time_cur);
    const std::string upload_filename = std::to_string(upload_sequence) + extension;
    const std::string file_bytes(reinterpret_cast<const char*>(image_bytes.data()), image_bytes.size());

    std::string path = "/sendPic?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id) +
                       "&airlineKey=" + airline_key +
                       "&detectTimeCur=" + detect_time_cur;

    httplib::UploadFormDataItems items;
    items.push_back({"file", file_bytes, upload_filename, mime_type});

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Post(path.c_str(), items));
}

HttpResult HttpClient::SendPicWithMission(int site_id,
                                          int device_id,
                                          const std::string& image_path,
                                          const std::string& airline_key,
                                          const std::string& detect_time_cur) {
    if (image_path.empty()) {
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    if (site_id <= 0 || device_id <= 0 ||
        airline_key.empty() || detect_time_cur.empty()) {
        ROS_WARN("sendPic skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::string file_bytes;
    if (!ReadBinaryFile(image_path, &file_bytes)) {
        ROS_WARN("sendPic skipped because image file is not readable: %s", image_path.c_str());
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::vector<uint8_t> bytes(file_bytes.begin(), file_bytes.end());
    return SendPicBytesWithMission(site_id,
                                   device_id,
                                   GetImageExtension(image_path),
                                   bytes,
                                   airline_key,
                                   detect_time_cur);
}

HttpResult HttpClient::SendDeviceState(const DeviceState& device_state) {
    int site_id = 0;
    int device_id = 0;
    if (!GetAirlineInfoTargetIds(&site_id, &device_id)) {
        ROS_WARN("sendDeviceData skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::string path = "/sendDeviceData?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id);

    const std::string file_body = device_state.ToJson().dump();
    httplib::UploadFormDataItems items;
    items.push_back({"file", file_body, "file.json", kJsonContentType});

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Post(path.c_str(), items));
}

HttpResult HttpClient::SendFlightStates(const std::vector<FlightState>& flight_states) {
    int site_id = 0;
    int device_id = 0;
    std::string airline_key;
    std::string detect_time_cur;
    GetCurrentMissionContext(&site_id, &device_id, &airline_key, &detect_time_cur);

    if (site_id <= 0 || device_id <= 0 ||
        airline_key.empty() || detect_time_cur.empty()) {
        ROS_WARN("sendFlyData skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::string path = "/sendFlyData?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id) +
                       "&airlineKey=" + airline_key +
                       "&detectTimeCur=" + detect_time_cur;

    json payload = json::array();
    for (const auto& state : flight_states) {
        payload.push_back(state.ToJson());
    }

    httplib::UploadFormDataItems items;
    items.push_back({"file", payload.dump(), "file.json", kJsonContentType});

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Post(path.c_str(), items));
}

HttpResult HttpClient::SendErrorData(const ErrorData& error_data) {
    int site_id = 0;
    int device_id = 0;
    std::string airline_key;
    std::string detect_time_cur;
    GetCurrentMissionContext(&site_id, &device_id, &airline_key, &detect_time_cur);

    if (site_id <= 0 || device_id <= 0 ||
        airline_key.empty() || detect_time_cur.empty()) {
        ROS_WARN("sendErrorData skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::string path = "/sendErrorData?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id) +
                       "&airlineKey=" + airline_key +
                       "&detectTimeCur=" + detect_time_cur;

    httplib::UploadFormDataItems items;
    items.push_back({"file", error_data.ToJson().dump(), "file.json", kJsonContentType});

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Post(path.c_str(), items));
}

HttpResult HttpClient::SendTakeoffState(int takeoff_state) {
    int site_id = 0;
    int device_id = 0;
    if (!GetAirlineInfoTargetIds(&site_id, &device_id)) {
        ROS_WARN("sendTakeoffState skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::string path = "/sendTakeoffState?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id) +
                       "&takeoffState=" + std::to_string(takeoff_state);

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Get(path.c_str()));
}

HttpResult HttpClient::SendFlyOver() {
    int site_id = 0;
    int device_id = 0;
    std::string airline_key;
    std::string detect_time_cur;
    GetCurrentMissionContext(&site_id, &device_id, &airline_key, &detect_time_cur);

    return SendFlyOverWithMission(site_id, device_id, airline_key, detect_time_cur);
}

HttpResult HttpClient::SendFlyOverWithMission(int site_id,
                                              int device_id,
                                              const std::string& airline_key,
                                              const std::string& detect_time_cur) {
    if (site_id <= 0 || device_id <= 0 ||
        airline_key.empty() || detect_time_cur.empty()) {
        ROS_WARN("sendFlyOver skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::string path = "/sendFlyOver?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id) +
                       "&airlineKey=" + airline_key +
                       "&detectTimeCur=" + detect_time_cur;

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Get(path.c_str()));
}

HttpResult HttpClient::SendPicOver() {
    int site_id = 0;
    int device_id = 0;
    std::string airline_key;
    std::string detect_time_cur;
    GetCurrentMissionContext(&site_id, &device_id, &airline_key, &detect_time_cur);

    return SendPicOverWithMission(site_id, device_id, airline_key, detect_time_cur);
}

HttpResult HttpClient::SendPicOverWithMission(int site_id,
                                              int device_id,
                                              const std::string& airline_key,
                                              const std::string& detect_time_cur) {
    if (site_id <= 0 || device_id <= 0 ||
        airline_key.empty() || detect_time_cur.empty()) {
        ROS_WARN("sendPicOver skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::string path = "/sendPicOver?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id) +
                       "&airlineKey=" + airline_key +
                       "&detectTimeCur=" + detect_time_cur;

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Get(path.c_str()));
}

HttpResult HttpClient::AirlineSync() {
    int site_id = 0;
    int device_id = 0;
    if (!GetAirlineInfoTargetIds(&site_id, &device_id)) {
        ROS_WARN("airlineSync skipped because siteId, deviceId, airlineKey, or detectTimeCur from /airlineInfo is missing");
        HttpResult result;
        result.result_code = 3;
        return result;
    }

    std::string path = "/airlineSync?siteId=" + std::to_string(site_id) +
                       "&deviceId=" + std::to_string(device_id);

    std::lock_guard<std::mutex> lock(http_mutex_);
    return ParseResult(client_->Get(path.c_str()));
}

HttpResult HttpClient::ParseResult(const httplib::Result& result) {
    HttpResult output;
    if (!result) {
        output.result_code = 2;
        return output;
    }

    output.raw_body = result->body;

    if (result->status != 200) {
        output.result_code = 2;
        return output;
    }

    try {
        json body = json::parse(result->body);
        output.result_code = body.value("resultCode", 2);
        if (body.contains("result")) {
            output.result = body["result"];
        }
    } catch (const std::exception&) {
        output.result_code = 2;
    }

    return output;
}

std::string HttpClient::GetTimeStamp() const {
    std::time_t now = std::time(nullptr);
    std::tm tm_now = *std::localtime(&now);
    std::ostringstream ss;
    ss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

bool HttpClient::ReadBinaryFile(const std::string& image_path, std::string* file_bytes) const {
    if (file_bytes == nullptr) {
        return false;
    }

    std::ifstream file(image_path, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    std::ostringstream buffer;
    buffer << file.rdbuf();
    if (file.bad()) {
        return false;
    }

    *file_bytes = buffer.str();
    return true;
}

std::string HttpClient::GetImageExtension(const std::string& image_path) const {
    const std::size_t slash_pos = image_path.find_last_of("/\\");
    const std::size_t dot_pos = image_path.find_last_of('.');
    if (dot_pos == std::string::npos || (slash_pos != std::string::npos && dot_pos < slash_pos)) {
        return ".bin";
    }
    return NormalizeImageExtension(image_path.substr(dot_pos));
}

std::string HttpClient::NormalizeImageExtension(const std::string& image_extension) const {
    if (image_extension.empty()) {
        return ".bin";
    }

    std::string normalized = ToLowerCopy(image_extension);
    if (!normalized.empty() && normalized.front() != '.') {
        normalized = "." + normalized;
    }
    return normalized;
}

std::string HttpClient::GetImageMimeTypeByExtension(const std::string& image_extension) const {
    const std::string extension = NormalizeImageExtension(image_extension);
    if (extension == ".jpg" || extension == ".jpeg") {
        return "image/jpeg";
    }
    if (extension == ".png") {
        return "image/png";
    }
    if (extension == ".bmp") {
        return "image/bmp";
    }
    if (extension == ".webp") {
        return "image/webp";
    }
    if (extension == ".dng") {
        return "image/x-adobe-dng";
    }
    if (extension == ".tif" || extension == ".tiff") {
        return "image/tiff";
    }
    return "application/octet-stream";
}

std::string HttpClient::GetImageMimeType(const std::string& image_path) const {
    return GetImageMimeTypeByExtension(GetImageExtension(image_path));
}

bool HttpClient::IsSupportedImageExtension(const std::string& extension) const {
    const std::string normalized = NormalizeImageExtension(extension);
    return normalized == ".jpg" ||
           normalized == ".jpeg" ||
           normalized == ".png" ||
           normalized == ".bmp" ||
           normalized == ".webp" ||
           normalized == ".dng" ||
           normalized == ".tif" ||
           normalized == ".tiff";
}

std::vector<std::string> HttpClient::CollectPostLandImages(const std::string& detect_time_cur) const {
    std::vector<std::string> image_paths;
    if (post_land_image_root_dir_.empty()) {
        ROS_WARN("post_land_image_root_dir is empty, skipping image scan for detectTimeCur=%s",
                 detect_time_cur.c_str());
        return image_paths;
    }

    const std::string image_dir = JoinPath(post_land_image_root_dir_, detect_time_cur);
    struct stat directory_stat;
    if (stat(image_dir.c_str(), &directory_stat) != 0 || !S_ISDIR(directory_stat.st_mode)) {
        ROS_INFO("Post-land image directory does not exist or is not a directory: %s",
                 image_dir.c_str());
        return image_paths;
    }

    DIR* dir = opendir(image_dir.c_str());
    if (dir == nullptr) {
        ROS_WARN("Failed to open post-land image directory: %s", image_dir.c_str());
        return image_paths;
    }

    while (const dirent* entry = readdir(dir)) {
        const std::string file_name(entry->d_name);
        if (file_name == "." || file_name == "..") {
            continue;
        }

        const std::string image_path = JoinPath(image_dir, file_name);
        struct stat file_stat;
        if (stat(image_path.c_str(), &file_stat) != 0 || !S_ISREG(file_stat.st_mode)) {
            continue;
        }

        if (!IsSupportedImageExtension(GetImageExtension(file_name))) {
            continue;
        }

        image_paths.push_back(image_path);
    }

    closedir(dir);
    std::sort(image_paths.begin(), image_paths.end());
    return image_paths;
}

indooruav_msgs::TransferMissionMedia::Response HttpClient::TransferMissionMediaFromController(
    const std::string& airline_key,
    const std::string& detect_time_cur) {
    indooruav_msgs::TransferMissionMedia service;
    service.request.airline_key = airline_key;
    service.request.detect_time_cur = detect_time_cur;

    if (!transfer_mission_media_client_.call(service)) {
        indooruav_msgs::TransferMissionMedia::Response fallback;
        fallback.result_code = 2;
        fallback.matched_count = 0;
        fallback.uploaded_count = 0;
        fallback.failed_count = 0;
        ROS_WARN("Post-land workflow failed to call controller media transfer service [%s]",
                 controller_upload_mission_media_service_.c_str());
        return fallback;
    }

    return service.response;
}

void HttpClient::StartPostLandWorkflow() {
    if (post_land_workflow_thread_.joinable()) {
        post_land_workflow_thread_.join();
    }
    post_land_workflow_thread_ = std::thread(&HttpClient::RunPostLandWorkflow, this);
}

void HttpClient::RunPostLandWorkflow() {
    struct RunningFlagGuard {
        std::atomic<bool>& flag;
        ~RunningFlagGuard() {
            flag.store(false);
        }
    } running_flag_guard{post_land_workflow_running_};

    int site_id = 0;
    int device_id = 0;
    std::string airline_key;
    std::string detect_time_cur;
    GetCurrentMissionContext(&site_id, &device_id, &airline_key, &detect_time_cur);

    if (airline_key.empty() || detect_time_cur.empty()) {
        ROS_WARN("Post-land workflow skipped because airlineKey or detectTimeCur from /airlineInfo is missing");
        return;
    }

    ROS_INFO("Post-land workflow started: detectTimeCur=%s, image_source_mode=%s, image_root=%s",
             detect_time_cur.c_str(),
             post_land_image_source_mode_.c_str(),
             post_land_image_root_dir_.c_str());

    const HttpResult fly_over_result = SendFlyOverWithMission(site_id,
                                                              device_id,
                                                              airline_key,
                                                              detect_time_cur);
    if (fly_over_result.result_code != 1) {
        ROS_WARN("Post-land workflow sendFlyOver failed with resultCode=%d, continuing upload flow",
                 fly_over_result.result_code);
    } else {
        ROS_INFO("Post-land workflow sendFlyOver succeeded for detectTimeCur=%s",
                 detect_time_cur.c_str());
    }

    if (post_land_image_source_mode_ == "drone_sd_card") {
        const indooruav_msgs::TransferMissionMedia::Response media_response =
            TransferMissionMediaFromController(airline_key, detect_time_cur);
        if (media_response.result_code != 1) {
            ROS_WARN("Post-land workflow controller SD-card transfer finished with resultCode=%d, matched=%d, uploaded=%d, failed=%d",
                     media_response.result_code,
                     media_response.matched_count,
                     media_response.uploaded_count,
                     media_response.failed_count);
        } else {
            ROS_INFO("Post-land workflow controller SD-card transfer succeeded, matched=%d, uploaded=%d, failed=%d",
                     media_response.matched_count,
                     media_response.uploaded_count,
                     media_response.failed_count);
        }
    } else {
        const std::vector<std::string> image_paths = CollectPostLandImages(detect_time_cur);
        if (image_paths.empty()) {
            ROS_INFO("Post-land workflow found no images for detectTimeCur=%s",
                     detect_time_cur.c_str());
        } else {
            for (const std::string& image_path : image_paths) {
                const HttpResult send_pic_result = SendPicWithMission(site_id,
                                                                      device_id,
                                                                      image_path,
                                                                      airline_key,
                                                                      detect_time_cur);
                if (send_pic_result.result_code != 1) {
                    ROS_WARN("Post-land workflow sendPic failed for [%s] with resultCode=%d, continuing",
                             image_path.c_str(),
                             send_pic_result.result_code);
                    continue;
                }
                ROS_INFO("Post-land workflow uploaded image: %s", image_path.c_str());
            }
        }
    }

    const HttpResult pic_over_result = SendPicOverWithMission(site_id,
                                                              device_id,
                                                              airline_key,
                                                              detect_time_cur);
    if (pic_over_result.result_code != 1) {
        ROS_WARN("Post-land workflow sendPicOver failed with resultCode=%d",
                 pic_over_result.result_code);
    } else {
        ROS_INFO("Post-land workflow sendPicOver succeeded for detectTimeCur=%s",
                 detect_time_cur.c_str());
    }

    ROS_INFO("Post-land workflow finished for detectTimeCur=%s", detect_time_cur.c_str());
}

bool HttpClient::HandleSendAirline(indooruav_http::SendAirline::Request& req,
                                   indooruav_http::SendAirline::Response& res) {
    Airline airline;
    airline.airline_key = req.airline_key;
    airline.airline_map = req.airline_map;
    try {
        json list = json::parse(req.waypoint_list_json);
        if (!list.is_array()) {
            res.result_code = 3;
            return true;
        }
        for (const auto& item : list) {
            airline.waypoint_list.push_back(Waypoint::FromJson(item));
        }
    } catch (const std::exception&) {
        res.result_code = 3;
        return true;
    }

    HttpResult result = SendAirline(airline);
    res.result_code = result.result_code;
    if (result.result_code == 1) {
        std::lock_guard<std::mutex> lock(airline_mutex_);
        airline_key_ = airline.airline_key;
        detect_time_cur_.clear();
    }
    return true;
}

bool HttpClient::HandleSendPic(indooruav_http::SendPic::Request& req,
                               indooruav_http::SendPic::Response& res) {
    HttpResult result = SendPic(req.image_path);
    res.result_code = result.result_code;
    return true;
}

bool HttpClient::HandleUploadImageBytes(indooruav_msgs::UploadImageBytes::Request& req,
                                        indooruav_msgs::UploadImageBytes::Response& res) {
    int site_id = 0;
    int device_id = 0;
    if (!ResolveTargetIdsForMission(req.airline_key, req.detect_time_cur, &site_id, &device_id)) {
        ROS_WARN("Failed to resolve siteId/deviceId from /airlineInfo for detectTimeCur=%s",
                 req.detect_time_cur.c_str());
        res.result_code = 3;
        return true;
    }

    const HttpResult result = SendPicBytesWithMission(site_id,
                                                      device_id,
                                                      req.image_extension,
                                                      req.image_bytes,
                                                      req.airline_key,
                                                      req.detect_time_cur);
    if (result.result_code == 1) {
        ROS_INFO("Uploaded image bytes from source [%s] for detectTimeCur=%s",
                 req.source_name.c_str(),
                 req.detect_time_cur.c_str());
    } else {
        ROS_WARN("Failed to upload image bytes from source [%s] for detectTimeCur=%s, resultCode=%d",
                 req.source_name.c_str(),
                 req.detect_time_cur.c_str(),
                 result.result_code);
    }
    res.result_code = result.result_code;
    return true;
}

bool HttpClient::HandleSetTakeoffState(indooruav_http::TakeoffState::Request& req,
                                       indooruav_http::TakeoffState::Response& res) {
    {
        std::lock_guard<std::mutex> lock(takeoff_mutex_);
        takeoff_state_ = req.takeoff_state;
    }
    res.result_code = 1;
    return true;
}

bool HttpClient::HandleSendErrorData(indooruav_http::SendErrorData::Request& req,
                                     indooruav_http::SendErrorData::Response& res) {
    ErrorData error;
    error.time_stamp = GetTimeStamp();
    error.error_type = req.error_type;
    error.error_info = req.error_info;

    HttpResult result = SendErrorData(error);
    res.result_code = result.result_code;
    return true;
}

bool HttpClient::HandleSendFlyOver(indooruav_http::SendFlyOver::Request& req,
                                   indooruav_http::SendFlyOver::Response& res) {
    (void)req;
    HttpResult result = SendFlyOver();
    res.result_code = result.result_code;
    return true;
}

bool HttpClient::HandleSendPicOver(indooruav_http::SendPicOver::Request& req,
                                   indooruav_http::SendPicOver::Response& res) {
    (void)req;
    HttpResult result = SendPicOver();
    res.result_code = result.result_code;
    return true;
}

bool HttpClient::HandleAirlineSync(indooruav_http::AirlineSync::Request& req,
                                   indooruav_http::AirlineSync::Response& res) {
    (void)req;
    HttpResult result = AirlineSync();
    res.result_code = result.result_code;
    res.result_json = result.raw_body;
    return true;
}

bool HttpClient::HandleRunPostLandWorkflow(std_srvs::Empty::Request& req,
                                           std_srvs::Empty::Response& res) {
    (void)req;
    (void)res;

    if (post_land_workflow_running_.exchange(true)) {
        ROS_WARN("Post-land workflow trigger ignored because a previous workflow is still running");
        return true;
    }

    try {
        StartPostLandWorkflow();
        ROS_INFO("Accepted post-land workflow trigger via service [%s]",
                 kRunPostLandWorkflowService);
    } catch (const std::exception& e) {
        post_land_workflow_running_.store(false);
        ROS_ERROR("Failed to start post-land workflow thread: %s", e.what());
    }

    return true;
}

}  // namespace indooruav_http
