#include "http/http_client.h"

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#include <ros/package.h>
#include <tf/transform_datatypes.h>

namespace indooruav_http {

namespace {
const char* kJsonContentType = "application/json";
const char* kRunPostLandWorkflowService = "/indooruav_http/run_post_land_workflow";
const char* kUploadImageBytesService = "/indooruav_http/upload_image_bytes";
const char* kCacheAirlineMetaService = "/indooruav_http/cache_airline_meta";
const char* kWaypointSaveProxyService = "indooruav_controller/waypoint_recorder/save";
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
                       double flight_interval,
                       int sample_rate,
                       double takeoff_interval)
    : nh_(nh)
    , server_ip_(server_ip)
    , server_port_(server_port)
    , site_id_(site_id)
    , device_id_(device_id)
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
    nh_private.param<std::string>("takeoff_state_topic", takeoff_state_topic_, "/indooruav_http/takeoff_state");
    nh_private.param<std::string>("odom_topic", odom_topic_, "/Odometry_rotate");
    nh_private.param<std::string>("odom_px_topic", odom_px_topic_, "/Odometry_px");
    nh_private.param<std::string>("odom_waypoints_id_topic", odom_waypoints_id_topic_, "/Odometry_waypointsid");
    nh_private.param<std::string>("odom_fallback_topic", odom_fallback_topic_, "/Odometry");
    nh_private.param<std::string>("gimbal_topic", gimbal_topic_, "/gimbal/attitude");
    nh_private.param<std::string>("detection_topic", detection_topic_, "/detection/result");
    nh_private.param<bool>("enable_detection_error", enable_detection_error_, true);
    nh_private.param<std::string>("post_land_image_root_dir",
                                  post_land_image_root_dir_,
                                  "/tmp/indooruav_post_land_images");
    nh_private.param<std::string>("post_land_image_source_mode",
                                  post_land_image_source_mode_,
                                  "local_fs");
    nh_private.param<std::string>("controller_upload_mission_media_service",
                                  controller_upload_mission_media_service_,
                                  "/indooruav_controller/controller_hardware/upload_mission_photos_from_sd");
    nh_private.param<std::string>("waypoint_save_raw_service",
                                  waypoint_save_raw_service_,
                                  "indooruav_controller/waypoint_recorder/save_raw");
    nh_private.param<std::string>("waypoint_yaml_dir",
                                  waypoint_yaml_dir_,
                                  "config");
    nh_private.param<double>("waypoint_poll_interval_sec",
                             waypoint_poll_interval_sec_,
                             5.0);
    if (waypoint_poll_interval_sec_ < 1.0) {
        waypoint_poll_interval_sec_ = 1.0;
    }
    nh_private.param<double>("default_waypoint_xscale",
                             default_waypoint_xscale_,
                             15.8);
    nh_private.param<double>("default_waypoint_yscale",
                             default_waypoint_yscale_,
                             15.8);
    nh_private.param<double>("default_waypoint_xzero",
                             default_waypoint_xzero_,
                             0.0);
    nh_private.param<double>("default_waypoint_yzero",
                             default_waypoint_yzero_,
                             0.0);
    nh_private.param<std::string>("waypoint_map2d_dir",
                                  waypoint_map2d_dir_,
                                  "");
    nh_private.param<std::string>("ftp_server_ip",
                                  ftp_server_ip_,
                                  "");
    nh_private.param<int>("ftp_server_port",
                          ftp_server_port_,
                          21);
    nh_private.param<std::string>("ftp_user",
                                  ftp_user_,
                                  "");
    nh_private.param<std::string>("ftp_password",
                                  ftp_password_,
                                  "");
    nh_private.param<std::string>("ftp_remote_map_dir",
                                  ftp_remote_map_dir_,
                                  "/imgs/");
    if (!waypoint_map2d_dir_.empty() && waypoint_map2d_dir_.front() != '/') {
        const std::string pkg_path = ros::package::getPath("FASTLIO2_SAM_LC");
        if (!pkg_path.empty()) {
            waypoint_map2d_dir_ = JoinPath(pkg_path, waypoint_map2d_dir_);
        }
    }
    if (!waypoint_yaml_dir_.empty() && waypoint_yaml_dir_.front() != '/') {
        const std::string pkg_path = ros::package::getPath("indooruav_waypoint");
        if (!pkg_path.empty()) {
            waypoint_yaml_dir_ = JoinPath(pkg_path, waypoint_yaml_dir_);
        }
    }

    {
        const std::string mtime_file = JoinPath(waypoint_yaml_dir_, ".airline_mtime.json");
        std::ifstream in(mtime_file);
        if (in.is_open()) {
            try {
                json j = json::parse(in);
                for (auto& [path, mtime] : j.items()) {
                    waypoint_file_mtime_store_[path] = static_cast<std::time_t>(mtime.get<int64_t>());
                }
                ROS_INFO("Loaded %zu waypoint mtime entries from %s",
                         waypoint_file_mtime_store_.size(), mtime_file.c_str());
            } catch (const std::exception& e) {
                ROS_WARN("Failed to parse waypoint mtime file [%s]: %s",
                         mtime_file.c_str(), e.what());
            }
        }
    }

    nh_private.param<int>("takeoff_state", takeoff_state_, 1);

    SetupSubscribers(nh);
    SetupServices(nh);
    transfer_mission_media_client_ =
        nh.serviceClient<indooruav_msgs::TransferMissionMedia>(controller_upload_mission_media_service_);
    waypoint_save_raw_client_ =
        nh.serviceClient<std_srvs::Trigger>(waypoint_save_raw_service_);
}

HttpClient::~HttpClient() {
    if (post_land_workflow_thread_.joinable()) {
        post_land_workflow_thread_.join();
    }
}

void HttpClient::StartTimers() {
    flight_state_timer_ = nh_.createTimer(
        ros::Duration(flight_state_interval_),
        &HttpClient::FlightStateTimerCallback,
        this);

    takeoff_state_timer_ = nh_.createTimer(
        ros::Duration(takeoff_state_interval_),
        &HttpClient::TakeoffStateTimerCallback,
        this);

    waypoint_poll_timer_ = nh_.createTimer(
        ros::Duration(waypoint_poll_interval_sec_),
        &HttpClient::WaypointPollTimerCallback,
        this);
}

void HttpClient::SetupSubscribers(ros::NodeHandle& nh) {
    odom_sub_ = nh.subscribe(odom_topic_, 10, &HttpClient::OdomCallback, this);
    if (!odom_fallback_topic_.empty() && odom_fallback_topic_ != odom_topic_) {
        odom_fallback_sub_ = nh.subscribe(odom_fallback_topic_, 10, &HttpClient::OdomCallback, this);
    }
    gimbal_sub_ = nh.subscribe(gimbal_topic_, 10, &HttpClient::GimbalCallback, this);
    odom_px_sub_ = nh.subscribe(odom_px_topic_, 10, &HttpClient::OdomPxCallback, this);
    odom_waypoints_id_sub_ = nh.subscribe(odom_waypoints_id_topic_, 10, &HttpClient::OdomWaypointsIdCallback, this);
    detection_sub_ = nh.subscribe(detection_topic_, 10, &HttpClient::DetectionCallback, this);
    airline_info_sub_ = nh.subscribe(airline_info_topic_, 10, &HttpClient::AirlineInfoCallback, this);
    airline_key_sub_ = nh.subscribe(airline_key_topic_, 10, &HttpClient::AirlineKeyCallback, this);
    takeoff_state_sub_ = nh.subscribe(takeoff_state_topic_, 10, &HttpClient::TakeoffStateTopicCallback, this);
}

void HttpClient::SetupServices(ros::NodeHandle& nh) {
    cache_airline_meta_service_ = nh.advertiseService(
        kCacheAirlineMetaService,
        &HttpClient::HandleCacheAirlineMeta,
        this);
    send_airline_service_ = nh.advertiseService(
        "/indooruav_http/send_airline",
        &HttpClient::HandleSendAirline,
        this);
    send_pic_service_ = nh.advertiseService(
        "/indooruav_http/send_pic",
        &HttpClient::HandleSendPic,
        this);
    waypoint_save_proxy_service_ = nh.advertiseService(
        kWaypointSaveProxyService,
        &HttpClient::HandleWaypointSaveProxy,
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
    resend_all_airlines_service_ = nh.advertiseService(
        "/indooruav_http/resend_all_airlines",
        &HttpClient::HandleResendAllAirlines,
        this);
}

void HttpClient::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
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
    state.px = odom_px_;
    state.py = odom_py_;
    state.point_id = latest_waypoint_id_;

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    flight_state_buffer_.push_back(state);

    const size_t max_size = static_cast<size_t>(
        std::max(1, flight_state_sample_rate_) * std::max(1.0, flight_state_interval_));
    while (flight_state_buffer_.size() > max_size) {
        flight_state_buffer_.pop_front();
    }
}

void HttpClient::OdomPxCallback(const geometry_msgs::Point::ConstPtr& msg) {
    odom_px_ = msg->x;
    odom_py_ = msg->y;
}

void HttpClient::OdomWaypointsIdCallback(const std_msgs::UInt32::ConstPtr& msg) {
    latest_waypoint_id_ = static_cast<int>(msg->data);
}

void HttpClient::GimbalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
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
    GetCurrentTargetIds(&site_id, &device_id);

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
        ROS_WARN_THROTTLE(10.0, "HTTP request failed: connection error or timeout");
        return output;
    }

    output.raw_body = result->body;

    if (result->status != 200) {
        output.result_code = 2;
        ROS_WARN_THROTTLE(10.0, "HTTP request failed: status=%d, body=%s",
                          result->status, result->body.c_str());
        return output;
    }

    try {
        json body = json::parse(result->body);
        output.result_code = body.value("resultCode", 2);
        if (body.contains("result")) {
            output.result = body["result"];
        }
    } catch (const std::exception& e) {
        output.result_code = 2;
        ROS_WARN_THROTTLE(10.0, "HTTP response parse error: %s, body=%s",
                          e.what(), result->body.substr(0, 200).c_str());
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

void HttpClient::FillWaypointPxPy(Waypoint& waypoint) const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (flight_state_buffer_.empty()) {
        return;
    }

    double best_dist = std::numeric_limits<double>::max();
    const FlightState* best = nullptr;
    for (const auto& state : flight_state_buffer_) {
        double dx = state.positionx - waypoint.waypointx;
        double dy = state.positiony - waypoint.waypointy;
        double dz = state.positionz - waypoint.waypointz;
        double dist = dx * dx + dy * dy + dz * dz;
        if (dist < best_dist) {
            best_dist = dist;
            best = &state;
        }
    }

    if (best != nullptr) {
        waypoint.px = best->px;
        waypoint.py = best->py;
    }
}

bool HttpClient::BuildRecordedWaypointAirlineFromYaml(const std::string& yaml_path,
                                                  Airline* airline,
                                                  size_t* manual_waypoint_count,
                                                  std::string* error_message) const {
    if (airline == nullptr || manual_waypoint_count == nullptr) {
        if (error_message != nullptr) {
            *error_message = "Invalid output pointer.";
        }
        return false;
    }

    try {
        YAML::Node root = YAML::LoadFile(yaml_path);
        const YAML::Node waypoints = root["waypoints"];
        if (!waypoints || !waypoints.IsSequence()) {
            if (error_message != nullptr) {
                *error_message = "YAML missing 'waypoints' sequence.";
            }
            return false;
        }

        std::string basename = yaml_path.substr(yaml_path.find_last_of('/') + 1);
        if (basename.size() > 5 && basename.compare(basename.size() - 5, 5, ".yaml") == 0) {
            basename = basename.substr(0, basename.size() - 5);
        }
        airline->airline_key = basename;
airline->airline_map = "/P_P/" + basename + ".png";
        airline->xscale = root["map2d_scale_x"] ? root["map2d_scale_x"].as<double>() : default_waypoint_xscale_;
        airline->yscale = root["map2d_scale_y"] ? root["map2d_scale_y"].as<double>() : default_waypoint_yscale_;
        airline->xzero = root["map2d_origin_px_x"] ? root["map2d_origin_px_x"].as<double>() : default_waypoint_xzero_;
        airline->yzero = root["map2d_origin_px_y"] ? root["map2d_origin_px_y"].as<double>() : default_waypoint_yzero_;
        airline->angle = root["map2d_rotation_deg"] ? root["map2d_rotation_deg"].as<double>() : 0.0;
        airline->waypoint_list.clear();
        *manual_waypoint_count = 0;

        for (std::size_t i = 0; i < waypoints.size(); ++i) {
            const YAML::Node node = waypoints[i];
            const bool stop = node["stop"] ? node["stop"].as<bool>() : false;
            if (!stop) {
                continue;
            }

            Waypoint waypoint;
            waypoint.waypointx = node["x"].as<double>();
            waypoint.waypointy = node["y"].as<double>();
            waypoint.waypointz = node["z"].as<double>();
            waypoint.angle = node["yaw_deg"].as<double>();
            waypoint.distance = 0.0;
            // 从 waypoints_pixel 目录读取对应像素文件，按 stop=true 的索引顺序填充 px,py
            {
                static std::string cached_pixel_basename;
                static std::vector<std::pair<double, double>> cached_pixels;
                if (cached_pixel_basename != basename) {
                    cached_pixel_basename = basename;
                    cached_pixels.clear();
                    const std::string pixel_dir = ros::package::getPath("indooruav_waypoint") + "/waypoints_pixel/";
                    const std::string pixel_file = pixel_dir + basename + "_pixel.yaml";
                    std::ifstream in(pixel_file);
                    if (!in.is_open()) {
                        ROS_WARN("Pixel file not found: %s (px/py will be 0)", pixel_file.c_str());
                    } else {
                        std::string line;
                        while (std::getline(in, line)) {
                            line.erase(0, line.find_first_not_of(" \t\r\n"));
                            line.erase(line.find_last_not_of(" \t\r\n") + 1);
                            if (line.empty()) continue;
                            try {
                                YAML::Node doc = YAML::Load(line);
                                if (doc["px"] && doc["py"]) {
                                    cached_pixels.emplace_back(doc["px"].as<double>(),
                                                               doc["py"].as<double>());
                                }
                            } catch (const YAML::Exception&) {
                                // skip malformed line
                            }
                        }
                        ROS_INFO("Loaded %zu pixel coordinates from %s",
                                 cached_pixels.size(), pixel_file.c_str());
                    }
                }
                const size_t pixel_idx = *manual_waypoint_count;
                if (pixel_idx < cached_pixels.size()) {
                    waypoint.px = cached_pixels[pixel_idx].first;
                    waypoint.py = cached_pixels[pixel_idx].second;
                }
            }
            waypoint.point_id = static_cast<int>(*manual_waypoint_count) + 1;
            airline->waypoint_list.push_back(waypoint);
            ++(*manual_waypoint_count);
        }

        return true;
    } catch (const YAML::Exception& e) {
        if (error_message != nullptr) {
            *error_message = e.what();
        }
        return false;
    } catch (const std::exception& e) {
        if (error_message != nullptr) {
            *error_message = e.what();
        }
        return false;
    }
}

void HttpClient::WaypointPollTimerCallback(const ros::TimerEvent& event) {
    (void)event;
    ScanAndSendWaypointAirlines();
}

void HttpClient::ScanAndSendWaypointAirlines() {
    struct stat dir_stat;
    if (stat(waypoint_yaml_dir_.c_str(), &dir_stat) != 0 || !S_ISDIR(dir_stat.st_mode)) {
        ROS_WARN_THROTTLE(30.0, "Waypoint YAML directory not found: %s",
                          waypoint_yaml_dir_.c_str());
        return;
    }

    DIR* dir = opendir(waypoint_yaml_dir_.c_str());
    if (dir == nullptr) {
        ROS_WARN_THROTTLE(30.0, "Failed to open waypoint YAML directory: %s",
                          waypoint_yaml_dir_.c_str());
        return;
    }

    while (const dirent* entry = readdir(dir)) {
        const std::string name(entry->d_name);
        if (name == "." || name == "..") {
            continue;
        }
        if (name.size() <= 5 || name.compare(name.size() - 5, 5, ".yaml") != 0) {
            continue;
        }
        if (entry->d_type == DT_DIR) {
            continue;
        }
        if (name == "config.yaml") {
            continue;
        }

        const std::string full_path = JoinPath(waypoint_yaml_dir_, name);

        struct stat file_stat;
        if (stat(full_path.c_str(), &file_stat) != 0 || !S_ISREG(file_stat.st_mode)) {
            continue;
        }
        const std::time_t current_mtime = file_stat.st_mtime;

        {
            std::lock_guard<std::mutex> lock(waypoint_poll_mutex_);
            auto it = waypoint_file_mtime_store_.find(full_path);
            if (it != waypoint_file_mtime_store_.end() && it->second == current_mtime) {
                continue;
            }
        }

        TrySendWaypointAirlineFromFile(full_path, current_mtime);
    }

    closedir(dir);
}

bool HttpClient::TrySendWaypointAirlineFromFile(const std::string& yaml_path,
                                                 std::time_t current_mtime) {
    Airline airline;
    size_t manual_waypoint_count = 0;
    std::string error_message;

    if (!BuildRecordedWaypointAirlineFromYaml(yaml_path,
                                              &airline,
                                              &manual_waypoint_count,
                                              &error_message)) {
        ROS_WARN("Failed to build airline from YAML [%s]: %s",
                 yaml_path.c_str(), error_message.c_str());
        {
            std::lock_guard<std::mutex> lock(waypoint_poll_mutex_);
            waypoint_file_mtime_store_[yaml_path] = current_mtime;
        }
        return false;
    }

    HttpResult result = SendAirline(airline);
    if (result.result_code != 1 && result.result_code != 5) {
        ROS_WARN("Auto sendAirline failed for [%s] airlineKey=%s, resultCode=%d",
                 yaml_path.c_str(), airline.airline_key.c_str(), result.result_code);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(waypoint_poll_mutex_);
        waypoint_file_mtime_store_[yaml_path] = current_mtime;

        const std::string mtime_file = JoinPath(waypoint_yaml_dir_, ".airline_mtime.json");
        try {
            json j;
            for (const auto& [p, t] : waypoint_file_mtime_store_) {
                j[p] = static_cast<int64_t>(t);
            }
            std::ofstream out(mtime_file);
            if (out.is_open()) {
                out << j.dump();
            }
        } catch (const std::exception& e) {
            ROS_WARN("Failed to save waypoint mtime file: %s", e.what());
        }
    }

    if (result.result_code == 1) {
        ROS_INFO("Auto sendAirline succeeded for [%s], airlineKey=%s, waypoint count=%zu",
                 yaml_path.c_str(), airline.airline_key.c_str(), manual_waypoint_count);
    } else {
        ROS_INFO("Auto sendAirline skipped for [%s] airlineKey=%s, already exists on server",
                 yaml_path.c_str(), airline.airline_key.c_str());
    }

    std::string map_name = yaml_path.substr(yaml_path.find_last_of('/') + 1);
    if (map_name.size() > 5 && map_name.compare(map_name.size() - 5, 5, ".yaml") == 0) {
        map_name = map_name.substr(0, map_name.size() - 5);
    }
    if (!map_name.empty()) {
        FtpUploadMapImage(map_name);
    }
    return true;
}

bool HttpClient::FtpUploadMapImage(const std::string& map_name) {
    if (ftp_server_ip_.empty()) {
        return true;
    }

    const std::string local_path = JoinPath(waypoint_map2d_dir_, map_name + ".png");
    struct stat st;
    if (stat(local_path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
        ROS_WARN("Map image not found for FTP: %s", local_path.c_str());
        return false;
    }

    std::string ftp_url = "ftp://" + ftp_server_ip_;
    if (ftp_server_port_ != 21) {
        ftp_url += ":" + std::to_string(ftp_server_port_);
    }
    ftp_url += ftp_remote_map_dir_ + map_name + ".png";

    std::string cmd = "curl -s --connect-timeout 5 --max-time 10 -T \"" + local_path + "\" \"" + ftp_url + "\"";
    if (!ftp_user_.empty()) {
        cmd += " --user \"" + ftp_user_ + ":" + ftp_password_ + "\"";
    }
    ROS_INFO("FTP upload: %s", cmd.c_str());

    int ret = std::system(cmd.c_str());
    if (ret != 0) {
        ROS_WARN("FTP upload failed (exit=%d): %s", ret, cmd.c_str());
        return false;
    }

    ROS_INFO("FTP upload succeeded: %s", ftp_url.c_str());
    return true;
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

    // == 着地后处理完成，重启系统 ==
    ROS_INFO("Post-land workflow complete, rebooting system...");
    std::system("sync");                                          // 刷写文件系统缓存
    sleep(2);                                                     // 等待日志落盘
    std::system("echo \"888888\" | sudo -S /sbin/reboot &");     // 后台重启（不阻塞当前线程）
}

bool HttpClient::HandleCacheAirlineMeta(indooruav_http::CacheAirlineMeta::Request& req,
                                    indooruav_http::CacheAirlineMeta::Response& res) {
    (void)req;
    ROS_WARN_ONCE("cache_airline_meta is deprecated; airline metadata is now auto-derived from YAML files");
    res.result_code = 1;
    return true;
}

bool HttpClient::HandleWaypointSaveProxy(std_srvs::Trigger::Request& req,
                                         std_srvs::Trigger::Response& res) {
    (void)req;
    std_srvs::Trigger raw_service;
    if (!waypoint_save_raw_client_.call(raw_service)) {
        res.success = false;
        res.message = "Failed to call raw waypoint save service: " + waypoint_save_raw_service_;
        ROS_WARN("%s", res.message.c_str());
        return true;
    }

    res.success = raw_service.response.success;
    res.message = raw_service.response.message;
    if (!raw_service.response.success) {
        ROS_WARN("Raw waypoint save service reported failure: %s", res.message.c_str());
        return true;
    }

    ROS_INFO("Waypoint save proxy succeeded; polling timer will detect and send updated YAML");
    return true;
}

bool HttpClient::HandleSendAirline(indooruav_http::SendAirline::Request& req,
                                   indooruav_http::SendAirline::Response& res) {
    Airline airline;
    airline.airline_key = req.airline_key;
    airline.airline_map = req.airline_map;
    airline.xscale = req.xscale;
    airline.yscale = req.yscale;
    airline.xzero = req.xzero;
    airline.yzero = req.yzero;
    airline.angle = req.angle;
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

bool HttpClient::HandleResendAllAirlines(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res) {
    (void)req;
    (void)res;
    {
        std::lock_guard<std::mutex> lock(waypoint_poll_mutex_);
        waypoint_file_mtime_store_.clear();
    }
    const std::string mtime_file = JoinPath(waypoint_yaml_dir_, ".airline_mtime.json");
    std::ofstream out(mtime_file, std::ios::trunc);
    out.close();
    ROS_INFO("Resend all airlines triggered: cleared mtime store and file [%s]",
             mtime_file.c_str());
    return true;
}

}  // namespace indooruav_http
