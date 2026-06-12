#ifndef INDOORUAV_HTTP_CLIENT_H
#define INDOORUAV_HTTP_CLIENT_H

#include <deque>
#include <map>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <http/httplib/httplib.h>
#include "http/data_types.h"

#include "indooruav_http/AirlineSync.h"
#include "indooruav_http/SendAirline.h"
#include "indooruav_http/SendErrorData.h"
#include "indooruav_http/SendFlyOver.h"
#include "indooruav_http/SendPic.h"
#include "indooruav_http/SendPicOver.h"
#include "indooruav_http/TakeoffState.h"
#include "indooruav_msgs/TransferMissionMedia.h"
#include "indooruav_msgs/UploadImageBytes.h"

namespace indooruav_http {

class HttpClient {
public:
	HttpClient(ros::NodeHandle& nh,
			   const std::string& server_ip,
			   int server_port,
			   int site_id,
			   int device_id,
			   double device_interval,
			   double flight_interval,
			   int sample_rate,
			   double takeoff_interval);
	~HttpClient();

	void StartTimers();

private:
	void SetupSubscribers(ros::NodeHandle& nh);
	void SetupServices(ros::NodeHandle& nh);
	void MarkTelemetryReceived();
	void UpdateDerivedDeviceState();
	bool HasFreshDeviceStateInfo() const;

	void BatteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
	void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void GimbalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void DetectionCallback(const std_msgs::String::ConstPtr& msg);
	void AirlineInfoCallback(const std_msgs::String::ConstPtr& msg);
	void AirlineKeyCallback(const std_msgs::String::ConstPtr& msg);
	void DeviceStateInfoCallback(const std_msgs::String::ConstPtr& msg);
	void TakeoffStateTopicCallback(const std_msgs::Int32::ConstPtr& msg);

	void DeviceStateTimerCallback(const ros::TimerEvent& event);
	void FlightStateTimerCallback(const ros::TimerEvent& event);
	void TakeoffStateTimerCallback(const ros::TimerEvent& event);

	HttpResult SendAirline(const Airline& airline);
	HttpResult SendPic(const std::string& image_path);
	HttpResult SendDeviceState(const DeviceState& device_state);
	HttpResult SendFlightStates(const std::vector<FlightState>& flight_states);
	HttpResult SendErrorData(const ErrorData& error_data);
	HttpResult SendTakeoffState(int takeoff_state);
	HttpResult SendFlyOver();
	HttpResult SendPicOver();
	HttpResult AirlineSync();
	HttpResult SendPicBytesWithMission(int site_id,
									   int device_id,
									   const std::string& image_extension,
									   const std::vector<uint8_t>& image_bytes,
									   const std::string& airline_key,
									   const std::string& detect_time_cur);
	HttpResult SendPicWithMission(int site_id,
								  int device_id,
								  const std::string& image_path,
								  const std::string& airline_key,
								  const std::string& detect_time_cur);
	HttpResult SendFlyOverWithMission(int site_id,
									  int device_id,
									  const std::string& airline_key,
									  const std::string& detect_time_cur);
	HttpResult SendPicOverWithMission(int site_id,
									  int device_id,
									  const std::string& airline_key,
									  const std::string& detect_time_cur);

	HttpResult ParseResult(const httplib::Result& result);
	std::string GetTimeStamp() const;
	bool ReadBinaryFile(const std::string& image_path, std::string* file_bytes) const;
	std::string GetImageExtension(const std::string& image_path) const;
	std::string NormalizeImageExtension(const std::string& image_extension) const;
	std::string GetImageMimeTypeByExtension(const std::string& image_extension) const;
	std::string GetImageMimeType(const std::string& image_path) const;
	bool IsSupportedImageExtension(const std::string& extension) const;
	void GetCurrentTargetIds(int* site_id, int* device_id);
	bool GetAirlineInfoTargetIds(int* site_id, int* device_id);
	bool GetCurrentMissionContext(int* site_id,
								  int* device_id,
								  std::string* airline_key,
								  std::string* detect_time_cur);
	bool ResolveTargetIdsForMission(const std::string& airline_key,
									const std::string& detect_time_cur,
									int* site_id,
									int* device_id);
	std::vector<std::string> CollectPostLandImages(const std::string& detect_time_cur) const;
	indooruav_msgs::TransferMissionMedia::Response TransferMissionMediaFromController(
		const std::string& airline_key,
		const std::string& detect_time_cur);
	void StartPostLandWorkflow();
	void RunPostLandWorkflow();

	bool HandleSendAirline(indooruav_http::SendAirline::Request& req,
						   indooruav_http::SendAirline::Response& res);
	bool HandleSendPic(indooruav_http::SendPic::Request& req,
					   indooruav_http::SendPic::Response& res);
	bool HandleUploadImageBytes(indooruav_msgs::UploadImageBytes::Request& req,
								indooruav_msgs::UploadImageBytes::Response& res);
	bool HandleSetTakeoffState(indooruav_http::TakeoffState::Request& req,
							   indooruav_http::TakeoffState::Response& res);
	bool HandleSendErrorData(indooruav_http::SendErrorData::Request& req,
							 indooruav_http::SendErrorData::Response& res);
	bool HandleSendFlyOver(indooruav_http::SendFlyOver::Request& req,
						 indooruav_http::SendFlyOver::Response& res);
	bool HandleSendPicOver(indooruav_http::SendPicOver::Request& req,
						 indooruav_http::SendPicOver::Response& res);
	bool HandleAirlineSync(indooruav_http::AirlineSync::Request& req,
						   indooruav_http::AirlineSync::Response& res);
	bool HandleRunPostLandWorkflow(std_srvs::Empty::Request& req,
								   std_srvs::Empty::Response& res);

private:
	ros::NodeHandle& nh_;

	std::string server_ip_;
	int server_port_ = 0;
	int site_id_ = 0;
	int device_id_ = 0;
	double uav_online_timeout_sec_ = 5.0;

	double device_state_interval_ = 30.0;
	double flight_state_interval_ = 3.0;
	int flight_state_sample_rate_ = 1;
	double takeoff_state_interval_ = 3.0;
	std::string post_land_image_root_dir_;
	std::string post_land_image_source_mode_;
	std::string controller_upload_mission_media_service_;

	std::unique_ptr<httplib::Client> client_;
	std::mutex http_mutex_;
	std::atomic<bool> post_land_workflow_running_{false};
	std::thread post_land_workflow_thread_;

	DeviceState current_device_state_;
	std::deque<FlightState> flight_state_buffer_;
	std::mutex buffer_mutex_;
	ros::Time last_telemetry_time_;

	ros::Time last_sample_time_;
	ros::Time last_device_state_info_time_;
	double gimbal_roll_ = 0.0;
	double gimbal_pitch_ = 0.0;
	double gimbal_yaw_ = 0.0;
	int pantograph_is_ = 0;
	int abnormal_is_ = 0;
	double pantograph_locx_ = 0.0;
	double pantograph_locy_ = 0.0;
	double pantograph_locz_ = 0.0;
	double abnormal_locx_ = 0.0;
	double abnormal_locy_ = 0.0;
	double abnormal_locz_ = 0.0;
	bool enable_detection_error_ = true;
	bool has_device_state_info_ = false;

	int airline_info_site_id_ = 0;
	int airline_info_device_id_ = 0;
	std::string airline_info_airline_key_;
	std::string airline_info_detect_time_cur_;
	bool has_airline_info_context_ = false;
	std::string airline_key_;
	std::string detect_time_cur_;
	std::mutex airline_mutex_;

	int takeoff_state_ = 1;
	std::mutex takeoff_mutex_;

	ros::Subscriber battery_sub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber odom_fallback_sub_;
	ros::Subscriber gimbal_sub_;
	ros::Subscriber detection_sub_;
	ros::Subscriber airline_info_sub_;
	ros::Subscriber airline_key_sub_;
	ros::Subscriber device_state_info_sub_;
	ros::Subscriber takeoff_state_sub_;

	ros::Timer device_state_timer_;
	ros::Timer flight_state_timer_;
	ros::Timer takeoff_state_timer_;

	ros::ServiceServer send_airline_service_;
	ros::ServiceServer send_pic_service_;
	ros::ServiceServer upload_image_bytes_service_;
	ros::ServiceServer set_takeoff_state_service_;
	ros::ServiceServer send_error_data_service_;
	ros::ServiceServer send_fly_over_service_;
	ros::ServiceServer send_pic_over_service_;
	ros::ServiceServer airline_sync_service_;
	ros::ServiceServer run_post_land_workflow_service_;
	ros::ServiceClient transfer_mission_media_client_;

	std::string airline_key_topic_;
	std::string airline_info_topic_;
	std::string device_state_info_topic_;
	std::string takeoff_state_topic_;
	std::string battery_topic_;
	std::string odom_topic_;
	std::string odom_fallback_topic_;
	std::string gimbal_topic_;
	std::string detection_topic_;
};

}  // namespace indooruav_http

#endif  // INDOORUAV_HTTP_CLIENT_H
