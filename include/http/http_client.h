#ifndef INDOORUAV_HTTP_CLIENT_H
#define INDOORUAV_HTTP_CLIENT_H

#include <atomic>
#include <cstdint>
#include <ctime>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <http/httplib/httplib.h>
#include "http/data_types.h"

#include "indooruav_http/AirlineSync.h"
#include "indooruav_http/CacheAirlineMeta.h"
#include "indooruav_http/SendAirline.h"
#include "indooruav_http/SendErrorData.h"
#include "indooruav_http/SendFlyOver.h"
#include "indooruav_http/SendPic.h"
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
			   double flight_interval,
			   int sample_rate,
			   double takeoff_interval);
	~HttpClient();

	void StartTimers();

private:
	void SetupSubscribers(ros::NodeHandle& nh);
	void SetupServices(ros::NodeHandle& nh);
	void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void OdomPxCallback(const geometry_msgs::Point::ConstPtr& msg);
	void OdomWaypointsIdCallback(const std_msgs::UInt32::ConstPtr& msg);
	void GimbalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void DetectionCallback(const std_msgs::String::ConstPtr& msg);
	void AirlineInfoCallback(const std_msgs::String::ConstPtr& msg);
	void AirlineKeyCallback(const std_msgs::String::ConstPtr& msg);
	void TakeoffStateTopicCallback(const std_msgs::Int32::ConstPtr& msg);

	void FlightStateTimerCallback(const ros::TimerEvent& event);
	void TakeoffStateTimerCallback(const ros::TimerEvent& event);
	void WaypointPollTimerCallback(const ros::TimerEvent& event);

	HttpResult SendAirline(const Airline& airline);
	HttpResult SendPic(const std::string& image_path);
	HttpResult SendFlightStates(const std::vector<FlightState>& flight_states);
	HttpResult SendErrorData(const ErrorData& error_data);
	HttpResult SendTakeoffState(int takeoff_state);
	HttpResult SendFlyOver();
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
	void ForwardAirlineInfoToRemoteController(int site_id,
											 int device_id,
											 const std::string& airline_key,
											 const std::string& detect_time_cur);
	std::vector<std::string> CollectPostLandImages(const std::string& detect_time_cur) const;
	indooruav_msgs::TransferMissionMedia::Response TransferMissionMediaFromController(
		const std::string& airline_key,
		const std::string& detect_time_cur);
	void FillWaypointPxPy(Waypoint& waypoint) const;
	bool LoadWaypointPixelFile(const std::string& pixel_path,
							   std::vector<std::pair<double, double>>* pixels) const;
	bool BuildRecordedWaypointAirlineFromYaml(const std::string& yaml_path,
							  const std::string& pixel_path,
							  Airline* airline,
							  size_t* manual_waypoint_count,
							  std::string* error_message) const;
	void ScanAndSendWaypointAirlines();
	bool TrySendWaypointAirlineFromFile(const std::string& yaml_path,
										const std::string& pixel_path,
										std::time_t current_mtime);
	bool FtpUploadMapImage(const std::string& map_name);
	void StartPostLandWorkflow();
	void RunPostLandWorkflow();

	bool HandleCacheAirlineMeta(indooruav_http::CacheAirlineMeta::Request& req,
								indooruav_http::CacheAirlineMeta::Response& res);
	bool HandleSendAirline(indooruav_http::SendAirline::Request& req,
						   indooruav_http::SendAirline::Response& res);
	bool HandleSendPic(indooruav_http::SendPic::Request& req,
					   indooruav_http::SendPic::Response& res);
	bool HandleWaypointSaveProxy(std_srvs::Trigger::Request& req,
					  std_srvs::Trigger::Response& res);
	bool HandleUploadImageBytes(indooruav_msgs::UploadImageBytes::Request& req,
								indooruav_msgs::UploadImageBytes::Response& res);
	bool HandleSetTakeoffState(indooruav_http::TakeoffState::Request& req,
							   indooruav_http::TakeoffState::Response& res);
	bool HandleSendErrorData(indooruav_http::SendErrorData::Request& req,
							 indooruav_http::SendErrorData::Response& res);
	bool HandleSendFlyOver(indooruav_http::SendFlyOver::Request& req,
						 indooruav_http::SendFlyOver::Response& res);
	bool HandleAirlineSync(indooruav_http::AirlineSync::Request& req,
						   indooruav_http::AirlineSync::Response& res);
	bool HandleRunPostLandWorkflow(std_srvs::Empty::Request& req,
								   std_srvs::Empty::Response& res);
	bool HandleResendAllAirlines(std_srvs::Empty::Request& req,
								 std_srvs::Empty::Response& res);

private:
	ros::NodeHandle& nh_;

	std::string server_ip_;
	int server_port_ = 0;
	int site_id_ = 0;
	int device_id_ = 0;
	double flight_state_interval_ = 3.0;
	int flight_state_sample_rate_ = 1;
	double takeoff_state_interval_ = 3.0;
	std::string post_land_image_root_dir_;
	std::string post_land_image_source_mode_;
	std::string remote_controller_ip_;
	int remote_controller_port_ = 20000;
	std::string controller_upload_mission_media_service_;
	std::string waypoint_save_raw_service_;

	std::string waypoint_yaml_dir_;
	std::string waypoint_pixel_dir_;
	double waypoint_poll_interval_sec_ = 5.0;
	double default_waypoint_xscale_ = 15.8;
	double default_waypoint_yscale_ = 15.8;
	double default_waypoint_xzero_ = 0.0;
	double default_waypoint_yzero_ = 0.0;
	std::string waypoint_map2d_dir_;
	std::string ftp_server_ip_;
	int ftp_server_port_ = 21;
	std::string ftp_user_;
	std::string ftp_password_;
	std::string ftp_remote_map_dir_;
	std::map<std::string, std::time_t> waypoint_file_mtime_store_;
	std::mutex waypoint_poll_mutex_;
	ros::Timer waypoint_poll_timer_;

	std::unique_ptr<httplib::Client> client_;
	std::mutex http_mutex_;
	std::unique_ptr<httplib::Client> remote_controller_client_;
	std::mutex remote_controller_http_mutex_;
	std::atomic<bool> post_land_workflow_running_{false};
	std::thread post_land_workflow_thread_;

	std::deque<FlightState> flight_state_buffer_;
	mutable std::mutex buffer_mutex_;

	ros::Time last_sample_time_;
	double gimbal_roll_ = 0.0;
	double gimbal_pitch_ = 0.0;
	double gimbal_yaw_ = 0.0;
	double odom_px_ = 0.0;
	double odom_py_ = 0.0;
	int latest_waypoint_id_ = 0;
	int pantograph_is_ = 0;
	int abnormal_is_ = 0;
	double pantograph_locx_ = 0.0;
	double pantograph_locy_ = 0.0;
	double pantograph_locz_ = 0.0;
	double abnormal_locx_ = 0.0;
	double abnormal_locy_ = 0.0;
	double abnormal_locz_ = 0.0;
	bool enable_detection_error_ = true;

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

	ros::Subscriber odom_sub_;
	ros::Subscriber odom_px_sub_;
	ros::Subscriber odom_waypoints_id_sub_;
	ros::Subscriber odom_fallback_sub_;
	ros::Subscriber gimbal_sub_;
	ros::Subscriber detection_sub_;
	ros::Subscriber airline_info_sub_;
	ros::Subscriber airline_key_sub_;
	ros::Subscriber takeoff_state_sub_;

	ros::Timer flight_state_timer_;
	ros::Timer takeoff_state_timer_;

	ros::ServiceServer cache_airline_meta_service_;
	ros::ServiceServer send_airline_service_;
	ros::ServiceServer send_pic_service_;
	ros::ServiceServer waypoint_save_proxy_service_;
	ros::ServiceServer upload_image_bytes_service_;
	ros::ServiceServer set_takeoff_state_service_;
	ros::ServiceServer send_error_data_service_;
	ros::ServiceServer send_fly_over_service_;
	ros::ServiceServer airline_sync_service_;
	ros::ServiceServer run_post_land_workflow_service_;
	ros::ServiceServer resend_all_airlines_service_;
	ros::ServiceClient waypoint_save_raw_client_;
	ros::ServiceClient transfer_mission_media_client_;

	std::string airline_key_topic_;
	std::string airline_info_topic_;
	std::string takeoff_state_topic_;
	std::string odom_topic_;
	std::string odom_px_topic_;
	std::string odom_waypoints_id_topic_;
	std::string odom_fallback_topic_;
	std::string gimbal_topic_;
	std::string detection_topic_;
};

}  // namespace indooruav_http

#endif  // INDOORUAV_HTTP_CLIENT_H
