#ifndef INDOORUAV_HTTP_SERVER_H
#define INDOORUAV_HTTP_SERVER_H

#include <atomic>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <yaml-cpp/yaml.h>

#include <http/httplib/httplib.h>

namespace indooruav_http {

class HttpServer {
public:
	HttpServer(ros::NodeHandle& nh, int port);
	~HttpServer();

	bool Start();
	void Stop();

private:
	void SetupPublishers(ros::NodeHandle& nh);
	void SetupRoutes();
	void SetupCommandClients(ros::NodeHandle& nh);

	void HandleAirlineInfo(const httplib::Request& req, httplib::Response& res);
	void HandleCommand(const httplib::Request& req, httplib::Response& res);

	void LaunchNodesForTakeoff();
	void DelayedTakeoffCommand(const ros::TimerEvent& event);

	bool CallCommandService(int command_mode);
	bool GetIntParam(const httplib::Request& req, const std::string& key, int* value) const;
	bool GetStringParam(const httplib::Request& req, const std::string& key, std::string* value) const;

	void SendResult(httplib::Response& res, int result_code) const;

private:
	int port_ = 0;
	std::unique_ptr<httplib::Server> server_;
	std::unique_ptr<std::thread> server_thread_;
	std::atomic<bool> is_running_{false};

	ros::NodeHandle nh_;

	ros::Publisher airline_info_pub_;
	ros::Publisher airline_key_pub_;
	std::string airline_info_topic_;
	std::string airline_key_topic_;
	std::string airline_key_param_;

	std::unordered_map<int, std::string> command_service_names_;
	std::unordered_map<int, ros::ServiceClient> command_clients_;
	double command_service_wait_timeout_sec_ = 1.0;

	ros::Timer takeoff_timer_;
	std::vector<pid_t> launched_pids_;
};

}  // namespace indooruav_http

#endif  // INDOORUAV_HTTP_SERVER_H