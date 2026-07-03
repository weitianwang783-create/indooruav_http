#include <ros/ros.h>
#include "http/http_client.h"

// HTTP客户端入口：启动定时上报并进入ROS循环
int main(int argc, char** argv) {
    ros::init(argc, argv, "indooruav_http_client");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    std::string server_ip;
    int server_port = 7000;
    int site_id = 11;
    int device_id = 1;
    double flight_state_interval = 3.0;
    int flight_state_sample_rate = 1;
    double takeoff_state_interval = 3.0;

    nh_private.param<std::string>("server_ip", server_ip, "127.0.0.1");
    nh_private.param<int>("server_port", server_port, 7000);
    nh_private.param<int>("site_id", site_id, 11);
    nh_private.param<int>("device_id", device_id, 1);
    nh_private.param<double>("flight_state_interval", flight_state_interval, 3.0);
    nh_private.param<int>("flight_state_sample_rate", flight_state_sample_rate, 1);
    nh_private.param<double>("takeoff_state_interval", takeoff_state_interval, 3.0);

    indooruav_http::HttpClient client(
        nh,
        server_ip,
        server_port,
        site_id,
        device_id,
        flight_state_interval,
        flight_state_sample_rate,
        takeoff_state_interval);
    client.StartTimers();

    ros::spin();
    return 0;
}
