#include <ros/ros.h>
#include "http/http_server.h"

// HTTP服务端入口：启动本地HTTP监听并进入ROS循环
int main(int argc, char** argv) {
    ros::init(argc, argv, "indooruav_http_server");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    int local_port = 20000;
    nh_private.param<int>("local_port", local_port, 20000);

    indooruav_http::HttpServer server(nh, local_port);
    if (!server.Start()) {
        ROS_ERROR("Failed to start HTTP server on port %d", local_port);
        return 1;
    }

    ros::spin();
    return 0;
}
