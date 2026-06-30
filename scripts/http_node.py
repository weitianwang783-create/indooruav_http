#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import json
import time
import requests
from flask import Flask, request, jsonify
from indooruav_http.srv import SendAirline, SendAirlineResponse
from indooruav_http.srv import TakeoffState, TakeoffStateResponse

app = Flask(__name__)

# 配置信息
server_url = "http://127.0.0.1:8080"  # 前端/远端服务器地址
site_id = 11
device_id = 1
airline_key_current = "AAAA"

# ================= HTTP Server (前端发往无人机) =================

@app.route('/sendAirline', methods=['GET', 'POST'])
def send_airline():
    """
    1.1、接收前端的新建航线信息
    例如：http://ip:port/sendAirline?siteId=11&deviceId=1&file=file.json
    """
    try:
        req_site_id = request.args.get('siteId', 11, type=int)
        req_device_id = request.args.get('deviceId', 1, type=int)
        
        # 假设json内容放在body中（或者作为文件上传）
        route_data = request.json
        if not route_data and 'file' in request.files:
            file_storage = request.files['file']
            route_data = json.load(file_storage)
        elif not route_data and request.data:
            route_data = json.loads(request.data)

        if not route_data:
            return jsonify({"resultCode": 3}) # 数据格式不正确

        airline_key = route_data.get('airlineKey', '')
        airline_map = route_data.get('airlineMap', '')
        xscale = route_data.get('xscale', 0.0)
        yscale = route_data.get('yscale', 0.0)
        xzero = route_data.get('xzero', 0.0)
        yzero = route_data.get('yzero', 0.0)
        angle = route_data.get('angle', 0.0)
        waypoint_list = route_data.get('waypointList', [])

        # 调用ROS服务通知系统其他节点航线信息
        rospy.wait_for_service('/indooruav/send_airline', timeout=2.0)
        airline_srv = rospy.ServiceProxy('/indooruav/send_airline', SendAirline)

        resp = airline_srv(airline_key, airline_map, xscale, yscale, xzero, yzero, angle, json.dumps(waypoint_list))
        
        return jsonify({"resultCode": resp.result_code})

    except Exception as e:
        rospy.logerr(f"Error in sendAirline: {e}")
        return jsonify({"resultCode": 2})

@app.route('/sendTakeoffState', methods=['GET', 'POST'])
def send_takeoff_state():
    """
    1.6、接收起飞许可
    http://ip:port/sendTakeoffState?siteId=11&deviceId=1&takeoffState=1
    """
    try:
        takeoff_state = request.args.get('takeoffState', 1, type=int)
        
        rospy.wait_for_service('/indooruav/takeoff_state', timeout=2.0)
        takeoff_srv = rospy.ServiceProxy('/indooruav/takeoff_state', TakeoffState)
        
        resp = takeoff_srv(takeoff_state)
        return jsonify({"resultCode": resp.result_code})

    except Exception as e:
        rospy.logerr(f"Error in sendTakeoffState: {e}")
        return jsonify({"resultCode": 2})

def run_flask(port):
    app.run(host='0.0.0.0', port=port, threaded=True)


# ================= HTTP Client (无人机发往前端) =================

def send_device_data_task():
    """ 1.2、发送设备状态信息 (30秒) """
    rate = rospy.Rate(1.0 / 30.0)
    while not rospy.is_shutdown():
        try:
            # TODO: 替换为订阅到的真实数据
            payload = {
                "uavState": 1,
                "controlState": 1,
                "controlSoc": 90.1,
                "controlRssi": 99.9,
                "batteryTemp": 50.5,
                "batterySoc": 90.9,
                "batteryRssi": 99.9,
                "batteryVolt": 40.1,
                "batteryCycleNum": 1
            }

            url = f"{server_url}/sendDeviceData?siteId={site_id}&deviceId={device_id}"
            
            # 使用 files 参数模拟上传 json 文件或者直接 post json
            files = {'file': ('file.json', json.dumps(payload), 'application/json')}
            resp = requests.post(url, files=files, timeout=5)
            # rospy.loginfo(f"send_device_data resp: {resp.text}")

        except Exception as e:
            rospy.logwarn(f"Failed to send device data: {e}")
            
        rate.sleep()

def send_fly_data_task():
    """ 1.3、发送无人机位置信息 (3秒) """
    rate = rospy.Rate(1.0 / 3.0)
    while not rospy.is_shutdown():
        try:
            detect_time_cur = time.strftime('%Y%m%d%H%M%S', time.localtime())
            
            # TODO: 替换为真实的飞行点迹列表 (每秒采集一次，满3个打成数组)
            payload = [{
                "timeStamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
                "positionx": 11.1, "positiony": 22.2, "positionz": 33.3,
                "px": 0.0, "py": 0.0,
                "attitudeRoll": 11.4, "attitudePitch": 22.5, "attitudeYaw": 33.6,
                "horizontalSpeed": 20.7, "verticalSpeed": 1.8, "lineType": 1,
                "poseAngleRoll": 11.9, "poseAnglePitch": 22.1, "poseAngleYaw": 33.2,
                "pantographIs": 0, "abnormalIs": 0,
                "pantographLocx": 0.0, "pantographLocy": 0.0, "pantographLocz": 0.0,
                "abnormalLocx": 0.0, "abnormalLocy": 0.0, "abnormalLocz": 0.0
            }]

            url = f"{server_url}/sendFlyData?siteId={site_id}&deviceId={device_id}&airlineKey={airline_key_current}&detectTimeCur={detect_time_cur}"
            
            files = {'file': ('file.json', json.dumps(payload), 'application/json')}
            resp = requests.post(url, files=files, timeout=5)
            # rospy.loginfo(f"send_fly_data resp: {resp.text}")

        except Exception as e:
            rospy.logwarn(f"Failed to send fly data: {e}")

        rate.sleep()

# 对于报警 1.4 sendErrorData 可以留出一个接口给其他 ROS Node 触发
def handle_error_trigger(error_type, error_info):
    try:
        detect_time_cur = time.strftime('%Y%m%d%H%M%S', time.localtime())
        payload = {
            "timeStamp": time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()),
            "errorType": error_type,
            "errorInfo": error_info
        }
        url = f"{server_url}/sendErrorData?siteId={site_id}&deviceId={device_id}&airlineKey={airline_key_current}&detectTimeCur={detect_time_cur}"
        files = {'file': ('file.json', json.dumps(payload), 'application/json')}
        requests.post(url, files=files, timeout=3)
    except Exception as e:
        rospy.logwarn(f"Failed to send error data: {e}")

if __name__ == '__main__':
    rospy.init_node('http_node', anonymous=True)
    
    # 获取参数
    local_port = rospy.get_param('~local_port', 20000)
    server_url = rospy.get_param('~server_url', 'http://127.0.0.1:8080')
    site_id = rospy.get_param('~site_id', 11)
    device_id = rospy.get_param('~device_id', 1)

    # 1. 启动 HTTP Server (Flask) 在子线程
    t_server = threading.Thread(target=run_flask, args=(local_port,))
    t_server.daemon = True
    t_server.start()
    rospy.loginfo(f"HTTP Server started on port {local_port}")

    # 2. 启动 HTTP Client 循环上报任务 (在子线程，以免阻塞 ROS spin)
    t_device = threading.Thread(target=send_device_data_task)
    t_device.daemon = True
    t_device.start()

    t_fly = threading.Thread(target=send_fly_data_task)
    t_fly.daemon = True
    t_fly.start()
    
    rospy.loginfo("HTTP Node running...")
    rospy.spin()
