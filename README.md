# indooruav_http 使用说明

## 1. 功能概述
`indooruav_http` 包含两个 ROS 节点：
- `indooruav_http_server`：对外提供本地 HTTP 接口，例如 `/airlineInfo` 和 `/sendCommand`，接收前端请求后转发到 ROS。
- `indooruav_http_client`：按照接口文档向前端服务器上报设备状态、飞行状态、报警、起飞许可、任务完成、图片上传等信息。

当前已支持：
- 通过 `/airlineInfo` 缓存 `airlineKey` 和 `detectTimeCur`
- 通过 `/sendCommand?commandMode=1` 转发起飞事件
- 通过 `/indooruav_http/send_pic` 上传单张图片
- 通过 `/indooruav_http/send_pic_over` 通知图片批次上传完成
- 在降落完成后自动执行 `1.9 sendFlyOver -> 1.10 sendPic -> 1.11 sendPicOver`

接口定义参考：`drone-interface.md`

## 2. 构建
依赖环境：
- ROS1 Noetic + catkin
- cpp-httplib
- nlohmann/json

构建整个工作空间：
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

只构建相关功能包：
```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg indooruav_http indooruav_core
source ~/catkin_ws/devel/setup.bash
```

## 3. 启动方式
### 3.1 一键启动
```bash
roslaunch indooruav_http bringup_indooruav_http.launch
```

可选参数：
```bash
roslaunch indooruav_http bringup_indooruav_http.launch start_core:=false
roslaunch indooruav_http bringup_indooruav_http.launch \
  server_config:=/path/to/http_server.yaml \
  client_config:=/path/to/http_client.yaml
```

### 3.2 单独启动 HTTP 服务端
```bash
rosrun indooruav_http indooruav_http_server _local_port:=20000
```

主要 HTTP 接口：
- `/airlineInfo`
- `/sendCommand`

说明：
- `commandMode=1` 默认映射到 `indooruav_core/state_machine_event/takeoff_command`
- `/sendCommand?commandMode=1` 会调用一个 `std_srvs/Empty` 类型的 ROS service
- 当前链路为：前端 `/sendCommand` -> `indooruav_http_server` -> `indooruav_core/state_machine_event/takeoff_command` -> `indooruav_core`

### 3.3 单独启动 HTTP 客户端
```bash
rosrun indooruav_http indooruav_http_client \
  _server_ip:=127.0.0.1 \
  _server_port:=8080 \
  _site_id:=11 \
  _device_id:=1 \
  _device_state_interval:=30 \
  _flight_state_interval:=3 \
  _flight_state_sample_rate:=1 \
  _takeoff_state_interval:=3
```

重要参数：
- `server_ip` / `server_port`：前端服务器地址
- `site_id` / `device_id`：站点编号和设备编号
- `device_state_interval`：设备状态上报周期
- `flight_state_interval`：飞行状态上报周期
- `flight_state_sample_rate`：飞行状态采样频率
- `takeoff_state_interval`：起飞许可上报周期
- `uav_online_timeout_sec`：超过多久未收到真实遥测后，将 `uavState` 判为离线
- `odom_topic` / `odom_fallback_topic`：位姿来源
- `post_land_image_root_dir`：降落后自动上传图片的根目录，实际扫描路径为 `<post_land_image_root_dir>/<detectTimeCur>/`

可用的 ROS service：
- `/indooruav_http/send_airline`
- `/indooruav_http/send_pic`
- `/indooruav_http/send_pic_over`
- `/indooruav_http/send_fly_over`
- `/indooruav_http/send_error_data`
- `/indooruav_http/set_takeoff_state`
- `/indooruav_http/airline_sync`
- `/indooruav_http/run_post_land_workflow`

说明：
- `uavState` 会根据最近是否收到 `battery/odom/gimbal` 遥测自动切换在线/离线
- `send_pic` 在当前 `detectTimeCur` 批次内按 `1、2、3...` 的顺序命名
- 降落完成后，`indooruav_core` 会自动触发 `/indooruav_http/run_post_land_workflow`
- 自动工作流固定执行：`1.9 sendFlyOver -> 扫描图片目录 -> 多次 1.10 sendPic -> 1.11 sendPicOver`
- 支持的图片扩展名：`.jpg`、`.jpeg`、`.png`、`.bmp`、`.webp`

### 3.4 启动状态机
如果你希望 `/sendCommand?commandMode=1` 返回成功，需要启动状态机：
```bash
rosrun indooruav_core indooruav_core_node
```

## 4. 本地测试
下面所有命令默认都在 WSL 终端执行。

### 4.1 前端调用本地 HTTP 服务端
启动整套服务：
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch indooruav_http bringup_indooruav_http.launch
```

示例请求：
```text
http://127.0.0.1:20000/airlineInfo?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959
http://127.0.0.1:20000/sendCommand?siteId=11&deviceId=1&commandMode=1
```

### 4.2 本地假前端测试图片上传
这套测试适合验证：
- `1.10 sendPic`
- `1.11 sendPicOver`
- 降落完成后的自动回传流程

终端 1：启动本地假前端，监听 `8080`
```bash
mkdir -p /tmp/fake_frontend_imgs

python3 - <<'PY'
from http.server import BaseHTTPRequestHandler, HTTPServer
import cgi
import json
import os
import urllib.parse

SAVE_ROOT = "/tmp/fake_frontend_imgs"
os.makedirs(SAVE_ROOT, exist_ok=True)

class Handler(BaseHTTPRequestHandler):
    def reply(self, code=200, obj=None):
        if obj is None:
            obj = {"resultCode": 1}
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        qs = urllib.parse.parse_qs(parsed.query)

        if parsed.path == "/sendPic":
            form = cgi.FieldStorage(
                fp=self.rfile,
                headers=self.headers,
                environ={
                    "REQUEST_METHOD": "POST",
                    "CONTENT_TYPE": self.headers.get("Content-Type", ""),
                    "CONTENT_LENGTH": self.headers.get("Content-Length", "0"),
                },
            )
            if "file" not in form:
                return self.reply(400, {"resultCode": 3})

            file_item = form["file"]
            device_id = qs.get("deviceId", ["unknown"])[0]
            detect_time = qs.get("detectTimeCur", ["unknown"])[0]
            year, month, day = detect_time[0:4], detect_time[4:6], detect_time[6:8]
            out_dir = os.path.join(SAVE_ROOT, device_id, year, month, day, detect_time)
            os.makedirs(out_dir, exist_ok=True)

            out_path = os.path.join(out_dir, os.path.basename(file_item.filename))
            with open(out_path, "wb") as f:
                f.write(file_item.file.read())

            print(f"[sendPic] saved: {out_path}", flush=True)
            return self.reply(200, {"resultCode": 1})

        length = int(self.headers.get("Content-Length", "0"))
        if length > 0:
            self.rfile.read(length)

        return self.reply(200, {"resultCode": 1})

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        print(f"[GET] {parsed.path}?{parsed.query}", flush=True)
        if parsed.path == "/airlineSync":
            return self.reply(200, {"resultCode": 1, "result": []})
        return self.reply(200, {"resultCode": 1})

    def log_message(self, fmt, *args):
        pass

print("fake frontend server listening on 0.0.0.0:8080", flush=True)
HTTPServer(("0.0.0.0", 8080), Handler).serve_forever()
PY
```

终端 2：准备测试图片
```bash
mkdir -p /tmp/indooruav_post_land_images/20250701125959
cp /home/wwt/catkin_ws/src/indooruav_core/scripts/image.png /tmp/indooruav_post_land_images/20250701125959/001.png
cp /home/wwt/catkin_ws/src/indooruav_core/scripts/image.png /tmp/indooruav_post_land_images/20250701125959/002.png
find /tmp/indooruav_post_land_images -type f | sort
```

终端 3：启动 `core + http`
```bash
cd ~/catkin_ws
catkin_make --pkg indooruav_core indooruav_http
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch indooruav_http bringup_indooruav_http.launch
```

建议确认客户端配置：
```yaml
server_ip: "127.0.0.1"
server_port: 8080
post_land_image_root_dir: "/tmp/indooruav_post_land_images"
```

### 4.3 测试 A：只验证 http 工作流
终端 4：
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

curl "http://127.0.0.1:20000/airlineInfo?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959"
rosservice call /indooruav_http/run_post_land_workflow
find /tmp/fake_frontend_imgs -type f | sort
```

成功时，假前端终端会看到：
```text
[GET] /sendFlyOver?airlineKey=AAAA&detectTimeCur=20250701125959&deviceId=1&siteId=11
[sendPic] saved: /tmp/fake_frontend_imgs/1/2025/07/01/20250701125959/1.png
[sendPic] saved: /tmp/fake_frontend_imgs/1/2025/07/01/20250701125959/2.png
[GET] /sendPicOver?airlineKey=AAAA&detectTimeCur=20250701125959&deviceId=1&siteId=11
```

本地落盘结果应为：
```bash
/tmp/fake_frontend_imgs/1/2025/07/01/20250701125959/1.png
/tmp/fake_frontend_imgs/1/2025/07/01/20250701125959/2.png
```

### 4.4 测试 B：验证 `core -> http` 自动触发
重要前提：发送 `/indooruav_core/state_machine_event/land_complete` 时，状态机当前必须已经在 `Land` 状态。

终端 4：
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

curl "http://127.0.0.1:20000/airlineInfo?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959"

rosservice call /indooruav_core/state_machine_event/takeoff_command
rosservice call /indooruav_core/state_machine_event/check_passed
rosservice call /indooruav_core/state_machine_event/takeoff_complete
rosservice call /indooruav_core/state_machine_event/cruise_complete
rosservice call /indooruav_core/state_machine_event/land_complete

find /tmp/fake_frontend_imgs -type f | sort
```

状态机成功推进时，`indooruav_core` 终端应出现类似：
```text
[Event] TakeoffCommand, [Current State] Await
[Event] CheckPassed, [Current State] CheckBeforeTakeOff
[Event] TakeoffComplete, [Current State] TakeOff
[Event] CruiseComplete, [Current State] Cruise
[State] Land
[Event] LandComplete, [Current State] Land
[State] Charge
```

假前端终端会再次看到：
```text
[GET] /sendFlyOver?airlineKey=AAAA&detectTimeCur=20250701125959&deviceId=1&siteId=11
[sendPic] saved: /tmp/fake_frontend_imgs/1/2025/07/01/20250701125959/1.png
[sendPic] saved: /tmp/fake_frontend_imgs/1/2025/07/01/20250701125959/2.png
[GET] /sendPicOver?airlineKey=AAAA&detectTimeCur=20250701125959&deviceId=1&siteId=11
```

## 5. 结果码说明
- `resultCode=1`：成功
- `resultCode=2`：失败
- `resultCode=3`：参数错误或数据格式错误
- `resultCode=5`：`airlineKey` 重复，仅 `1.1` 接口使用

## 6. 常见问题
- `/sendCommand` 返回 2：确认状态机已启动，或检查 `command_mode_1_service` 配置。
- 飞行状态不上传：需要 `odom_topic` 有数据，否则客户端会提示样本不足。
- `land_complete` 没触发自动上传：大概率是发送时状态机还不在 `Land` 状态。
- `/indooruav_http/send_pic` 返回 3：先确认已经通过 `/airlineInfo` 下发了 `airlineKey` 和 `detectTimeCur`。
- `find /tmp/fake_frontend_imgs -type f` 没有结果：先看假前端终端是否收到 `/sendPic`，再确认 `http_client.yaml` 是否指向 `127.0.0.1:8080`。
- WSL 提示 `localhost 代理未镜像`：这条提示通常可以忽略，不影响这里基于 `127.0.0.1` 的本地测试。
