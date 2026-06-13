# indooruav_http 使用说明


雷达静态IP需要手动开启脚本，命令为
cd ~/Project/IndoorUavInspection2/catkin_ws/src/indooruav_http
sudo bash scripts/configure_radar_static_ip.sh
后期要设成一个systemd服务

### 雷达静态 IP systemd 服务

如果需要让雷达口 `eth0` 在开机时自动配置为静态 IP，可以使用仓库内提供的模板：

- service 模板：`config/radar-static-ip.service`
- 环境文件模板：`config/radar-static-ip.env`
- 执行脚本：`scripts/configure_radar_static_ip.sh`

推荐部署路径：

```bash
sudo mkdir -p /opt/indooruav/scripts /etc/indooruav
sudo cp scripts/configure_radar_static_ip.sh /opt/indooruav/scripts/
sudo chmod +x /opt/indooruav/scripts/configure_radar_static_ip.sh
sudo cp config/radar-static-ip.env /etc/indooruav/radar-static-ip.env
sudo cp config/radar-static-ip.service /etc/systemd/system/radar-static-ip.service
```

按现场情况修改环境文件：

```bash
sudoedit /etc/indooruav/radar-static-ip.env
```

默认值为：

- `ETH_IFACE=eth0`
- `RADAR_HOST_IP=192.168.10.50`
- `RADAR_PREFIX_LEN=24`
- `RADAR_DEVICE_IP=192.168.10.3`

启用并立即执行：

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now radar-static-ip.service
```

检查状态：

```bash
systemctl status radar-static-ip.service
ip -4 addr show dev eth0
ip route get 192.168.10.3 from 192.168.10.50
```
## 1. 功能概述

`indooruav_http` 包含两个 ROS 节点：

- `indooruav_http_server`
  - 对外提供本地 HTTP 接口
  - 当前主要接收前端的 `/airlineInfo` 和 `/sendCommand`
  - 其中 `commandMode=1` 会转发到 `indooruav_core/state_machine_event/takeoff_command`

- `indooruav_http_client`
  - 按接口文档向前端服务器上报设备状态、飞行状态、报警、起飞许可、任务完成、图片上传等信息
  - 支持手动传图，也支持降落完成后的自动回传流程

当前已支持的关键能力：

- 缓存前端通过 `/airlineInfo` 下发的 `siteId`、`deviceId`、`airlineKey`、`detectTimeCur`
- 前端通过 `/sendCommand?commandMode=1` 触发起飞事件
- 通过 `/indooruav_http/send_pic` 上传单张本地图片
- 通过 `/indooruav_http/send_pic_over` 通知图片批次上传完成
- 在降落完成后自动执行：
  - `1.9 sendFlyOver`
  - `1.10 sendPic`
  - `1.11 sendPicOver`
- 自动传图支持两种图片来源模式：
  - `local_fs`：从机载电脑本地目录扫描图片后上传
  - `drone_sd_card`：通过 `indooruav_controller` 从无人机 SD 卡读取图片字节流后直接上传前端

接口文档参考：[drone-interface.md](/home/wwt/catkin_ws/src/indooruav_http/drone-interface.md)

## 2. 构建

依赖环境：

- ROS1 Noetic
- `cpp-httplib`
- `nlohmann/json`

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
catkin_make --pkg indooruav_msgs indooruav_http indooruav_core
source ~/catkin_ws/devel/setup.bash
```

如果你同时启用了 `drone_sd_card` 模式，还需要 `indooruav_controller` 能成功编译。

## 3. 启动方式

### 3.1 一键启动

```bash
roslaunch indooruav_http bringup_indooruav_http.launch
```

说明：

- 这个 launch 现在只启动 `indooruav_http_server` 和 `indooruav_http_client`
- 不会再自动启动 `indooruav_core`

可选参数示例：

```bash
roslaunch indooruav_http bringup_indooruav_http.launch \
  server_config:=/path/to/http_server.yaml \
  client_config:=/path/to/http_client.yaml
```

### 3.2 单独启动 HTTP 服务端

```bash
rosrun indooruav_http indooruav_http_server _local_port:=20000
```

主要本地 HTTP 接口：

- `/airlineInfo`
- `/sendCommand`

说明：

- `/airlineInfo` 会同步当前任务的 `siteId`、`deviceId`、`airlineKey`、`detectTimeCur`
- `commandMode=1` 当前表示起飞
- 请求链路为：
  - 前端 `/sendCommand`
  - `indooruav_http_server`
  - `indooruav_core/state_machine_event/takeoff_command`

### 3.3 单独启动 HTTP 客户端

```bash
rosrun indooruav_http indooruav_http_client \
  _server_ip:=127.0.0.1 \
  _server_port:=8080 \
  _site_id:=11 \
  _device_id:=1
```

常用参数说明：

- `server_ip` / `server_port`
  - 前端服务器地址
- `site_id` / `device_id`
  - 站点编号和设备编号
  - 作为默认值使用；一旦前端调用 `/airlineInfo`，后续机载上报会改用 `/airlineInfo` 里的 `siteId` 和 `deviceId`
- `device_state_interval`
  - 设备状态上报周期
- `flight_state_interval`
  - 飞行状态上报周期
- `flight_state_sample_rate`
  - 飞行状态采样频率
- `takeoff_state_interval`
  - 起飞许可上报周期
- `uav_online_timeout_sec`
  - 多久没收到遥测就把 `uavState` 判为离线
- `post_land_image_root_dir`
  - `local_fs` 模式下的图片根目录
- `post_land_image_source_mode`
  - 图片来源模式，可选 `local_fs` 或 `drone_sd_card`
- `controller_upload_mission_media_service`
  - `drone_sd_card` 模式下调用的 controller service 名称

当前可用 ROS service：

- `/indooruav_http/send_airline`
- `/indooruav_http/send_pic`
- `/indooruav_http/send_pic_over`
- `/indooruav_http/send_fly_over`
- `/indooruav_http/send_error_data`
- `/indooruav_http/set_takeoff_state`
- `/indooruav_http/airline_sync`
- `/indooruav_http/run_post_land_workflow`
- `/indooruav_http/upload_image_bytes`

### 3.4 状态机自动触发链路

如果你启用了 `indooruav_core` 和自动回传链路，那么在状态机发生：

- `LandComplete -> Charge`

时，`indooruav_core` 会调用：

- `/indooruav_http/run_post_land_workflow`

然后 `indooruav_http` 会后台异步执行：

- `1.9 sendFlyOver`
- 图片上传
- `1.11 sendPicOver`

### 3.4 雷达静态 IP 开机启动

本仓库已包含系统服务模板和配置文件：

- `config/radar-static-ip.service`
- `config/radar-static-ip.env`
- `scripts/configure_radar_static_ip.sh`

运行以下命令即可把服务安装到系统并开启开机启动：

```bash
cd $(dirname "$(realpath "$0")")
cd ..
sudo bash scripts/install_radar_static_ip.sh
```

安装完成后，使用下面命令检查服务状态：

```bash
sudo systemctl status radar-static-ip.service
```

如果需要立即生效，可运行：

```bash
sudo systemctl start radar-static-ip.service
```

默认配置文件拷贝到：

- `/etc/indooruav/radar-static-ip.env`
- `/etc/systemd/system/radar-static-ip.service`
- `/opt/indooruav/scripts/configure_radar_static_ip.sh`

请根据实际机载电脑网口名称调整 `config/radar-static-ip.env` 中的 `ETH_IFACE` 和 IP 参数。

当前可用 ROS service：

- `/indooruav_http/send_airline`
- `/indooruav_http/send_pic`
- `/indooruav_http/send_pic_over`
- `/indooruav_http/send_fly_over`
- `/indooruav_http/send_error_data`
- `/indooruav_http/set_takeoff_state`
- `/indooruav_http/airline_sync`
- `/indooruav_http/run_post_land_workflow`
- `/indooruav_http/upload_image_bytes`

## 4. 图片上传逻辑

### 4.1 手动单张传图

调用：

- `/indooruav_http/send_pic`

请求参数：

- `image_path`

行为：

- 读取本地文件
- 使用当前缓存的 `airlineKey` 和 `detectTimeCur`
- 调用前端 `POST /sendPic`
- 不会自动触发 `1.11 sendPicOver`

### 4.2 自动回传图片命名规则

无论图片来自本地文件还是 SD 卡字节流，上传到前端时都使用同一命名规则：

- 同一批次 `detectTimeCur` 下按顺序命名
- 文件名格式为：
  - `1.jpg`
  - `2.jpg`
  - `3.png`
  - ...
- 扩展名跟随原始图片类型

也就是说，前端最终保存路径一般会表现为：

- `D:/tycho/imgs/<deviceId>/YYYY/MM/DD/<detectTimeCur>/1.jpg`
- `D:/tycho/imgs/<deviceId>/YYYY/MM/DD/<detectTimeCur>/2.jpg`

其中：

- `YYYY/MM/DD` 从 `detectTimeCur` 推导
- 最终 Windows 路径由前端服务器负责创建

### 4.3 local_fs 模式，本地电脑测试传图模式

当：

- `post_land_image_source_mode: "local_fs"`

时，自动回传会扫描：

- `<post_land_image_root_dir>/<detectTimeCur>/`

只做一层非递归扫描，按文件名字典序上传，支持的扩展名：

- `.jpg`
- `.jpeg`
- `.png`
- `.bmp`
- `.webp`
- `.dng`
- `.tif`
- `.tiff`

### 4.4 drone_sd_card 模式  sd卡直传模式

当：

- `post_land_image_source_mode: "drone_sd_card"`

时，自动回传不会扫描本地图片目录，而是调用：

- `/indooruav_controller/controller_hardware/upload_mission_photos_from_sd`

由 `indooruav_controller` 完成以下流程：

- 申请 downloader rights
- 从 PSDK 相机管理模块下载文件列表
- 按当前任务的 `detectTimeCur` 筛选本次任务照片
- 逐张从无人机 SD 卡下载图片字节流
- 每张通过 `/indooruav_http/upload_image_bytes` 交给 `indooruav_http`
- `indooruav_http` 直接以内存字节流调用前端 `POST /sendPic`

这个流程不会把中间图片文件落盘到机载电脑。

## 5. 配置说明

### 5.1 indooruav_http/config/http_client.yaml

当前关键配置示例：

```yaml
server_ip: "127.0.0.1"
server_port: 8080
site_id: 11
device_id: 1
post_land_image_root_dir: "/tmp/indooruav_post_land_images"
post_land_image_source_mode: "local_fs"
controller_upload_mission_media_service: "/indooruav_controller/controller_hardware/upload_mission_photos_from_sd"
```

### 5.2 indooruav_controller/config/config.yaml

SD 卡直传相关关键配置：

```yaml
indooruav_controller:
  services:
    upload_mission_photos_from_sd: "indooruav_controller/controller_hardware/upload_mission_photos_from_sd"
    http_upload_image_bytes: "/indooruav_http/upload_image_bytes"

  parameters:
    media_camera_mount_position: -1    //相机挂载编号
    media_time_tolerance_sec: 5.0
    media_file_wait_timeout_sec: 60.0
```

重点说明：

- `media_camera_mount_position` 必须按你的实机挂载位配置
- 默认值 `-1` 表示未配置
- 没配这个值时，`drone_sd_card` 模式会直接返回前置条件错误，不会开始取图

## 6. 本地假前端测试

下面这套命令主要验证：

- `1.9 sendFlyOver`
- `1.10 sendPic`
- `1.11 sendPicOver`
- 降落后的自动回传流程

### 6.1 终端 1：启动假前端服务器

如果你是在自己的电脑上测试，最简单的方式是直接运行仓库里的脚本：

Linux / WSL：

```bash
python3 /home/wwt/catkin_ws/src/indooruav_http/scripts/fake_frontend_server.py \
  --host 0.0.0.0 \
  --port 8080 \
  --save-root /tmp/fake_frontend_imgs
```

Windows PowerShell：

```powershell
py "\\wsl.localhost\Ubuntu-20.04\home\wwt\catkin_ws\src\indooruav_http\scripts\fake_frontend_server.py" --host 0.0.0.0 --port 8080 --save-root D:\fake_frontend_imgs
```

如果你还是想临时用一段内联 Python，也可以继续用下面这段：

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

### 6.2 终端 2：准备 local_fs 测试图片

```bash
mkdir -p /tmp/indooruav_post_land_images/20250701125959
cp /home/wwt/catkin_ws/src/indooruav_core/scripts/image.png /tmp/indooruav_post_land_images/20250701125959/001.png
cp /home/wwt/catkin_ws/src/indooruav_core/scripts/image.png /tmp/indooruav_post_land_images/20250701125959/002.png
find /tmp/indooruav_post_land_images -type f | sort
```

### 6.3 终端 3：启动 core + http

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg indooruav_msgs indooruav_http indooruav_core
source ~/catkin_ws/devel/setup.bash
roslaunch indooruav_http bringup_indooruav_http.launch
```

确认 `http_client.yaml` 至少满足：

```yaml
server_ip: "127.0.0.1"
server_port: 8080
post_land_image_source_mode: "local_fs"
post_land_image_root_dir: "/tmp/indooruav_post_land_images"
```

### 6.4 终端 4：只测试 http 自动工作流

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

### 6.5 终端 4：测试 core 自动触发

注意：直接发 `land_complete` 之前，状态机必须已经处于 `Land` 状态。

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

curl "http://127.0.0.1:20000/airlineInfo?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959"

rosservice call /indooruav_core/state_machine_event/takeoff_command
rosservice call /indooruav_core/state_machine_event/check_passed
rosservice call /indooruav_core/state_machine_event/takeoff_complete
rosservice call /indooruav_core/state_machine_event/cruise_complete
rosservice call /indooruav_core/state_machine_event/land_complete
```

正常情况下：

- `indooruav_core` 会进入 `Charge`
- `indooruav_http` 会自动执行 `1.9 -> 多次 1.10 -> 1.11`

## 7. SD 卡直传模式说明

### 7.1 目标

`drone_sd_card` 模式的目标是：

- 不依赖机载电脑中间落盘目录
- 直接通过 PSDK 从无人机 SD 卡读取图片
- 以内存字节流上传到前端 `POST /sendPic`

### 7.2 需要的配置

至少需要：

- `indooruav_http/config/http_client.yaml`

```yaml
post_land_image_source_mode: "drone_sd_card"
controller_upload_mission_media_service: "/indooruav_controller/controller_hardware/upload_mission_photos_from_sd"
```

- `indooruav_controller/config/config.yaml`

```yaml
indooruav_controller:
  parameters:
    media_camera_mount_position: 1
    media_time_tolerance_sec: 5.0
    media_file_wait_timeout_sec: 60.0
```

其中：

- `media_camera_mount_position` 的值必须按你的 Matrice 4T 当前 PSDK 挂载位确认
- 上面 `1` 只是示例，不是通用固定值

### 7.3 新增 ROS 接口

共享 service：

- `indooruav_msgs/UploadImageBytes`
- `indooruav_msgs/TransferMissionMedia`

内部 service：

- `/indooruav_http/upload_image_bytes`
- `/indooruav_controller/controller_hardware/upload_mission_photos_from_sd`

### 7.4 当前限制

- 只处理静态图片
- 当前只筛选并上传：
  - `JPEG`
  - `DNG`
  - `TIFF`
- 视频文件不会上传
- 图片筛选仍依赖 `detectTimeCur`
- 时间筛选窗口由 `media_time_tolerance_sec` 控制
- 同一时刻只允许一个 SD 卡批处理运行

## 8. 常见问题

### 8.1 为什么手动 `send_pic` 之后没有自动调用 `1.11`

这是当前设计如此：

- 手动 `/indooruav_http/send_pic` 只负责上传单张图片
- 不会自动调用 `/sendPicOver`
- `1.11` 只在你手动调用 `/indooruav_http/send_pic_over`，或者自动降落后工作流结束时调用

### 8.2 为什么重复上传图片有时失败

常见原因不是“同名覆盖”，而是：

- 前端服务不可达
- `airlineKey` 或 `detectTimeCur` 没先通过 `/airlineInfo` 写入
- 本地图片路径不可读
- 前端服务对重复文件名有自己的拒绝策略

当前客户端上传命名规则会在同一 `detectTimeCur` 下按 `1、2、3...` 递增，不会主动覆盖前一次的同名文件。

### 8.3 自动回传为什么没触发

最常见原因：

- `indooruav_core` 当前并不在 `Land` 状态，直接发 `land_complete` 会被忽略
- 没先调用 `/airlineInfo`
- `post_land_image_source_mode` 配置错了
- `drone_sd_card` 模式下没有配置 `media_camera_mount_position`

## 9. 真机 SD 卡取图测试步骤

### 9.1 终端 1：启动本机假前端

因为你现在没有真前端，可以先在本机起一个假前端：

```bash
mkdir -p /tmp/fake_frontend_imgs

python3 - <<'PY'
from http.server import BaseHTTPRequestHandler, HTTPServer
import cgi, json, os, urllib.parse

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
            return self.reply()
        return self.reply()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        print(f"[GET] {parsed.path}?{parsed.query}", flush=True)
        if parsed.path == "/airlineSync":
            return self.reply(200, {"resultCode": 1, "result": []})
        return self.reply()

    def log_message(self, fmt, *args):
        pass

print("fake frontend listening on 0.0.0.0:8080", flush=True)
HTTPServer(("0.0.0.0", 8080), Handler).serve_forever()
PY
```

### 9.2 终端 2：启动 ROS 环境和节点

先启动 `indooruav_controller`：

```bash
source /opt/ros/noetic/setup.bash
source ~/Project/IndoorUavInspection2/catkin_ws/devel/setup.bash

roslaunch ~/Project/IndoorUavInspection2/catkin_ws/src/indooruav_controller/launch/bringup_controller_hardware.launch
```

再另开一个终端，启动 `indooruav_http`，不用启动 `core`：

```bash
source /opt/ros/noetic/setup.bash
source ~/Project/IndoorUavInspection2/catkin_ws/devel/setup.bash

roslaunch ~/Project/IndoorUavInspection2/catkin_ws/src/indooruav_http/launch/bringup_indooruav_http.launch
```

如果你的 `rospack` 环境已经正常，也可以用包名启动：

```bash
roslaunch indooruav_controller bringup_controller_hardware.launch
roslaunch indooruav_http bringup_indooruav_http.launch
```

### 9.3 终端 3：先记录任务时间，再写入 `/airlineInfo`

```bash
source /opt/ros/noetic/setup.bash
source ~/Project/IndoorUavInspection2/catkin_ws/devel/setup.bash

export DETECT=$(date +%Y%m%d%H%M%S)
echo $DETECT

python3 - <<PY
import os, urllib.request
detect = os.environ["DETECT"]
url = f"http://127.0.0.1:20000/airlineInfo?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur={detect}"
print(urllib.request.urlopen(url).read().decode())
PY
```

### 9.4 在地面直接拍 1 到 2 张照片，写进无人机 SD 卡

```bash
rosservice call /indooruav_controller/controller_hardware/camera_mode_photo
rosservice call /indooruav_controller/controller_hardware/camera_photo_shoot
sleep 3
rosservice call /indooruav_controller/controller_hardware/camera_photo_shoot
sleep 5
```

### 9.5 直接触发“降落后回传工作流”，不用发 `land_complete`

```bash
rosservice call /indooruav_http/run_post_land_workflow
```

### 9.6 查看结果

```bash
find /tmp/fake_frontend_imgs -type f | sort
```

### 9.7 成功时你应该看到什么

假前端终端里如果成功，会出现：

```text
[GET] /sendFlyOver?...
[sendPic] saved: /tmp/fake_frontend_imgs/1/YYYY/MM/DD/<detectTimeCur>/1.jpg
[sendPic] saved: /tmp/fake_frontend_imgs/1/YYYY/MM/DD/<detectTimeCur>/2.jpg
[GET] /sendPicOver?...
```

这就说明：

- `1.9` 调到了
- SD 卡图片取出来并上传了
- `1.11` 也调到了

### 9.8 如果你想先单独测“SD 卡取图”

可以直接调 `controller`，不走整个 `http` 工作流：

```bash
rosservice call /indooruav_controller/controller_hardware/upload_mission_photos_from_sd "{airline_key: 'AAAA', detect_time_cur: '$DETECT'}"
```

重点看返回里的：

- `matched_count`
- `uploaded_count`
- `failed_count`

