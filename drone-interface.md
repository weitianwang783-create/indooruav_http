# indooruav_http 接口说明

本文档只整理当前 `indooruav_http` 已实现、且你现在会实际用到的接口。

## 1. 本地 HTTP 接口

这些接口由 `indooruav_http_server` 提供，前端调用的是机载电脑本地 HTTP 服务。

### 1.1 `/airlineInfo`

用途：

- 前端下发当前任务标识
- 缓存 `siteId`
- 缓存 `deviceId`
- 缓存 `airlineKey`
- 缓存 `detectTimeCur`

请求示例：

```text
GET /airlineInfo?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959
```

参数说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `siteId` | int | 站点编号 |
| `deviceId` | int | 设备编号 |
| `airlineKey` | string | 当前任务唯一标识 |
| `detectTimeCur` | string | 当前检测时间，格式 `YYYYMMDDHHMMSS` |

返回示例：

```json
{"resultCode":1}
```

说明：

- `/airlineInfo` 的四个字段会作为当前任务上下文
- 后续机载发往前端的上报请求会优先复用这四个字段
- 如果还没收到 `/airlineInfo`，则 `siteId` 和 `deviceId` 回退为节点启动参数

### 1.2 `/sendCommand`

用途：

- 前端发送控制命令到本地服务端
- 当前已经实现 `commandMode=1` 触发起飞

请求示例：

```text
GET /sendCommand?siteId=11&deviceId=1&commandMode=1
```

参数说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `siteId` | int | 站点编号 |
| `deviceId` | int | 设备编号 |
| `commandMode` | int | 命令类型，当前 `1` 表示起飞 |

当前实际转发链路：

- 前端 `/sendCommand?commandMode=1`
- `indooruav_http_server`
- `indooruav_core/state_machine_event/takeoff_command`

返回示例：

```json
{"resultCode":1}
```

说明：

- `resultCode=1` 表示本地 HTTP 服务端成功把请求转发给状态机 service
- 不表示无人机已经真的完成起飞

## 2. 前端服务器上报接口

这些接口由 `indooruav_http_client` 调用前端服务器。

### 2.1 `1.9 sendFlyOver`

用途：

- 无人机任务完成后通知前端

请求格式：

```text
GET /sendFlyOver?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959
```

参数说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `siteId` | int | 站点编号 |
| `deviceId` | int | 设备编号 |
| `airlineKey` | string | 当前任务标识 |
| `detectTimeCur` | string | 当前检测时间 |

返回示例：

```json
{"resultCode":1}
```

### 2.2 `1.10 sendPic`

用途：

- 上传单张图片到前端服务器

请求格式：

```text
POST /sendPic?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959
Content-Type: multipart/form-data
field name: file
```

参数说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `siteId` | int | 站点编号 |
| `deviceId` | int | 设备编号 |
| `airlineKey` | string | 当前任务标识 |
| `detectTimeCur` | string | 当前检测时间 |
| `file` | 文件 | 单张图片，使用 `multipart/form-data` 上传 |

当前实现的上传命名规则：

- 同一 `detectTimeCur` 批次内，按顺序命名
- 文件名格式：
  - `1.jpg`
  - `2.jpg`
  - `3.png`
  - ...

扩展名来源：

- 本地文件上传时，跟随本地文件扩展名
- SD 卡直传时，跟随 PSDK 媒体文件类型或原始文件名扩展名

前端保存目录约定：

```text
D:/tycho/imgs/<deviceId>/YYYY/MM/DD/<detectTimeCur>/
```

例如：

```text
D:/tycho/imgs/1/2025/07/01/20250701125959/1.png
D:/tycho/imgs/1/2025/07/01/20250701125959/2.png
```

注意：

- 路径创建和最终落盘由前端服务器负责
- `indooruav_http` 只负责上传文件和命名

返回示例：

```json
{"resultCode":1}
```

### 2.3 `1.11 sendPicOver`

用途：

- 通知前端“本批次图片上传完成”

请求格式：

```text
GET /sendPicOver?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959
```

参数说明：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `siteId` | int | 站点编号 |
| `deviceId` | int | 设备编号 |
| `airlineKey` | string | 当前任务标识 |
| `detectTimeCur` | string | 当前检测时间 |

返回示例：

```json
{"resultCode":1}
```

说明：

- 手动调用 `/indooruav_http/send_pic` 后，不会自动触发 `1.11`
- 自动降落后工作流结束时，会自动调用 `1.11`

## 3. 自动降落后回传工作流

当前自动工作流触发点：

- `indooruav_core`
- 状态机在 `LandComplete -> Charge` 时
- 调用 `/indooruav_http/run_post_land_workflow`

随后 `indooruav_http` 后台异步执行：

1. `1.9 sendFlyOver`
2. 图片上传
3. `1.11 sendPicOver`

图片来源模式：

- `local_fs`
  - 扫描 `<post_land_image_root_dir>/<detectTimeCur>/`
- `drone_sd_card`
  - 调用 `indooruav_controller` 通过 PSDK 从无人机 SD 卡下载图片字节流后上传

## 4. 当前相关 ROS 接口

### 4.1 `indooruav_http`

| Service | 类型 | 说明 |
| --- | --- | --- |
| `/indooruav_http/send_pic` | `indooruav_http/SendPic` | 手动上传本地图片 |
| `/indooruav_http/send_pic_over` | `indooruav_http/SendPicOver` | 手动通知图片上传完成 |
| `/indooruav_http/send_fly_over` | `indooruav_http/SendFlyOver` | 手动调用 1.9 |
| `/indooruav_http/run_post_land_workflow` | `std_srvs/Empty` | 启动自动降落后回传流程 |
| `/indooruav_http/upload_image_bytes` | `indooruav_msgs/UploadImageBytes` | 给 controller 传单张图片字节流的内部 service |

### 4.2 `indooruav_controller`

| Service | 类型 | 说明 |
| --- | --- | --- |
| `/indooruav_controller/controller_hardware/upload_mission_photos_from_sd` | `indooruav_msgs/TransferMissionMedia` | 从无人机 SD 卡筛选并上传当前任务图片 |

## 5. 结果码约定

当前相关接口统一按以下规则理解：

| resultCode | 含义 |
| --- | --- |
| `1` | 成功 |
| `2` | 运行期失败，例如前端服务不可达、HTTP 非 200、调用链失败 |
| `3` | 前置条件错误，例如缺少 `airlineKey`、`detectTimeCur`、本地文件不可读、配置缺失 |

