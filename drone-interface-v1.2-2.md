接口文档，只读，不可更改

# 服务器端接口

## 1.1、发送新建航线信息

http://ip:port/sendAirline?siteId=11&deviceId=1&file=file.json

字段格式说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| file | 文件 | 航线JSON文件 |

file 中航点JSON文件中字段格式说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| airlineKey | String | 航线唯一标识 |
| airlineMap | String | 航线地图，base64图片，如果http图片地址也可以 |
| waypointList | 集合 | 航点集合 |

waypointList字段格式说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| waypointx | double | 航点x坐标，精度0.00 |
| waypointy | double | 航点y坐标，精度0.00 |
| waypointz | double | 航点z坐标，精度0.00 |
| distance | double | 距离，精度0.00 |
| angle | double | 角度，精度0.00 |

JSON示例：

| { "airlineKey": "AAAAAAAAA", "airlineMap": "AAAAAAAAA", "waypointList": [ { "distance": 1, "angle": 1, "waypointx": 1, "waypointy": 1, "waypointz": 1 }, { "distance": 2, "angle": 2, "waypointx": 2, "waypointy": 2, "waypointz": 2 }, { "distance": 3, "angle": 3, "waypointx": 3, "waypointy": 3, "waypointz": 3 } ] } |
| --- |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确，5 airlineKey重复 |

返回值示例：

| { "resultCode": 1 } |
| --- |

## 1.2、发送设备状态信息

http://ip:port/sendDeviceData?siteId=11&deviceId=1&file=file.json

协议属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| file | 文件 | 设备状态JSON文件 |

file 中设备状态JSON文件中字段格式说明：（设备状态需可设置(30秒)时间发送一次）

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| uavState | int | 无人机状态：1在线，0离线 |
| controlState | int | 遥控器状态：1在线，0离线 |
| controlSoc | double | 遥控器电量：0.0%-100.0%，精度0.00 |
| controlRssi | double | 遥控器信号强度：0.0%-100.0%，精度0.00 |
| batteryTemp | double | 无人机电池温度,，单位℃，精度0.00 |
| batterySoc | double | 无人机电池电量：0.00%-100.00%，精度0.00 |
| batteryRssi | double | 无人机信号强度：0.0%-100.0%，精度0.00 |
| batteryVolt | double | 无人机电池电压，单位V，精度0.00 |
| batteryCycleNum | int | 无人机电池循环次数 |
| droneNestState | int | 机巢状态：1在线，0离线 |
| putterLrState | int | 左右推杆状态：1松开，0 闭合 |
| putterBaState | int | 前后推杆状态：1松开，0 闭合 |
| putterState | int | 推杆整体状态：1松开，0 闭合 |
|  |  | (如果有其他类型，后续继续添加) |

JSON示例：

| { "uavState": 1, "controlState": 1, "controlSoc": 90.1, "controlRssi": 99.9, "batteryTemp": 50.5, "batterySoc": 90.9, "batteryRssi": 99.9, "batteryVolt": 40.1, "batteryCycleNum": 1, "droneNestState": 1, "putterLrState": 1, "putterBaState": 1, "putterState": 1 } |
| --- |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确 |

返回值示例：

| { "resultCode": 1 } |
| --- |

## 1.3、发送无人机位置信息

http://ip:port/sendFlyData?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959&file=file.json

协议属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| airlineKey | String | 当前航线唯一标识 |
| detectTimeCur | String | 检测时间，格式：20250701125959 |
| file | 文件 | 无人机位置信息JSON文件 |

file 中无人机位置信息JSON文件中字段格式说明：（当前航线信息需设置(3秒)时间发送一次）

无人机这里一秒记录1次，每3秒发送含有3个元素的数组，每个元素有如下数据

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| timeStamp | string | 记录时间，格式：2025-07-01 12:59:59 |
| positionx | double | DJI无人机当前位置 x精度0.00 |
| positiony | double | DJI无人机当前位置 y精度0.00 |
| positionz | double | DJI无人机当前位置 z精度0.00 |
| attitudeRoll | double | DJI无人机当前姿态roll精度0.00 |
| attitudePitch | double | DJI无人机当前姿态pitch精度0.00 |
| attitudeYaw | double | DJI无人机当前姿态yaw精度0.00 |
| horizontalSpeed | double | DJI无人机当前水平速度，单位m/s，精度0.00 |
| verticalSpeed | double | DJI无人机当前垂直速度，单位m/s，精度0.00 |
| lineType | int | 当前航点类型：1全局航线，0局部航线 |
| poseAngleRoll | double | DJI无人机云台姿态角roll精度0.00 |
| poseAnglePitch | double | DJI无人机云台姿态角pitch精度0.00 |
| poseAngleYaw | double | DJI无人机云台姿态角yaw精度0.00 |
| pantographIs | int | 机载D435深度相机是否检测到受电弓： 1，此刻有检测到，0，此刻没有检测到 |
| abnormalIs | int | 机载D435深度相机是否检测到异物： 1，此刻有检测到，0，此刻没有检测到 |
| pantographLocx | double | 受电弓坐标x精度0.00 |
| pantographLocy | double | 受电弓坐标y精度0.00 |
| pantographLocz | double | 受电弓坐标z精度0.00 |
| abnormalLocx | double | 异物坐标x精度0.00 只有计算完了坐标才会发送，只要发送了(非0.0，0.0，0.0这种空坐标)，客户端就应该绘制到CAD地图上 |
| abnormalLocy | double | 异物坐标y精度0.00 |
| abnormalLocz | double | 异物坐标z精度0.00 |
| (如果有其他类型，后续继续添加) |  |  |

JSON示例：

| [ { "timeStamp": "2025-10-15 13:57:38", "positionx": 11.1, "positiony": 22.2, "positionz": 33.3, "attitudeRoll": 111.4, "attitudePitch": 222.5, "attitudeYaw": 333.6, "horizontalSpeed": 20.7, "verticalSpeed": 1.8, "lineType": 1, "poseAngleRoll": 1111.9, "poseAnglePitch": 2222.1, "poseAngleYaw": 3333.2, "pantographIs": 0, "abnormalIs": 0, "pantographLocx": 10.3, "pantographLocy": 10.4, "pantographLocz": 10.5, "abnormalLocx": 20.6, "abnormalLocy": 20.7, "abnormalLocz": 20.8 } ] |
| --- |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确 |

返回值示例：

| { "resultCode": 1 } |
| --- |

## 1.4、发送报警、安全信息

http://ip:port/sendErrorData?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959&file=file.json

协议属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| airlineKey | String | 当前航线唯一标识 |
| detectTimeCur | String | 检测时间，格式：20250701125959 |
| file | 文件 | 报警信息JSON文件 |

file 中报警信息JSON文件中字段格式说明：

（机载检测到了错误才会发送，不会高频发送。客户端收到此类数据要立刻解析和呈现）

无人机仅向机载报告产生了错误（给客户端呈现一下），但是立即执行错误处理

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| timeStamp | string | 报警时间，格式：2025-07-01 12:59:59 |
| errorType | int | 报警类型： 1位置错误。 2姿态错误。 3电池容量报警。 4碰撞报警。 如果有其他类型，后续继续添加 |
| errorInfo | string | 报警信息： 1可能是错误位置信息 2可能是错误姿态信息 3 电池当前容量 4可能是发生碰撞位置信息 等 |

JSON示例：

| { "timeStamp": "2025-10-15 11:47:00", "errorType": 4, "errorInfo": "[22,33,44]" } |
| --- |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确 |

返回值示例：

| { "resultCode": 1 } |
| --- |

## 1.6、起飞许可

http://ip:port/sendTakeoffState?siteId=11&deviceId=1&takeoffState=1

协议属性说明：需可设置(3秒)时间发送一次

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| takeoffState | int | 1可以起飞：航点JSON文件解析无误，无人机自检无误（激光雷达、双目深度相机、机载与无人机的通讯、电池容量均没有问题） 2不可起飞：没有选择航线 3不可起飞：无人机自检有误 4不可起飞：激光雷达有问题 5不可起飞：双目深度相机有问题 6不可起飞：机载与无人机的通讯有问题 7不可起飞：电池容量有问题 |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确 |

返回值格式：

| { "resultCode": 1 } |
| --- |

## 1.9、无人机返回（任务完成）

http://ip:port/sendFlyOver?siteId=11&deviceId=1&airlineKey=AAAA&detectTimeCur=20250701125959

协议属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| airlineKey | String | 当前航线唯一标识 |
| detectTimeCur | String | 检测时间，格式：20250701125959 |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确 |

返回值格式：

| { "resultCode": 1 } |
| --- |


1.10 sendPic`

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

## 1.11、上传图片完成

http://ip:port/sendPicOver?siteId=11&deviceId=1&airlineKey =1&takeoffState=1&detectTimeCur=20250701125959

协议属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| airlineKey | String | 当前航线唯一标识 |
| detectTimeCur | String | 检测时间，格式：20250701125959 |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确 |

返回值格式：

| { "resultCode": 1 } |
| --- |

## 1.14、航线信息同步

http://ip:port/airlineSync?siteId=11&deviceId=1

协议属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 没有数据 |
| result | 集合 | 航线信息集合 |

result中字段格式说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| airlineKey | String | 航线唯一标识 |
| airlineMap | String | 航线地图，base64图片，如果http图片地址也可以 |
| waypointList | 集合 | 航点集合 |

waypointList字段格式说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| waypointx | double | 航点x坐标，精度0.00 |
| waypointy | double | 航点y坐标，精度0.00 |
| waypointz | double | 航点z坐标，精度0.00 |
| distance | double | 距离，精度0.00 |
| angle | double | 角度，精度0.00 |

result示例：

| { "result": [ { "waypointList": [ { "distance": 1, "waypointz": 1, "waypointy": 1, "waypointx": 1, "angle": 1 }, { "distance": 2, "waypointz": 2, "waypointy": 2, "waypointx": 2, "angle": 2 }, { "distance": 3, "waypointz": 3, "waypointy": 3, "waypointx": 3, "angle": 3 } ], "airlineKey": "AAAAAAAAA", "airlineMap": "AAAAAAAAA" } ], "resultCode": 1 } |
| --- |

# 无人机端接口

## 1.5、选择航线

http://ip:port/airlineInfo?siteId=11&deviceId=1&airlineKey=AAAAAAA&detectTimeCur=20250701125959

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| airlineKey | String | 航线唯一标识 |
| detectTimeCur | String | 检测时间，格式：20250701125959 |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确 |

返回值格式：

| { "resultCode": 1 } |
| --- |

## 1.7、控制命令

http://ip:port/sendCommand?siteId=11&deviceId=1&commandMode=1

控制命令JSON对象中数据格式说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| siteId | int | 站点ID，需可设置，默认11 |
| deviceId | int | 设备ID，需可设置，默认1 |
| commandMode | int | 1. 起飞 2. 定点悬停 3. 原地降落 4. 返航 5. 开始充电 6. 停止充电 (如果有其他类型，后续继续添加) |

返回值对象属性说明：

| 属性名 | 类型 | 描述 |
| --- | --- | --- |
| resultCode | Integer | 请求结果:1 成功，2 失败，3 数据格式不正确 |

返回值格式：

| { "resultCode": 1 } |
| --- |
