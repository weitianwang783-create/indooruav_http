#ifndef INDOORUAV_HTTP_DATA_TYPES_H
#define INDOORUAV_HTTP_DATA_TYPES_H

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace indooruav_http {

using json = nlohmann::json;

struct Waypoint {
	double waypointx = 0.0;
	double waypointy = 0.0;
	double waypointz = 0.0;
	double distance = 0.0;
	double angle = 0.0;

	static Waypoint FromJson(const json& j) {
		Waypoint wp;
		wp.waypointx = j.value("waypointx", 0.0);
		wp.waypointy = j.value("waypointy", 0.0);
		wp.waypointz = j.value("waypointz", 0.0);
		wp.distance = j.value("distance", 0.0);
		wp.angle = j.value("angle", 0.0);
		return wp;
	}

	json ToJson() const {
		return json{
			{"waypointx", waypointx},
			{"waypointy", waypointy},
			{"waypointz", waypointz},
			{"distance", distance},
			{"angle", angle}
		};
	}
};

struct Airline {
	std::string airline_key;
	std::string airline_map;
	double xscale = 0.0;
	double yscale = 0.0;
	double xzero = 0.0;
	double yzero = 0.0;
	std::vector<Waypoint> waypoint_list;

	static Airline FromJson(const json& j) {
		Airline airline;
		airline.airline_key = j.value("airlineKey", "");
		airline.airline_map = j.value("airlineMap", "");
		airline.xscale = j.value("xscale", 0.0);
		airline.yscale = j.value("yscale", 0.0);
		airline.xzero = j.value("xzero", 0.0);
		airline.yzero = j.value("yzero", 0.0);
		if (j.contains("waypointList") && j.at("waypointList").is_array()) {
			for (const auto& item : j.at("waypointList")) {
				airline.waypoint_list.push_back(Waypoint::FromJson(item));
			}
		}
		return airline;
	}

	json ToJson() const {
		json list = json::array();
		for (const auto& wp : waypoint_list) {
			list.push_back(wp.ToJson());
		}
		return json{
			{"airlineKey", airline_key},
			{"airlineMap", airline_map},
			{"xscale", xscale},
			{"yscale", yscale},
			{"xzero", xzero},
			{"yzero", yzero},
			{"waypointList", list}
		};
	}
};

struct DeviceState {
	int uav_state = 0;
	int control_state = 0;
	double control_soc = 0.0;
	double control_rssi = 0.0;
	double battery_temp = 0.0;
	double battery_soc = 0.0;
	double battery_rssi = 0.0;
	double battery_volt = 0.0;
	int battery_cycle_num = 0;

	json ToJson() const {
		return json{
			{"uavState", uav_state},
			{"controlState", control_state},
			{"controlSoc", control_soc},
			{"controlRssi", control_rssi},
			{"batteryTemp", battery_temp},
			{"batterySoc", battery_soc},
			{"batteryRssi", battery_rssi},
			{"batteryVolt", battery_volt},
			{"batteryCycleNum", battery_cycle_num}
		};
	}
};

struct FlightState {
	std::string time_stamp;
	double positionx = 0.0;
	double positiony = 0.0;
	double positionz = 0.0;
	double attitude_roll = 0.0;
	double attitude_pitch = 0.0;
	double attitude_yaw = 0.0;
	double horizontal_speed = 0.0;
	double vertical_speed = 0.0;
	int line_type = 1;
	double pose_angle_roll = 0.0;
	double pose_angle_pitch = 0.0;
	double pose_angle_yaw = 0.0;
	int pantograph_is = 0;
	int abnormal_is = 0;
	double pantograph_locx = 0.0;
	double pantograph_locy = 0.0;
	double pantograph_locz = 0.0;
	double abnormal_locx = 0.0;
	double abnormal_locy = 0.0;
	double abnormal_locz = 0.0;

	json ToJson() const {
		return json{
			{"timeStamp", time_stamp},
			{"positionx", positionx},
			{"positiony", positiony},
			{"positionz", positionz},
			{"attitudeRoll", attitude_roll},
			{"attitudePitch", attitude_pitch},
			{"attitudeYaw", attitude_yaw},
			{"horizontalSpeed", horizontal_speed},
			{"verticalSpeed", vertical_speed},
			{"lineType", line_type},
			{"poseAngleRoll", pose_angle_roll},
			{"poseAnglePitch", pose_angle_pitch},
			{"poseAngleYaw", pose_angle_yaw},
			{"pantographIs", pantograph_is},
			{"abnormalIs", abnormal_is},
			{"pantographLocx", pantograph_locx},
			{"pantographLocy", pantograph_locy},
			{"pantographLocz", pantograph_locz},
			{"abnormalLocx", abnormal_locx},
			{"abnormalLocy", abnormal_locy},
			{"abnormalLocz", abnormal_locz}
		};
	}
};

struct ErrorData {
	std::string time_stamp;
	int error_type = 0;
	std::string error_info;

	json ToJson() const {
		return json{
			{"timeStamp", time_stamp},
			{"errorType", error_type},
			{"errorInfo", error_info}
		};
	}
};

struct HttpResult {
	int result_code = 2;
	json result = json::object();
	std::string raw_body;
};

}  // namespace indooruav_http

#endif  // INDOORUAV_HTTP_DATA_TYPES_H
