#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mpc_ros2_pkg/MpcCal.h"
#include "mpc_ros2_pkg/matplotlibcpp.h" 

#include <stdint.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

namespace plt = matplotlibcpp;

class LeaderNode : public rclcpp::Node {
public:
	LeaderNode() : Node("leader_node") {
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		vehicle_local_positon_sub_ = this->create_subscription<VehicleLocalPosition>
		("/fmu/out/vehicle_local_position_v1", qos, [this](const VehicleLocalPosition msg) {
			local_position_ = msg;
		});

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				this->PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->Arm();
			}

			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}

			if (abs(local_position_.z) <= 4.5) {
				PublishOffboardControlMode(true, false, false);
				TakeOff();
			}
			else {
				PublishOffboardControlMode(true, true, true);
				this->MpcFun();
			}

		};
		timer_ = this->create_wall_timer(20ms, timer_callback);
	}
private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_positon_sub_;

	std::atomic<uint64_t> timestamp_;  

	uint64_t offboard_setpoint_counter_;   

	VehicleLocalPosition local_position_{};

    std::string pkg_share = ament_index_cpp::get_package_share_directory("mpc_ros2_pkg");
    std::unique_ptr<GetRefPath> ref_ptr = std::make_unique<GetRefPath>(pkg_share+"/config/ref.csv");
	std::unique_ptr<MpcCal> mpc_ptr = std::make_unique<MpcCal>();

	bool is_path_get = ref_ptr->GetPath();
	vector<vector<double>> path = ref_ptr->path;

	int path_ref_index = 0;

	VectorXd sta_local = VectorXd::Zero(STANUM);
	VectorXd sta_except = VectorXd::Zero(STANUM);

	void GetStaLocal();
	void MpcFun();
	void DrawTra();

	void Arm();
	void DisArm();

	void PublishOffboardControlMode(bool pos_bl, bool vel_bl, bool acc_bl);
	void TakeOff();
	void PublishTrajectorSetpoint(VectorXd sta_except);
	void PublishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);

};

void LeaderNode::GetStaLocal() {
	sta_local(0) = local_position_.x; 
	sta_local(1) = local_position_.y;
	sta_local(2) = local_position_.vx;
	sta_local(3) = local_position_.vy;
	sta_local(4) = local_position_.ax;
	sta_local(5) = local_position_.ay;
}

void LeaderNode::MpcFun() {
	this->GetStaLocal();
	sta_except = sta_local;
	if (path.size() == COLS && is_path_get && path_ref_index < path.size() && mpc_ptr->MPCSolver(path[path_ref_index], sta_except)) {

		PublishTrajectorSetpoint(sta_except);
		path_ref_index += 1;
	}
	else if (path_ref_index >= path.size()) {
		this->DrawTra();
		RCLCPP_INFO(this->get_logger(), "all point solved, now shutdown");
		rclcpp::shutdown();
	}
	else {
		RCLCPP_WARN(this->get_logger(), "something wrong, now shutdown");
		rclcpp::shutdown();
	}
}

void LeaderNode::DrawTra() {
	vector<double> x;
	vector<double> y;

	size_t size = ref_ptr->path.size();

	x.reserve(size);
	y.reserve(size);

	plt::figure_size(1200, 700);

	for (size_t i = 0; i < size; i++) {
		x.push_back(ref_ptr->path[i][0]);
		y.push_back(ref_ptr->path[i][1]);
	}

	plt::plot(x, y, "k--");

	x.clear();
	y.clear();

	for (size_t i = 0;i < size; i++) {
		x.push_back(mpc_ptr->sta_node[i][0]);
		y.push_back(mpc_ptr->sta_node[i][1]);
	}

	plt::plot(x, y, "r-");

	plt::show();
}

void LeaderNode::Arm() {
	PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void LeaderNode::DisArm() {
	PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void LeaderNode::PublishOffboardControlMode(bool pos_bl, bool vel_bl, bool acc_bl) {
	OffboardControlMode msg{};
	msg.position = pos_bl;
	msg.velocity = vel_bl;
	msg.acceleration = acc_bl;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void LeaderNode::TakeOff() {
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = 0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void LeaderNode::PublishTrajectorSetpoint(VectorXd sta_except) {
	TrajectorySetpoint msg{};
	msg.position = {static_cast<float>(sta_except(0)), static_cast<float>(sta_except(1)), -5.0};
	msg.velocity = {static_cast<float>(sta_except(2)), static_cast<float>(sta_except(3)), 0.0};
	msg.acceleration = {static_cast<float>(sta_except(4)), static_cast<float>(sta_except(5)), 0.0};
	msg.yaw = 0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void LeaderNode::PublishVehicleCommand(uint16_t command, float param1, float param2) {
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LeaderNode>());
	rclcpp::shutdown();
	return 0;
}
