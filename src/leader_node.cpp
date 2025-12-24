#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include "mpc_ros2_pkg/MpcCal.h"
#include <stdint.h>


#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class LeaderNode : public rclcpp::Node {
public:
	LeaderNode() : Node("leader_node") {
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		vehicle_local_positon_sub_ = this->create_subscription<VehicleLocalPosition>
		("/px4_1/fmu/out/vehicle_local_position_v1", qos, [this](const VehicleLocalPosition msg) {
			local_position_ = msg;
		});

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (abs(local_position_.z) <= 4.5) {
				PublishOffboardControlMode(true, false);
				PublishTrajectorSetpoint();
			}
			else {
				PublishOffboardControlMode(true, true);
				PublishTrajectorSetpoint();
			}

			if (offboard_setpoint_counter_ == 10) {
				this->PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->Arm();
			}

			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}

		};
		timer_ = this->create_wall_timer(50ms, timer_callback);
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

	void Arm();
	void DisArm();

	void PublishOffboardControlMode(bool pos, bool vel);
	void PublishTrajectorSetpoint();
	void PublishTrajectorSetpoint(float vel);
	void PublishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);

};

void LeaderNode::Arm() {
	PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void LeaderNode::DisArm() {
	PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void LeaderNode::PublishOffboardControlMode(bool pos_bl, bool vel_bl) {
	OffboardControlMode msg{};
	msg.position = pos_bl;
	msg.velocity = vel_bl;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void LeaderNode::PublishTrajectorSetpoint() {
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -5.0};
	msg.yaw = 0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void LeaderNode::PublishTrajectorSetpoint(float vel) {
	TrajectorySetpoint msg{};
	msg.position = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), -5.0};
	msg.velocity = {vel, 0.0, 0.0};
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
