#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "mpc_ros2_pkg/MpcCal.h"
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <cmath>
#include <vector>

using namespace std::chrono;
using namespace std::chrono_literals;

class MpcTestNode : public rclcpp::Node {
public:
	MpcTestNode() : Node("mpc_test_node") {
		auto timer_callback = [this]() -> void {
            if (ref_ptr->GetPath()) {
                for (int i = 0; i < ref_ptr->path.size(); i++) {
                    ref.push_back(ref_ptr->path[i][0]);
                    ref.push_back(ref_ptr->path[i][1]);

                    if (!mpc_ptr->MPCSolver(ref, sta_local)) {
                        RCLCPP_WARN(this->get_logger(), "solve fail, now shutdown");
                        rclcpp::shutdown();
                    }
                }

                RCLCPP_INFO(this->get_logger(), "solve finish, now shutdown");
                rclcpp::shutdown();
            }
            else {
                RCLCPP_WARN(this->get_logger(), "get ref data fail, now shutdown");
                rclcpp::shutdown();
            }
		};
		timer_ = this->create_wall_timer(50ms, timer_callback);
	}
private:
	rclcpp::TimerBase::SharedPtr timer_;
	std::atomic<uint64_t> timestamp_;  

    std::string pkg_share = ament_index_cpp::get_package_share_directory("mpc_ros2_pkg");
    std::unique_ptr<GetRefPath> ref_ptr = std::make_unique<GetRefPath>(pkg_share+"/config/ref.csv");
    std::unique_ptr<MpcCal> mpc_ptr = std::make_unique<MpcCal>();

    VectorXd sta_local = VectorXd::Zero(STANUM);
    vector<double> ref;
    
    int index = 0;
};


int main(int argc, char *argv[]) {
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MpcTestNode>());
	rclcpp::shutdown();
	return 0;
}
