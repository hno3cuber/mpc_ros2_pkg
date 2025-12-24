#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "mpc_ros2_pkg/Config.h"

using namespace std;

class GetRefPath {
public:
	GetRefPath(string file_name);
	bool PathData();
	vector<vector<double>> path;
private:
	string file_name;
};