#include "mpc_ros2_pkg/GetRefPath.h"

GetRefPath::GetRefPath(string file_name) {
    this->file_name = file_name;
    path.clear();
    cout << "get file:" << file_name << " success" << endl;
}

bool GetRefPath::GetPath() {
    ifstream file(file_name);

    vector<vector<double>> rawData;
    string line;

    if (!file.is_open()) {
        cerr << "open data error" << endl;
        return false;
    }

    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<double> row;
        row.reserve(COLS);
        while (getline(ss, cell, ',')) {
            if (!cell.empty())
                row.push_back(stod(cell));
        }
        rawData.push_back(row);
    }

    if (rawData.size() == ROWS && rawData[0].size() == rawData[1].size()) {
        size_t cols = rawData[0].size(); 
        path.reserve(cols);

        for (size_t i = 0; i < cols; ++i) {
            path.push_back({ rawData[0][i], rawData[1][i] });
        }

        if (path.size() < COLS) {
            return false;
        }

        cout << "path get finish," << path.size() << " points" << endl;
        return true;
    }
    else {
        return false;
    }
}