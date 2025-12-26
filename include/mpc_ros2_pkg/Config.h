#pragma once

constexpr auto N = 20; //预测步长

constexpr auto STANUM = 6; //状态数量
constexpr auto CONNUM = 2; //输入数量

constexpr auto DT = 0.01;
constexpr auto DTT = DT*DT;
constexpr auto DTTT = DT*DT*DT;

constexpr auto COLS = 803;
constexpr auto ROWS = 2;

constexpr auto CONSROWNUM = N; //约束数量