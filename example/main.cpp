#include "../include/rk4.hpp"
#include <cstdio>
#include <cstdlib>

int main() {
    // 參數
    const int    n      = 3;      // 狀態維度
    const double dt     = 0.01;   // 步長
    const double t_end  = 10.0;    // 模擬終止時間
    double x0[n] = {0.0, 1.5, -1.2}; // 初始狀態

    // 建立求解器並模擬
    RK4 solver(n, dt);
    // solver.simulate(t_end, example_deriv_func, x0);
    solver.simulate(t_end, example_deriv_func_simple, x0);

    // 輸出到檔案（CSV: t,x1,x2,...,xn）
    std::FILE* fp = std::fopen("traj.txt", "w");
    if (!fp) return 1;

    std::fprintf(fp, "t");
    for (int j = 0; j < n; ++j) std::fprintf(fp, " x%d", j + 1);
    std::fprintf(fp, "\n");

    for (int i = 0; i < solver.data_length; ++i) {
        std::fprintf(fp, "%.8f", solver.data_t[i]);
        const double* row = solver.data_x + i * n;
        for (int j = 0; j < n; ++j) std::fprintf(fp, " %.12f", row[j]);
        std::fprintf(fp, "\n");
    }

    std::fclose(fp);
    return 0;
}