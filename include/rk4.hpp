
#pragma once
#include <cmath>

// 四階龍格-庫塔法 (Runge-Kutta 4th Order Method) 實作
class RK4 {
public:
    // 建構子，初始化狀態向量大小
    RK4(int set_state_size, double set_dt) : size_of_state(set_state_size), dt(set_dt) {
        x = new double[size_of_state]();
        k1 = new double[size_of_state]();
        k2 = new double[size_of_state]();
        k3 = new double[size_of_state]();
        k4 = new double[size_of_state]();

        x_temp = new double[size_of_state];
    }

    // 解算函式，傳入微分方程的函式指標
    void solve(void (*deriv_func)(int, double, double*, double*), double* initial_state) {
        // 初始化狀態向量
        for (int i = 0; i < size_of_state; ++i) {
            x[i] = initial_state[i];
        }

        // 計算 k1
        deriv_func(size_of_state, t, x, k1);

        // 計算 k2
        
        for (int i = 0; i < size_of_state; ++i) {
            x_temp[i] = x[i] + 0.5 * dt * k1[i];
        }
        deriv_func(size_of_state, t + 0.5 * dt, x_temp, k2);

        // 計算 k3
        for (int i = 0; i < size_of_state; ++i) {
            x_temp[i] = x[i] + 0.5 * dt * k2[i];
        }
        deriv_func(size_of_state, t + 0.5 * dt, x_temp, k3);

        // 計算 k4
        for (int i = 0; i < size_of_state; ++i) {
            x_temp[i] = x[i] + dt * k3[i];
        }
        deriv_func(size_of_state, t + dt, x_temp, k4);

        // 更新狀態向量
        for (int i = 0; i < size_of_state; ++i) {
            x[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }

        // 更新時間
        t += dt;
    }

    // 模擬到 t_end
    void simulate(double t_end, void (*deriv_func)(int, double, double*, double*), double* initial_state) {
        // 初始化狀態向量
        for (int i = 0; i < size_of_state; ++i) {
            x[i] = initial_state[i];
        }

        t = 0.0;
        int idx_max = (int)round(t_end / dt)+1;
        data_length = idx_max;
        data_x = new double[idx_max * size_of_state];
        data_t = new double[idx_max];
        for (int i = 0; i < idx_max; i++)
        {   
            data_t[i] = t;
            for (int j = 0; j < size_of_state; j++)
            {
                data_x[i * size_of_state + j] = x[j];
            }
            solve(deriv_func, x); 
        }
    }

    // 解算函式，傳入微分方程的函式指標 跟上面的比較簡化版 沒有保證n參數
    void solve(void (*deriv_func)(double, double*, double*), double* initial_state) {
        // 初始化狀態向量
        for (int i = 0; i < size_of_state; ++i) {
            x[i] = initial_state[i];
        }

        // 計算 k1
        deriv_func(t, x, k1);

        // 計算 k2
        
        for (int i = 0; i < size_of_state; ++i) {
            x_temp[i] = x[i] + 0.5 * dt * k1[i];
        }
        deriv_func( t + 0.5 * dt, x_temp, k2);

        // 計算 k3
        for (int i = 0; i < size_of_state; ++i) {
            x_temp[i] = x[i] + 0.5 * dt * k2[i];
        }
        deriv_func( t + 0.5 * dt, x_temp, k3);

        // 計算 k4
        for (int i = 0; i < size_of_state; ++i) {
            x_temp[i] = x[i] + dt * k3[i];
        }
        deriv_func( t + dt, x_temp, k4);

        // 更新狀態向量
        for (int i = 0; i < size_of_state; ++i) {
            x[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }

        // 更新時間
        t += dt;

    }

    // 模擬到 t_end 跟上面比較簡化版 沒有保證n參數
    void simulate(double t_end, void (*deriv_func)(double, double*, double*), double* initial_state) {
        // 初始化狀態向量
        for (int i = 0; i < size_of_state; ++i) {
            x[i] = initial_state[i];
        }

        t = 0.0;
        int idx_max = (int)round(t_end / dt)+1;
        data_length = idx_max;
        data_x = new double[idx_max * size_of_state];
        data_t = new double[idx_max];
        for (int i = 0; i < idx_max; i++)
        {   
            data_t[i] = t;
            for (int j = 0; j < size_of_state; j++)
            {
                data_x[i * size_of_state + j] = x[j];
            }
            solve(deriv_func, x); 
        }
    }

    // 取得當前狀態向量
    double* get_state() {
        return x;
    }

    // 取得當前時間
    double get_time() {
        return t;
    }

    // 釋放資源
    ~RK4() {
        delete[] x;
        delete[] k1;
        delete[] k2;
        delete[] k3;
        delete[] k4;
        delete[] x_temp;
        delete[] data_x;
    }

    double* data_x = nullptr;
    double* data_t = nullptr;
    int data_length = 0;
private:
    double step = 0.01;  // 預設步長
    double t = 0.0;    // 當前時間
    double dt = 0.01;   // 時間增量
    double* x = nullptr; // 狀態向量
    double* x_temp = nullptr; // 臨時狀態向量
    double* k1 = nullptr; // 中間變量 k1
    double* k2 = nullptr; // 中間變量 k2
    double* k3 = nullptr; // 中間變量 k3
    double* k4 = nullptr; // 中間變量 k4
    int size_of_state = 0;      // 狀態向量維度
};

// deriv_func 範例
void example_deriv_func(int n, double t, double* x, double* dx) {
    // 假設一個簡單的微分方程 dx/dt = -x
    // for (int i = 0; i < n; ++i) {
    //     dx[i] = -x[i];
    // }
    double T = 1.5;
    double M=3;
    double C=2;
    double K=1;

    //step input of first order system
    dx[0] = -1/T*x[0] + 1/T;

    // MCK system
    dx[1] = x[2];
    dx[2] = -C/M*x[2] - K/M*x[1] ;
}

void example_deriv_func_simple(double t, double* x, double* dx) {
    // 假設一個簡單的微分方程 dx/dt = -x
    // for (int i = 0; i < n; ++i) {
    //     dx[i] = -x[i];
    // }
    double T = 1.5;
    double M=3;
    double C=2;
    double K=1;

    //step input of first order system
    dx[0] = -1/T*x[0] + 1/T;

    // MCK system
    dx[1] = x[2];
    dx[2] = -C/M*x[2] - K/M*x[1] ;
}
