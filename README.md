# 🧮 RK4 Numerical Simulation 教學與範例比較

本專案示範如何使用 **C++ 實作的 Runge–Kutta 四階法 (RK4)** 進行動態系統數值模擬，  
並在 **MATLAB** 中比對解析解與 `lsim()` 結果。  

## 目錄
- [🧮 RK4 Numerical Simulation 教學與範例比較](#-rk4-numerical-simulation-教學與範例比較)
  - [目錄](#目錄)
  - [專案介紹](#專案介紹)
  - [環境需求](#環境需求)
    - [C++ 端](#c-端)
    - [MATLAB 端](#matlab-端)
  - [資料夾結構](#資料夾結構)
  - [函示庫使用重點](#函示庫使用重點)
  - [MATLAB 比對教學](#matlab-比對教學)
    - [一階系統結果](#一階系統結果)
    - [二階系統結果](#二階系統結果)
  - [作者資訊](#作者資訊)



## 專案介紹

這個專案提供一個簡潔的 **RK4 數值積分器類別** `RK4`，  
可模擬任意一階常微分方程系統：

$$
\dot{x} = f(t, x)
$$

只需定義微分方程函式 `f()`，並設定步長 `dt` 與模擬時間 `t_end`，  
即可得到時間序列資料 `t`, `x(t)`，並匯出到文字檔供 MATLAB 繪圖與分析。

---

## 環境需求

### C++ 端
- 支援 C++17 的編譯器  
  - Windows: [MSYS2 MinGW-w64](https://www.msys2.org/) 或 Visual Studio  
  - macOS: Xcode Command Line Tools  
  - Linux: `sudo apt install g++`
- 建議使用 [Visual Studio Code](https://code.visualstudio.com/)  
  並安裝以下擴充功能：
  - ✅ **C/C++ (Microsoft)**
  - ✅ **Code Runner**

### MATLAB 端
- MATLAB R2020 或以上版本
- 具備 Control System Toolbox (for `tf`, `lsim`, `ss`, etc.)


## 資料夾結構

RK4/
├── include/
│ └── rk4.hpp # RK4 類別定義與實作
├── example/
│ ├── main.cpp # 範例主程式
│ ├── traj.txt # C++ 模擬輸出結果 (執行後自動生成)
│ └── C_Numerical_simulation.m # MATLAB 比對腳本
└── README.md # 本教學文件

## 函示庫使用重點
請參考main.cpp # 範例主程式。`rk4.hpp` 提供了一個通用的 **Runge–Kutta 四階數值積分器**，  
可用來模擬任意一階常微分方程組 (ODE system)。  
以下整理使用時需要注意的重點：


```cpp
RK4 solver(int state_size, double dt);
```
* state_size:狀態變數的維度 (例：一階系統 = 1，二階系統 = 2)
* dt:模擬時間步長

建立後會自動配置內部記憶體（x, k1~k4, x_temp 等暫存陣列）。

```cpp
void deriv_func(int n, double t, double* x, double* dx);
```
* n:狀態維度 (與建構時相同)
* t:當前時間 (s)
* x:狀態向量 
* dx:狀態微分

使用 `simulate()` 自動積分到指定時間：
```
solver.simulate(t_end, deriv_func, x0);
```
* t_end:模擬結束時間 (s)
* deriv_func:使用者定義的微分方程函式指標
* x0:初始狀態陣列

內部會自動從 t=0 開始，以固定步長 dt 積分至 t_end。模擬完成後，結果會存在兩個公開成員：
* double* data_t	時間序列 (大小 = 步數)	
* double* data_x	狀態序列 (展開成一維陣列)	
* int data_length	儲存的資料筆數 (對應時間軸長度)

會在解構子 (~RK4()) 自動釋放動態記憶體。一般情況下不需額外 delete，只要物件離開作用域即可。

## MATLAB 比對教學
* 執行 C_Numerical_simulation.m 。
* 會自動讀取 traj.txt
* 產生一階系統與二階 MCK 系統

比對：
* MATLAB 解析解（Analytic）
* MATLAB 數值模擬 (lsim)
* C++ RK4 結果

### 一階系統結果
<img src=example/figure/F_step_response.png width="400">
<img src=example/figure/F_step_response_error.png width="400">



### 二階系統結果
<img src=example/figure/S_response.png width="400">
<img src=example/figure/S_response_error.png width="400">


## 作者資訊

Author: Chih-Chia Chen

Date: October 2025

Purpose: 教學用數值積分與 MATLAB 比對範例

Language: C++ + MATLAB