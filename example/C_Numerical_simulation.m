clc;clear;close all;
% 模擬標準一階系統1/(Ts +1)的單位步階響應
dt = 0.01;
t_end = 10;
t = 0:dt:t_end;
T = 1.5; % 時間常數

y_analytic = 1 - exp(-1/T*t);

r = ones(1,length(t));%參考輸入
x_0 = 0; %初始狀態

c_data = textread('traj.txt', '' , 'headerlines', 1);  %意味著讀取資料的時候跳過
y_c = c_data(:,2)';

%% lsim
sys_tf = tf(1,[T 1]); % 轉移函數
y_lsim = lsim(sys_tf,r,t,x_0)';% 模擬步階輸入
% 補充:線性系統轉移函數步階響應可以直接用 step 函數如下所示
% y_step = step(sys_tf,t);

fig(1) = figure("Name","F_step_response");
hold on
plot(t,y_lsim,"-b","LineWidth",2);
plot(t,y_analytic,"-.r","LineWidth",2);
plot(t,y_c,"--c","LineWidth",2);
xlim tight;ylim padded;
xlabel("Time(s)");ylabel("y(t)");

legend("lsim","Analytic","c++","Location","best")


fig(2) = figure("Name","F_step_response_error");
subplot(2,1,1)
plot(t,y_analytic-y_lsim,"-b","LineWidth",2);
xlim tight;ylim padded;
ylabel("Analytic-lsim");
subplot(2,1,2)
plot(t,y_analytic-y_c,"-b","LineWidth",2);
xlim tight;ylim padded;
xlabel("Time(s)");ylabel("Analytic-C++");


M=3;C=2;K=1;
A = [ 0      1;
     -K/M  -C/M ];
B = [0; 1/M];
C = [1 0;   % 輸出1：位移 x
        0 1];  % 輸出2：速度 xdot
D = [0; 0];

sys = ss(A,B,C,D);
x_0 = [1.5 -1.2];
y_lsim = lsim(sys,zeros(size(t)),t,x_0)';
y_c = c_data(:,3:4)';

fig(3) = figure("Name","S_response");
subplot(2,1,1)
hold on
plot(t,y_lsim(1,:),"-b","LineWidth",2);
plot(t,y_c(1,:),"-.c","LineWidth",2);
xlim tight;ylim padded;
ylabel("position");
legend("lsim","c++","Location","best")

subplot(2,1,2)
hold on
plot(t,y_lsim(2,:),"-b","LineWidth",2);
plot(t,y_c(2,:),"-.c","LineWidth",2);
ylabel("velocity");
xlim tight;ylim padded;
xlabel("Time(s)");

fig(4) = figure("Name","S_response_error");
subplot(2,1,1)
title("Analytic-C++");
plot(t,y_lsim(1,:)-y_c(1,:),"-b","LineWidth",2);
xlim tight;ylim padded;
ylabel("position");
subplot(2,1,2)
plot(t,y_lsim(2,:)-y_c(2,:),"-b","LineWidth",2);
xlim tight;ylim padded;
ylabel("velocity");
xlabel("Time(s)");


for i = 1:length(fig)
    exportgraphics(fig(i),['.\figure\' fig(i).Name '.png']);
end
