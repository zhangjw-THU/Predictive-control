%% 单容积分对象基于阶跃响应的输入输出模型的预测控制算法
clear all;
close all;

% 原模型
num = [1];
den = [120,0];
sys=tf(num,den);%模型传递函数
figure(1)
subplot(2,1,1)
step(num,den);
title('阶跃响应');
xlabel('time');

% 进行预测控制
G = poly2tfd(1,[120 0]);%模型的MPC格式
T = 20; % 采样周期
Nt = 1;%输出的稳定性向量
TFinal = 1000; % 控制时间
Model = tfd2step(TFinal,T,Nt,G);%单位阶跃响应
PreModel = Model;%预测控制器

P = 10;%预测长度
M = 2;%控制长度
Ywt = [];
Uwt = 1;
Dkmpc = mpccon(PreModel,Ywt,Uwt,M,P);%计算预测控制器 增益矩阵
tend = 12000;
r = 1;
[y,u,ym] = mpcsim(PreModel,Model,Dkmpc,tend,r);
t = 0:20:12000;
% figure(1)
subplot(2,1,2)
plot(t,y);
title('预测控制输出(system response)');
xlabel('time');

figure(2)
subplot(2,1,1)
plot(t,ym);
title('ym (model response)');
xlabel('time');
subplot(2,1,2)
plot(t,u);
title('u(manipulated variable)');
xlabel('time');
