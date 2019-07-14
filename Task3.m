%% 单容对象基于阶跃响应的输入输出模型的预测控制算法
% 对控制规律的探究
clear all;
close all;

% 原模型
num = [10];
den = [1200,1];
sys=tf(num,den);%模型传递函数
figure(1)
% subplot(2,1,1)
step(num,den);
title('阶跃响应');
xlabel('time');

% 进行预测控制
G = poly2tfd(10,[1200 1]);%模型的MPC格式
T = 20; % 采样周期
Nt = 1;%输出的稳定性向量
TFinal = 1000; % 控制时间
Model = tfd2step(TFinal,T,Nt,G);%单位阶跃响应
PreModel = Model;%预测控制器

P = 10;%预测长度
M = [1,2,4,6];%控制长度
Ywt = [];
Uwt = 1;
Dkmpc1 = mpccon(PreModel,Ywt,Uwt,M(1),P);%计算预测控制器 增益矩阵
Dkmpc2 = mpccon(PreModel,Ywt,Uwt,M(2),P);%计算预测控制器 增益矩阵
Dkmpc3 = mpccon(PreModel,Ywt,Uwt,M(3),P);%计算预测控制器 增益矩阵
Dkmpc4 = mpccon(PreModel,Ywt,Uwt,M(4),P);%计算预测控制器 增益矩阵

tend = 2000;
r = 1;

[y1,u1,ym1] = mpcsim(PreModel,Model,Dkmpc1,tend,r);% 预测仿真
[y2,u2,ym2] = mpcsim(PreModel,Model,Dkmpc2,tend,r);% 预测仿真
[y3,u3,ym3] = mpcsim(PreModel,Model,Dkmpc3,tend,r);% 预测仿真
[y4,u4,ym4] = mpcsim(PreModel,Model,Dkmpc4,tend,r);% 预测仿真
t = 0:20:2000;
% subplot(2,1,2)
% plot(t,y1,t,y2,t,y3,t,y4);
% legend('P=5','P=10','P=15','P=20');
% title('预测控制输出(system response)');
% xlabel('time');

figure(2)
plot(t,y1,t,y2,t,y3,t,y4);
legend('M=1','M=2','M=4','M=6');
title('预测控制输出(system response)');
xlabel('time');

figure(3)
% subplot(2,1,1)
plot(t,ym1,t,ym2,t,ym3,t,ym4);
legend('M=1','M=2','M=4','M=6');
title('ym (model response)');
xlabel('time');

figure(4)
plot(t,u1,t,u2,t,u3,t,u4);
legend('M=1','M=2','M=4','M=6');
title('u(manipulated variable)');
xlabel('time');
