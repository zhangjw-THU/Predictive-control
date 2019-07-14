%% 单容对象基于状态空间的输入输出模型的预测控制算法
clear all;
close all;

% 原模型
num = [10];
den = [1200,1];
sys=tf(num,den);%模型传递函数
figure(1)
step(num,den);
title('阶跃响应');
xlabel('time');

% 进行预测控制
G = poly2tfd(10,[1200 1]);%模型的MPC格式
T = 20; % 采样周期
Nt = 1;%输出的稳定性向量
TFinal = 1000; % 控制时间
Model = tfd2mod(T,Nt,G);%单位阶跃响应
PreModel = Model;%预测控制器

P = [5,10,15,20];%预测长度
M = 2;%控制长度
Ywt = [];
Uwt = 1;

tend = 2000;
r = 1;
[y1,u1] = scmpc(PreModel,Model,Ywt,Uwt,M,P(1),tend,r);
[y2,u2] = scmpc(PreModel,Model,Ywt,Uwt,M,P(2),tend,r);
[y3,u3] = scmpc(PreModel,Model,Ywt,Uwt,M,P(3),tend,r);
[y4,u4] = scmpc(PreModel,Model,Ywt,Uwt,M,P(4),tend,r);
t = 0:20:2000;


figure(2)
plot(t,y1,t,y2,t,y3,t,y4);
legend('P=5','P=10','P=15','P=20');
title('预测控制输出(system response)');
xlabel('time');

figure(3)
plot(t,u1,t,u2,t,u3,t,u4);
legend('P=5','P=10','P=15','P=20');
title('u(manipulated variable)');
xlabel('time');
