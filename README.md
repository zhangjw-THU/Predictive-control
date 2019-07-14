# **基于单容水箱的预测控制算法仿真**

[张嘉玮][http://github.com/zhangjw-THU]

+ 算法描述

  本次仿真实验采用先进控制算法之预测控制对水箱进行控制和仿真实验。算法的原理老师上课已经进行了详细的简述，这里不再赘述，其基本的环节如下图所示。

![img](file:///C:\Users\zjw\AppData\Local\Temp\ksohtml21796\wps1.png)

​	预测控制算法主要通过模型预测，反馈校正以及滚动优化三个步骤进行预测控制，使得模型输出能在预期的轨迹上运行。

该算法具有较好的鲁棒性，能够适应延迟、非最小相位等那难于控制的系统。

***

目录

[1. 算法描述	4](#_Toc20070_WPSOffice_Level1 )

[2. 单容水槽的预测控制	5](#_Toc22256_WPSOffice_Level1 )

[(1) 建模	5](#_Toc22256_WPSOffice_Level2 )

[(2) 仿真	6](#_Toc25107_WPSOffice_Level2 )

[(3) 基于单位阶跃模型的预测控制	7](#_Toc25443_WPSOffice_Level2 )

[① 仿真	7](#_Toc22256_WPSOffice_Level3 )

[② 对预测步长P的探究	9](#_Toc25107_WPSOffice_Level3 )

[③ 对控制步长M的探究	11](#_Toc25443_WPSOffice_Level3 )

[(4) 基于状态空间模型的预测控制	13](#_Toc25720_WPSOffice_Level2 )

[① 仿真	13](#_Toc25720_WPSOffice_Level3 )

[② 对预测步长P的探究	15](#_Toc19102_WPSOffice_Level3 )

[③ 控制步长M的探究	16](#_Toc12971_WPSOffice_Level3 )

[3. 单容积分水槽的预测控制	18](#_Toc25107_WPSOffice_Level1 )

[(1) 建模	18](#_Toc19102_WPSOffice_Level2 )

[(2) 仿真	18](#_Toc12971_WPSOffice_Level2 )

[(3) 基于单位阶跃模型的预测控制	19](#_Toc26361_WPSOffice_Level2 )

[(4) 基于状态空间模型的预测控制	20](#_Toc4319_WPSOffice_Level2 )

[4. 比较分析	21](#_Toc25443_WPSOffice_Level1 )

[5. 总结	22](#_Toc25720_WPSOffice_Level1 )

[6. 参考文献	22](#_Toc19102_WPSOffice_Level1 )

[[1\]. 过程控制系统 黄德先、王京春、金以慧	22](#_Toc23083_WPSOffice_Level2 )

[[2\]. 基于MATLAB的预测控制系统设计 魏源	22](#_Toc5880_WPSOffice_Level2 )

[[3\]. 基于状态空间模型的广义预测控制快速算法 梁晓明	22](#_Toc3439_WPSOffice_Level2 )

[[4\]. 课件 黄德先	22](#_Toc32419_WPSOffice_Level2 )

[7. 附录【代码】	22](#_Toc12971_WPSOffice_Level1 )

[(1) 单容、基于阶跃响应	22](#_Toc23470_WPSOffice_Level2 )

[(2) 单容、基于阶跃响应、对P的探究	24](#_Toc26443_WPSOffice_Level2 )

[(3) 单容、基于阶跃响应、对M的探究	26](#_Toc32295_WPSOffice_Level2 )

[(4) 单容、基于状态空间	29](#_Toc26288_WPSOffice_Level2 )

[(5) 单容、基于状态空间、对P的探究	30](#_Toc22315_WPSOffice_Level2 )

[(6) 单容、基于状态空间、对M的探究	31](#_Toc13093_WPSOffice_Level2 )

[(7) 单容积分、基于阶跃响应	33](#_Toc21481_WPSOffice_Level2 )

[(8) 单容积分、基于状态空间	34](#_Toc23739_WPSOffice_Level2 )

***

```matlab
%% 单容、基于阶跃响应
%% 单容对象基于阶跃响应的输入输出模型的预测控制算法
clear all;
close all;
 
% 原模型
num = [10];
den = [1200,1];
sys=tf(num,den);%模型传递函数
figure(1)
subplot(2,1,1)
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
```



```matlab
%% 单容对象基于阶跃响应的输入输出模型的预测控制算法
% 对预测步长P的探究
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
 
P = [5,10,15,20];%预测长度
M = 2;%控制长度
Ywt = [];
Uwt = 1;
Dkmpc1 = mpccon(PreModel,Ywt,Uwt,M,P(1));%计算预测控制器 增益矩阵
Dkmpc2 = mpccon(PreModel,Ywt,Uwt,M,P(2));%计算预测控制器 增益矩阵
Dkmpc3 = mpccon(PreModel,Ywt,Uwt,M,P(3));%计算预测控制器 增益矩阵
Dkmpc4 = mpccon(PreModel,Ywt,Uwt,M,P(4));%计算预测控制器 增益矩阵
 
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
legend('P=5','P=10','P=15','P=20');
title('预测控制输出(system response)');
xlabel('time');
 
figure(3)
% subplot(2,1,1)
plot(t,ym1,t,ym2,t,ym3,t,ym4);
legend('P=5','P=10','P=15','P=20');
title('ym (model response)');
xlabel('time');
 
figure(4)
plot(t,u1,t,u2,t,u3,t,u4);
legend('P=5','P=10','P=15','P=20');
title('u(manipulated variable)');
xlabel('time');

```



```matlab
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
```

```matlab
%% 单容对象基于状态空间的输入输出模型的预测控制算法
clear all;
close all;
 
% 原模型
num = [10];
den = [1200,1];
sys=tf(num,den);%模型传递函数
figure(1)
subplot(2,1,1)
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
 
P = 10;%预测长度
M = 2;%控制长度
Ywt = [];
Uwt = 1;
 
tend = 12000;
r = 1;
[y,u] = scmpc(PreModel,Model,Ywt,Uwt,M,P,tend,r);
t = 0:20:12000;
% figure(1)
subplot(2,1,2)
plot(t,y);
title('预测控制输出(system response)');
xlabel('time');
 
figure(2)
plot(t,u);
title('u(manipulated variable)');
xlabel('time');

```

```matlab
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


%% 单容、基于状态空间、对M的探究
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
 
P = 10;%预测长度
M = [1,2,4,6];%控制长度
Ywt = [];
Uwt = 1;
 
tend = 2000;
r = 1;
[y1,u1] = scmpc(PreModel,Model,Ywt,Uwt,M(1),P,tend,r);
[y2,u2] = scmpc(PreModel,Model,Ywt,Uwt,M(2),P,tend,r);
[y3,u3] = scmpc(PreModel,Model,Ywt,Uwt,M(3),P,tend,r);
[y4,u4] = scmpc(PreModel,Model,Ywt,Uwt,M(4),P,tend,r);
t = 0:20:2000;
 
 
figure(2)
plot(t,y1,t,y2,t,y3,t,y4);
legend('M=1','M=2','M=4','M=6');
title('预测控制输出(system response)');
xlabel('time');
 
figure(3)
plot(t,u1,t,u2,t,u3,t,u4);
legend('M=1','M=2','M=4','M=6');
title('u(manipulated variable)');
xlabel('time');
```

```matlab
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

```

```matlab
%% 单容积分对象基于状态空间的输入输出模型的预测控制算法
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
Model = tfd2mod(T,Nt,G);%单位阶跃响应
PreModel = Model;%预测控制器
 
P = 10;%预测长度
M = 2;%控制长度
Ywt = [];
Uwt = 1;
 
tend = 12000;
r = 1;
[y,u] = scmpc(PreModel,Model,Ywt,Uwt,M,P,tend,r);
t = 0:20:12000;
% figure(1)
subplot(2,1,2)
plot(t,y);
title('预测控制输出(system response)');
xlabel('time');
 
figure(2)
plot(t,u);
title('u(manipulated variable)');
xlabel('time');
```

