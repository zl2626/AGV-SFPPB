# SFPPB-RL AGV复现说明

本目录是在原 AGV-TFS 工程骨架上改写的版本。AGV_simulate.slx 和
AGV_plot.m 保持不变，只改了 SFPPB 边界、输入饱和辅助状态、Gaussian
RBF、ICAS-RL 控制器，并核正了 AGV_plant.m 中航向误差矩阵的符号。

控制链为：

SFPPB -> NMT -> z1 -> alpha1 -> O -> z2 -> delta
-> sat(delta) -> AGV。

没有 PI、LQR、CARE、Safety Filter、残差控制器或第二个方向盘通道。
SFPPB 使用一个公共饱和状态 rho；O 是论文输入饱和补偿状态。

## 运行

~~~matlab
out = sim('AGV_simulate','StopTime','20');
run('AGV_plot.m');
~~~

这是原 AGV-TFS 的运行方式：先运行 Simulink 模型，再在工作区执行
AGV_plot.m。AGV_plot.m 本身没有改动。绘图后可用 savefig 或 print
把结果保存到 fig3 文件夹。本次结果文件前缀为 agtfs_online_ 和
rl-online_。

## 参数位置

不使用参数文件，也不增加新的 .m 文件。

- AGV_transfor.m 顶部：kappa0_y、kappaT_y、T_y、l_kappa_y、
  l_s_y、eta_y、lambda1_y、lambda2_y，以及 phi 对应参数；
- assist1.m 顶部：u_d、p1、p2；
- AGV_ctrl.m 顶部：c1、c2、Upsilon1、Upsilon2、sigma1、sigma2、
  gamma_c1、gamma_c2、gamma_a1、gamma_a2。

参数名直接对应论文符号，注释使用中文。

## SFPPB和NMT

对 i=y,phi：

~~~text
kappa_i(t) = kappa0_i + (kappaT_i-kappa0_i)
             * sin(pi*t/(2*T_i))^l_kappa_i

S_i(t) = [1-sin(pi*t/(2*T_i))]^l_s_i
~~~

名义边界根据初始误差正负展开，再加入同一个 rho 的柔性放宽：

~~~text
B_under_i = b_under_i + e0_i*S_i - lambda1_i*tanh(rho)
B_bar_i   = b_bar_i   + e0_i*S_i + lambda2_i*tanh(rho)

z1_i = log((e_i-B_under_i)/(B_bar_i-e_i))

Gamma_i = B_bar_i_dot/(B_bar_i-e_i)
          + B_under_i_dot/(e_i-B_under_i)

z1_dot = varsigma_i*e_i_dot-Gamma_i
~~~

调试阶段不把越界误差夹回边界；真实越界会直接报错。

## 输入饱和辅助状态

assist1.m 直接使用论文展开式：

~~~text
rho_dot = -p1*rho + p2*(varpi1+varpi2)

varpi1 = [sign(delta-u_d)+1]*(delta-u_d)
varpi2 = [sign(delta+u_d)-1]*(delta+u_d)
~~~

## ICAS-RL控制器

自适应状态按论文符号排列为：

~~~text
[WF1; WC1; WA1; WF2; WC2; WA2; O]
~~~

每组权重为 N×2，O=[O_y,O_phi]。Identifier、Critic、Actor 更新律
直接采用论文形式，不使用 Q/R、Bellman residual、gamma_pi、projection
或额外泄漏项。

第一层：

~~~text
alpha1 = varsigma^(-1)
         [-c1*z1 + Gamma - WF1'*S_F1 - 0.5*WA1'*S_J1]
~~~

输入饱和补偿和第二层：

~~~text
O_dot = -O + C*(u_d*tanh(delta/u_d)-delta)
z2 = chi2-alpha1-O

p_a2 = 2*c2*z2 + 2*WF2'*S_F2 + WA2'*S_J2
delta = -0.5*C'*p_a2
~~~

Actor更新律前面使用论文Eq.(46)的负号；六组权重从幅值为 `0.4` 的
确定性对称初值开始，避免Critic/Actor从零初值时永远不学习，同时避免
二维AGV输入增益把初始方向盘请求瞬间放大。
最终只有一个标量方向盘请求 delta。

由于AGV是二维误差、单方向盘输入，控制器内部使用归一化的输入方向
`C/norm(C)`；车辆本体仍使用 `AGV_plant.m` 的物理输入矩阵。

## 车辆矩阵核对

`AGV_plant.m` 使用论文Eq.(13)的航向误差项：

~~~text
a22 = (-lf*cf + lr*cr)/Iz
b12 = (-lf*cf + lr*cr)/(m*vx) - vx
~~~

其中 `-vx` 是论文横向误差动力学中的运动学项，保留不删；修正的是
状态矩阵第四行第二列原先误写的 `-lf*cf-lr*cr`。

## 20秒直接仿真结果

本次在原 Simulink 模型上直接运行，未使用随机初值、测量噪声或
Monte Carlo：

- y 最小边界余量：0.198816；
- phi 最小边界余量：0.071791；
- 请求方向盘最大值：0.459727 rad；
- 实际方向盘最大值：0.459727 rad；
- rho 最大值：0；
- y RMS：0.040965；
- phi RMS：0.004770；
- 末时刻权重范数：3.325515。

结果图片保存在 fig3 文件夹。
