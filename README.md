# SFPPB-RL AGV复现说明

本目录是在原 AGV-TFS 工程骨架上改写的版本。AGV_plant.m、
AGV_simulate.slx 和 AGV_plot.m 保持不变，只改了边界、辅助状态、
RBF和控制器。

控制链只有：

SFPPB -> NMT -> z1 -> alpha1 -> z2 -> delta
-> sat(delta) -> AGV。

没有 LQR、CARE、Safety Filter、残差控制器或第二个方向盘通道。
SFPPB只使用一个公共饱和状态 rho。

## 运行

~~~matlab
out = sim('AGV_simulate','StopTime','20');
run('AGV_plot.m');
~~~

这就是原 AGV-TFS 的运行方式：先运行 Simulink 模型，再在工作区执行
AGV_plot.m。AGV_plot.m 本身没有改动。需要保存图片时，在绘图后手动用
savefig 或 print 保存到 fig3 文件夹。
本次按原方式生成的文件前缀为 agtfs_online_。

## 参数位置

不再使用 AGV_rebuild_params.m。调参位置如下：

- AGV_transfor.m 顶部：rho_0_y、rho_T_y、delta_y、
  lambda1_y、lambda2_y 等 SFPPB 参数；
- AGV_ctrl.m 顶部：Q1、R1、Q2、r2、gamma_F1、
  gamma_C1、gamma_A1 和 RBF节点数 N；
- assist1.m 顶部：u_d、m1、m2。

参数名尽量沿用论文公式和原 AGV-TFS 代码，注释使用中文。

## 控制器状态

控制器自适应状态按论文符号排列为：

~~~text
[WF1; WC1; WA1; WF2; WC2; WA2]
~~~

WF1、WF2 是四维物理状态不确定向量场的RBF权重；
WC1、WC2 是Critic权重；WA1、WA2 是Actor权重。
第一层输出二维虚拟控制 alpha1，第二层经过

~~~text
delta = -(1/(2*r2))*C'*grad_z2(J2)
~~~

得到唯一标量方向盘请求。

原始运行方式下的结果是一次确定性的20秒Simulink数值仿真，不包含
Monte Carlo、随机初值或测量噪声。
