# AGV-SFPPB-RL

本工程在原 AGV-TFS 的 Simulink 结构上实现：

\[
\boxed{\text{SFPPB}+\text{PI transformed error}+\text{ICAS-RL}}
\]

控制链为

```text
(e_y,e_phi) -> SFPPB/NMT -> z1 -> s1 -> ICAS-RL -> alpha1
            -> z2 -> s2 -> ICAS-RL -> delta -> sat(delta) -> AGV
```

PI 只重新定义 RL 使用的变换误差，不是第二个方向盘控制器。工程中没有 LQR、Safety Filter、残差 RL 或第二个转向通道。

## 运行

在 MATLAB 当前目录进入本目录后运行：

```matlab
out = sim('AGV_simulate','StopTime','20');
run('AGV_plot.m');
```

`AGV_plot.m` 按原 AGV-TFS 风格生成 13 幅图，Figure 13 是由 `rho_0` 恢复的 U 形参考轨迹和实际 AGV 轨迹。结果图保存在 `fig3`。

## 参数位置

不使用参数文件。所有控制器参数都在 `AGV_ctrl.m` 文件顶部：

```matlab
k1y = 0.10;   k1phi = 0.20;
k2y = 0.01;   k2phi = 0.01;
c1y = 2;      c1phi = 22;
c2y = 2;      c2phi = 5;
Upsilon1 = 0.04;  Upsilon2 = 0.04;
sigma1 = 0.08;    sigma2 = 0.08;
gamma_c1 = 0.004; gamma_c2 = 0.004;
gamma_a1 = 0.012; gamma_a2 = 0.012;
u_d = 0.5;
```

控制器连续状态顺序为

```text
[WF1; WC1; WA1; WF2; WC2; WA2; O; I1; I2]
```

因此状态数为 `12*N+6`。PI 状态满足 `I1_dot=z1`、`I2_dot=z2`，并使用 `s1=z1+K1*I1`、`s2=z2+K2*I2`。

## 输入方向增益

`AGV_ctrl.m` 中显式保留车辆物理输入方向，并直接归一化为控制方向：

```matlab
C_physical = [cf/m; lf*cf/Iz];
C = C_physical/norm(C_physical);
```

这样只保留方向，输入幅值由控制增益调节，代码更接近原 AGV-TFS 的直线写法。

## 结果

基准工况（`u_d=0.5`，20 s）本次结果为：

```text
RMS(e_y)       = 0.061317 m
RMS(e_phi)     = 0.005021 rad
max |delta|    = 0.465310 rad
max |sat(delta)| = 0.465310 rad
max |rho_0|    = 0.002000
W(20 s)        = 3.400500
```

饱和验证工况（将 `AGV_ctrl.m` 顶部 `u_d` 临时改为 `0.3`）为：

```text
max |delta|       = 0.465420 rad
max |sat(delta)|  = 0.300000 rad
RMS(e_y)          = 0.065174 m
RMS(e_phi)        = 0.005011 rad
W(20 s)           = 3.400200
max |rho_0|       = 0.002000
max flexible-bound widening > 0
```

饱和工况的原始仿真数据保存在 `out_sat03.mat`；默认代码已恢复为 `u_d=0.5`。

## 文件结构

- `AGV_ctrl.m`：PI、Identifier、Critic、Actor、O 补偿和唯一方向盘控制律。
- `AGV_transfor.m`：SFPPB 边界、边界导数、NMT 和 `Gamma`。
- `assist1.m`：输入饱和补偿状态 `rho`。
- `AGV_plant.m`：AGV 横向动力学模型。
- `AGV_plot.m`：原有时域结果图和 Figure 13 轨迹图。
- `AGV_simulate.slx`：原模型结构，增加了 PI 诊断记录通道并将 `rho_0` 接入车辆参考曲率输入。
