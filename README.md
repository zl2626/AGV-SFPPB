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

`AGV_plot.m` 按原 AGV-TFS 风格生成 13 幅图，Figure 13 是由 `rho_0` 恢复的曲线路径和实际 AGV 轨迹。结果图保存在 `fig3`。

主模型 `AGV_simulate.slx` 保留 20 s 小曲率基准工况。真正的 U 形路径验证放在 `AGV_simulate_U.slx`：

```matlab
out = sim('AGV_simulate_U','ReturnWorkspaceOutputs','on');
run('AGV_plot.m');
```

U 形工况使用 `v_x=20 m/s`、`rho_0=0.002`，在 `10~88.54 s` 保持曲率，航向变化约为
`20*0.002*(88.54-10)=pi`，因此是完整的 180° 半圆掉头；总仿真时间为 100 s。对应结果图命名为
`fig3/sfppb_pi_u_01~13`。之所以采用较长的半圆时间，是为了保持当前 SFPPB 边界和控制器不变；在 20 s 内直接使用 `rho_0=0.02` 会造成边界越界。

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

第二层控制量明确使用 PI 导数项：

```matlab
F2_PI = F2_hat + K2.*z2;
p_a2 = 2*C2.*s2 + 2*F2_PI + WA2'*S_J2;
```

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
RMS(e_y)       = 0.060814 m
RMS(e_phi)     = 0.004978 rad
max |delta|    = 0.464725 rad
max |sat(delta)| = 0.464725 rad
max |rho_0|    = 0.002000
max |rho|      = 0
J_delta        = 3.29334
W(20 s)        = 3.400336
```

饱和验证工况（将 `AGV_ctrl.m` 顶部 `u_d` 临时改为 `0.3`）为：

```text
max |delta|       = 0.464989 rad
max |sat(delta)|  = 0.300000 rad
RMS(e_y)          = 0.062300 m
RMS(e_phi)        = 0.005175 rad
W(20 s)           = 3.399705
max |rho_0|       = 0.002000
max |rho|         = 0.015386
max flexible-bound widening = 0.007692
J_delta           = 3.49859
sat time          = 1.098 s
```

默认代码已恢复为 `u_d=0.5`。`AGV_plot.m` 的 Figure 12 现在绘制真正的柔性辅助状态 `rho`，Figure 10 单独绘制道路曲率 `rho_0`。

## 文件结构

- `AGV_ctrl.m`：PI、Identifier、Critic、Actor、O 补偿和唯一方向盘控制律。
- `AGV_transfor.m`：SFPPB 边界、边界导数、NMT 和 `Gamma`。
- `assist1.m`：输入饱和补偿状态 `rho`，同时输出 `rho_dot` 给 SFPPB。
- `AGV_plant.m`：AGV 横向动力学模型。
- `AGV_plot.m`：时域结果图、`rho` 柔性状态图和 Figure 13 曲线路径图。
- `AGV_simulate.slx`：原模型结构，增加了 PI 诊断记录通道、`rho/rho_dot` 记录通道，并将 `rho_0` 接入车辆参考曲率输入。
- `AGV_simulate_U.slx`：真正 U 形路径验证模型，使用 100 s 仿真和 180° 半圆参考曲率。
