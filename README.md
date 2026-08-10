# AGV-SFPPB

基于 MATLAB/Simulink 的 AGV 横向控制研究工程。项目从原始 `AGV_TFS` 演化而来，目标是在二维跟踪误差、单一转向输入和输入约束下，实现 SFPPB（滑模柔性规定性能边界）与 Identifier–Critic–Actor 强化学习控制。

> 当前版本已通过 20 s 主基准，并额外通过 60 s 多扰动周期验证。这里的“通过”只表示当前模型和参数下的数值仿真通过，不等同于已经完成严格稳定性证明或所有工况的论文级验证。

## 当前结果

主模型保持 `StopTime = 20 s`、自动变步长求解器、转向约束 `|delta| <= 0.5 rad`，车辆模型、道路曲率序列、8 s 周期扰动以及 SFPPB 参数均未放宽。

| 指标 | 20 s 主基准 |
| --- | ---: |
| 仿真终止时间 | `20.000000 s` |
| 最小 y 边界裕量 | `0.00425618` |
| 最小 phi 边界裕量 | `0.000730602` |
| 最小归一化边界裕量 | `14.1873%` |
| 最大 Actor 原始转向 `max|delta_Actor|` | `0.470090 rad` |
| 最大安全滤波转向 `max|delta_safe|` | `0.496847 rad` |
| 最大实际转向 `max|sat(delta)|` | `0.496847 rad` |
| 安全滤波介入时间比例 | `6.2362%` |
| 最大安全修正量 | `0.369264 rad` |
| 安全区间冲突次数 | `0` |
| 最大饱和失配 | `0` |
| 最大 `||O2||` | `0` |
| 末端误差 `[e_y,e_phi]` | `[-0.00161726,-2.91e-6]` |
| Critic 末端权重范数 | `0.07756` |
| Actor 相对初值的权重变化 | `0.02443` |

扩展验证结果：

- 自动求解器运行 `60 s`：通过；最小 y/phi 裕量分别为 `0.00250423` 和 `0.000729638`，最大转向 `0.497136 rad`，无饱和失配。
- `ode15s` 运行 `20 s`：通过；边界裕量与自动求解器结果一致，最大转向 `0.496847 rad`。
- `AGV_ctrl.m`、`AGV_diagnose.m`、`AGV_RBF.m` 和 `AGV_plot.m` 的 MATLAB 静态检查均为 0 个问题。
- `AGV_ctrl` 保持 38 个连续状态和 13 个控制输入；输出由 3 个扩展为 8 个，仅增加 Actor、安全滤波和学习状态的诊断日志，没有改变车辆闭环控制通道。

新增日志以一个向量写入 `ctrl_diagnostics`，列顺序为：

```text
[delta_safe, delta_applied, total_weight_norm, delta_Actor,
 ||WF||, ||Wc||, ||Wa-Wa0||, safety_interval_conflict]
```

## 当前控制结构

```text
AGV plant
   -> SFPPB / NMT / first backstepping layer
   -> vector z2 and s1
   -> Identifier + four-dimensional Critic
   -> direct-policy Actor
   -> SFPPB safety projection
   -> steering constraint
   -> AGV plant
```

当前版本不再使用固定 `P`、`C_gain`、`z2_control` 或标量投影控制。二维状态信息始终保留到 Actor 和安全分配层。

### Identifier 与 Critic 状态

```matlab
Z_F = [e_y; e_phi; de_y; de_phi];
X_H = [s1_y; s1_phi; z2_bar_y; z2_bar_phi];
z2_bar = z2 - O2;
```

四维 Bellman 状态导数保持闭合：

```text
s1_dot = -C1*s1 + D1*Sigma*(z2_bar+O2)
z2_bar_dot_hat = F_hat + C*delta + O2
X_H_dot = [s1_dot; z2_bar_dot_hat]
```

Critic 使用 SFPPB 屏障条件初始值函数与 RBF 修正：

```text
J0 = 0.5*k0*sum((1+s1_i^2)*z2_bar_i^2)
Jc = J0 + Wc'*phi_J(X_H)
epsilon_H = L + grad(Jc)'*X_H_dot
```

Critic 采用低增益单次归一化更新和小泄漏项，避免旧版本中平方归一化导致的近边界冻结，也避免直接使用高增益单次归一化导致的暴冲：

```text
dWc = -0.005*omega_c*epsilon_H/(1+omega_c'*omega_c) - 0.0005*Wc
```

### Direct-policy Actor

当前方向盘命令只由 Actor 网络给出，不再采用“原控制器 + residual RL”的叠加形式：

```text
delta_nominal = Wa'*psi_A
```

Actor 特征同时包含归一化车辆状态和有界 SFPPB 状态：

```text
psi_A = [
    e_y/0.03;
    e_phi/0.005;
    de_y/0.5;
    de_phi/0.2;
    tanh(s1_y/5);
    tanh(s1_phi/5);
    tanh(z2_bar_y/0.25);
    tanh(z2_bar_phi/0.25);
    1
]
```

初始 Actor 权重由名义车辆 CARE/LQR 的可容许策略写入网络权重，而不是在 Actor 外另加一个基线控制器。在线更新沿 Hamiltonian 对策略的梯度下降，并用小正则项保持在可容许策略邻域。20 s 仿真中 Actor 权重发生了非零变化，因此当前结果不是冻结学习率得到的纯固定控制。

### SFPPB 安全投影与输入约束

Actor 输出进入一个标量安全可行域投影。投影根据两个误差通道的二阶 SFPPB 安全条件，计算同一个方向盘输入允许的上下界，再把 `delta_nominal` 投影到可行区间和 `[-0.5,0.5]` 的交集。

当前安全层参数为：

```text
terminal_bound = [0.03; 0.005]
lambda = 100
disturbance_bound = [25; 25]
```

`25` 不是按运行时间搜索得到的增益，而是高于车辆模型中“路径曲率项 + 18 幅值脉冲扰动”的统一鲁棒上界。该取值还使两个通道的最小归一化裕量在 20 s 基准中接近平衡。若两个通道的安全区间瞬时冲突，则优先保护归一化剩余裕量最小的通道。

输入约束辅助状态仍保留：

```text
O2_dot = -O2 + C*(sat(delta)-delta)
z2_bar = z2-O2
```

在当前 20 s 和 60 s 结果中，安全投影已使请求转向本身不超过 `0.5 rad`，所以 `O2` 全程为零；它仍用于覆盖未来可能出现的真实饱和失配。

## 诊断结果

运行：

```matlab
report = AGV_diagnose(20, false, 100);
```

当前 20 s 诊断结果：

| 指标 | 数值 |
| --- | ---: |
| `RMS ||F_true-F_hat||` | `14.1510` |
| `max ||omega_c||` | `84.0673` |
| `RMS |epsilon_hat|` | `11.8453` |
| `RMS |epsilon_data|` | `11.6977` |
| Bellman residual gap 比例 | `5.9587%` |
| 最大实际 Critic 更新率 | `0.08078` |
| 安全滤波介入时间比例 | `6.2362%` |
| 安全区间冲突次数 | `0` |
| 控制能量 `integral(delta^2)` | `0.207757` |

Identifier 在脉冲扰动期间仍有明显点对点误差，但模型残差与数据残差的差距约为 Bellman residual RMS 的 5.96%，已不再像旧版本那样与 Critic 数值条件完全混在一起。当前学习率使 Critic 保持真实更新，同时没有复现单次归一化高增益版本在 0.34 s 左右失稳的问题。

安全滤波诊断同时记录 `delta_Actor`、`delta_safe` 和 `delta_applied`。20 s 内安全滤波只在约 6.24% 的时间修改 Actor 输出，且可行区间没有冲突。因此当前结构不是由安全滤波全程替代 Actor；安全层主要在脉冲扰动附近介入。另一方面，当前仿真没有发生执行器饱和，`O2` 仍为零，所以这组结果只能证明输入约束得到满足，不能证明饱和补偿状态在当前工况中发挥了主要作用。

### Safety-filter lambda 受控扫描

保持 SFPPB、扰动上界、Actor/Critic、RBF 和车辆模型全部不变，只在 60 s 工况下扫描安全滤波参数：

| `lambda` | 60 s 状态 | 最小归一化裕量 | `max|delta|` | 滤波介入比例 | 区间冲突次数 | 控制能量 |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 60 | 未通过 | — | — | — | — | — |
| 80 | 通过 | `7.0533%` | `0.484797` | `12.0252%` | 7 | `0.703794` |
| 100 | 通过 | `8.3474%` | `0.497136` | `7.2802%` | 0 | `0.697138` |
| 120 | 通过 | `10.0292%` | `0.500000` | `6.2144%` | 2 | `0.693504` |

因此主线继续保留 `lambda = 100`：它是本轮候选中唯一在 60 s 内没有安全区间冲突的取值。`lambda = 80` 虽降低转向峰值，但边界裕量更小且出现 7 次冲突；`lambda = 120` 达到约 10% 裕量，却触及硬转向上限并出现冲突。该实验也说明，仅调 `lambda` 不能同时达到“归一化裕量大于 10% 且最大转向小于 0.47 rad”的新目标。

## 历史问题如何被解决

| 阶段 | 结果 | 结论 |
| --- | ---: | --- |
| 普通/加权标量投影 | 最好约 `7.06 s` | 固定权重只能在 y 与 phi 之间人工转移风险 |
| 二维 HJB 原型 | 约 `1.891 s` | 值函数状态包含 `s1`，但 Bellman residual 漏掉 `s1_dot` |
| 四维 Bellman 闭合 | 约 `1.895 s` | 方程闭合后暴露 Critic 数值条件和初始策略问题 |
| 单次归一化直接替换 | 约 `0.339 s` | 原学习率下更新过猛 |
| 屏障型标量分配种子 | 约 `7.1 s` | 能通过无扰动段，但单输入通道冲突仍在扰动窗爆发 |
| direct Actor + 安全投影 | `20 s / 60 s` 通过 | 保留二维信息，在线学习，并显式处理单输入安全可行域 |

## 当前仍存在的问题

1. 已通过的 20 s 和 60 s 仿真不是 Lyapunov/安全不变集的完整证明；论文中仍需给出 Actor、Critic、Identifier、SFPPB 与安全投影组合后的统一稳定性论证。
2. 安全投影使用名义 AGV 漂移模型和已知扰动上界 `25`。后续应针对质量、惯量、轮胎侧偏刚度、车速、扰动幅频和测量噪声做参数网格或 Monte Carlo 验证。
3. Identifier 在强脉冲扰动下误差仍大，当前 `Z_F` 尚未覆盖 `alpha1_dot` 的全部内部依赖。它对 Bellman residual 的影响已下降，但仍是后续改进重点。
4. Actor 当前采用 Hamiltonian 策略梯度原型，不等同于参考论文中的逐项 Actor 更新律；后续论文表述必须明确这是面向二维 AGV 单输入系统的推广实现。
5. 项目继续使用原工程的 Level-1 MATLAB S-Function，因此 MATLAB 会给出弃用警告。警告不影响当前结果，但长期维护可考虑迁移到 Level-2。
6. 当前结果尚未形成完整对照实验、消融实验和统计置信区间，不能直接宣称达到最终论文结论。

## 运行方法

1. 在 MATLAB 中将本目录设为当前文件夹。
2. 打开并运行 `AGV_simulate.slx`；模型默认仿真时间为 20 s。
3. 仿真结束后运行 `AGV_plot.m`。脚本生成 5 类论文主图和 1 张附录诊断图，并自动导出矢量 PDF 与 600 dpi PNG 到 `Fig/paper/`。
4. 运行 `AGV_diagnose(20,false,100)` 获取 Identifier、Bellman residual、Actor/安全滤波和权重数值诊断；第三个参数用于受控测试 `lambda`。

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `AGV_simulate.slx` | 保持原有闭环控制连线，并记录 Actor、安全滤波和学习状态 |
| `AGV_plant.m` | AGV 横向动力学、道路曲率和周期脉冲扰动 |
| `AGV_transfor.m` | SFPPB、NMT、第一层反步与二维 `z2` 生成 |
| `AGV_ctrl.m` | Identifier、四维 Critic、direct-policy Actor、安全投影和输入约束 |
| `AGV_RBF.m` | Identifier/Critic RBF 特征及解析 Jacobian |
| `assist1.m` | 原 SFPPB 输入约束柔性边界辅助状态 |
| `AGV_diagnose.m` | Identifier、双 Bellman residual、安全滤波和权重离线诊断 |
| `AGV_plot.m` | 单图单问题科研作图及 PDF/600 dpi PNG 导出 |

本轮没有增加代码文件。Simulink 仅扩展控制器诊断输出和日志，原车辆模型、13 路控制输入以及闭环执行器通道保持不变。
