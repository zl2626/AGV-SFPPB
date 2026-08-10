# AGV-SFPPB

基于 MATLAB/Simulink 的 AGV 横向控制复现工程。项目以原始 `AGV_TFS` 为基础，正在研究“滑模柔性规定性能边界（SFPPB）+ Identifier–Critic–Actor 强化学习”在二维 AGV、单转向输入系统中的推广。

> 当前版本已经进入二维 HJB/Actor 接口验证阶段，但尚未通过 20 s 全程仿真，也不声称已经完成论文级稳定性证明或结果复现。

## 当前版本：四维 Bellman 闭合的二维 z2—单输入 HJB 原型

上一轮 `P=diag([3,1])` 加权投影实验已经完成诊断任务：尺度闭合前后都在约 7.06 s 失效，且提高 y 权重后失效通道由 y 转移到 phi。这支持“把二维 y/phi 信息压缩为单个 `z2_control` 会丢失通道信息”的判断，但固定 P 只能人工分配控制权，不是最终方案。

当前版本正式移除了 `P`、`C_gain`、`z2_control` 和标量 `c2` 控制项，保留

```text
z2 = [z2_y; z2_phi],    delta in R
```

并使用

```text
delta = -(1/(2r))*C'*grad(J_actor)
```

直接构造二维状态到单转向输入的策略。

## 当前完成情况

- `AGV_transfor.m` 与 `AGV_simulate.slx` 已冻结，本轮没有修改第一层 SFPPB、`Gamma`、`D1`、`sigma_ni`、`alpha_1` 或 Simulink 连线。
- `AGV_ctrl.m` 保持原有 13 输入和 3 输出接口，内部连续状态改为 38 个：
  - 二维 Identifier 权重 `WF in R^(9x2)`，共 18 个状态；
  - Critic 权重 `Wc in R^9`；
  - Actor 权重 `Wa in R^9`；
  - 二维输入饱和辅助状态 `O2 in R^2`。
- Identifier 与 Critic/Actor 已拆分特征：

```matlab
Z_F = [e_y; e_phi; de_y; de_phi];
Z_J = [s1_y; s1_phi; z2_bar_y; z2_bar_phi];
```

- `AGV_RBF.m` 现在提供两套简单的 9 节点 RBF：
  - `phi_F` 使用车辆跟踪状态，供二维 Identifier 使用；
  - `phi_J` 使用 `s1 + z2_bar`，供 Critic/Actor 使用；
  - 同时返回 `dphi_J/dZ_J`，使值函数对二维 `z2_bar` 的梯度可以解析计算。
- 当前 HJB 特征状态及其估计动力学已经闭合为：

```text
X_H     = [s1; z2_bar]
s1_dot  = -C1*s1 + D1*Sigma*(z2_bar+O2)
z2_bar_dot_hat = F_hat+C*delta+O2
X_H_dot = [s1_dot; z2_bar_dot_hat]
```

其中 `Sigma=diag([u(1),u(4)])`、`D1=diag([2,1.5])`、`C1=diag([2,22])`，全部信号来自现有 13 路输入，不修改 Simulink。
- 值函数采用一个等权二次初始项和 RBF 修正：

```text
grad_XH(J_critic) = [0;0;0.04*z2_bar] + (dphi_J/dX_H)'*Wc
grad_z2(J_actor)  = 0.04*z2_bar + (dphi_J/dz2_bar)'*Wa
```

- 即时性能指标和 Critic regressor 已改为：

```text
L = s1'*s1 + z2_bar'*z2_bar + r*delta^2
omega_c = (dphi_J/dX_H)*X_H_dot
epsilon_H = L + grad_XH(J_critic)'*X_H_dot
```

- Critic 使用上述完整四维、归一化连续时间 Bellman 残差更新，而不是原来的纯衰减式或漏掉 `s1_dot` 的更新；Actor 暂时仍采用原型式权重跟踪。
- 输入约束辅助状态推广为

```text
O2_dot = -O2 + C*(sat(delta)-delta)
z2_bar = z2-O2
```

其中 `sat(delta)` 与实际送给车辆模型的第二路控制器输出一致。
- `AGV_plant.m`、`assist1.m`、`AGV_plot.m` 和所有外部端口保持不变。

## 已完成的检查

- MATLAB S-Function 接口检查通过：`AGV_transfor` 仍为 5 状态、21 输出、6 输入；`AGV_ctrl` 为 38 状态、3 输出、13 输入。
- `AGV_ctrl` 与两套 `AGV_RBF` 的直接调用均返回有限数值。
- `AGV_ctrl.m` 和 `AGV_RBF.m` 的 MATLAB 静态检查均为 0 个问题。
- `phi_J` 解析 Jacobian 已用中心有限差分核对，最大误差约 `6.01e-11`。
- 未修改 Simulink 模型、车辆模型和冻结的 `AGV_transfor.m`。

## 最新仿真结果

当前四维 Bellman 闭合版本仍未通过 20 s 仿真，约在 **t=1.8949 s** 因 SFPPB 变换接近奇异而终止。

在最后安全检查点 `t=1.8948 s`：

- `e_y=0.0168087`，y 上边界为 `0.0168219`，剩余裕量约 `1.33e-5`；
- phi 通道仍有约 `1.08e-2` 边界裕量，最终失效通道是 y；
- `delta≈-0.07776 rad`，临界阶段全程范围约 `[-0.098,0.0325] rad`；
- 饱和前后控制量完全一致，没有触发 `0.5 rad` 输入限制，因此 `O2` 仍为零；
- 全部 Identifier–Critic–Actor 权重范数由 0 增长到约 `0.18339`，比漏项版本的约 `0.07081` 明显增大。

在便于读取完整内部状态的 `t=1.8945 s`：

- `s1≈[15.8353,-0.1097]^T`，`z1≈[7.9136,0.1364]^T`，`z2≈[0.11177,-0.03055]^T`；
- `||WF||≈0.03824`、`||Wc||≈0.15530`、`||Wa||≈0.08967`；
- seed 控制分量约 `-0.03729 rad`，Actor 修正约 `-0.05950 rad`，Actor 已不再只是很小的附加项；
- `sigma_y≈2.25e4`，导致 `s1_dot_y≈5.00e3`，完整 Critic regressor 范数约 `504.86`；
- 即时成本约 `250.80`，Bellman 残差约 `276.36`。在当前 `(1+omega'omega)^2` 归一化下，对应 Critic 更新范数仅约 `1.61e-6`，近边界的大 regressor 反而被强烈压缩。

因此，这一轮已经修复了 `phi_J(s1,z2_bar)` 与 Bellman residual 不一致的数学缺项。失效时间只从约 1.89121 s 变为约 1.8949 s，但学习行为明显改变；当前不能再把问题简单归结为“Bellman 漏项”，新的直接数值瓶颈是 SFPPB 奇异尺度、巨大 Critic regressor 与平方归一化共同造成的更新尺度失衡。

## 历史诊断结果

| 版本 | 失效时间 | 首要现象 |
| --- | ---: | --- |
| 修正前版本 | 约 1.563 s | phi 接近边界，未饱和转角一度约 49.38 rad |
| 接口局部修正版 | 约 7.061 s | phi 接近 0.005 上界 |
| nominal Gamma | 约 2.52 s | y 接近上边界 |
| D1 尺度修正 | 约 1.92 s | y 接近上边界，普通投影存在通道抵消 |
| 加权投影、尺度未闭合 | 约 7.063 s | y 获救，phi 接近上边界；混入约 46% 增益放大 |
| 加权投影、尺度闭合 | 约 7.05982 s | y 仍安全，phi 距上界约 `5.45e-6` |
| 二维 HJB、Bellman 漏 `s1_dot` | 约 1.89121 s | y 接近下边界；四维值函数残差不闭合 |
| 当前四维 Bellman 闭合版本 | 约 1.8949 s | y 接近上边界；regressor 近边界急剧增大 |

这些结果不能表述为“已稳定跟踪”或“已完成论文复现”。

## 当前存在的问题

1. `s1_dot`、四维值函数梯度、`s1` 成本和完整 regressor 已经加入，Bellman 缺项不再是当前版本的问题。
2. `sigma_y` 在性能边界附近急剧增大，使 `s1_dot` 和 `omega_c` 达到很大尺度；当前平方归一化会把 Critic 更新压到很小，需要先重新审查归一化律，而不是直接调大学习率。
3. 等权二次初始值函数仍不是已证明可稳定完整 AGV/SFPPB 闭环的可容许初始策略。
4. 当前二维 Identifier 仍只使用 `[e_y,e_phi,de_y,de_phi]`，尚未覆盖 `alpha_1_dot` 所依赖的全部第一层内部状态；需要验证 `F_hat` 估计质量。
5. Actor 更新目前只是 `Wa` 跟踪 `Wc` 的原型，不等同于论文中的完整特征加权更新律。
6. Level-1 MATLAB S-Function 会产生弃用警告，但这不是本轮数值失效原因，当前为保持原 AGV_TFS 风格暂不迁移接口。

## 下一步工作

1. 不再恢复或搜索固定 `P`、`c2` 和学习率组合。
2. 首先分析 `omega_c` 的归一化形式与 SFPPB 奇异尺度是否匹配，避免在 `omega_c` 很大时把 Critic 更新冻结；本轮不通过改学习率掩盖该问题。
3. 归一化律明确后，再验证 `F_hat` 估计误差，并决定是否扩充 Identifier 输入。
4. 随后再处理可稳定初始策略和更忠实的 Actor 更新律；在 Bellman 闭合后才考虑离线/历史数据训练。
5. 增加 Identifier 估计误差、Bellman 残差、Actor/Critic 分项权重和 `O2` 的诊断输出；外部 3 路 Simulink 接口仍保持不变。
6. 以 20 s 无输出异常、误差始终位于 SFPPB 内、控制输入满足限制作为最低通过条件。

## 运行方式

1. 在 MATLAB 中将本目录设为当前文件夹。
2. 打开并运行 `AGV_simulate.slx`。
3. 仿真结束后运行 `AGV_plot.m` 绘制结果。

当前版本预期会在约 1.895 s 附近暴露上述问题；具体时刻会随求解器步长略有变化。这是四维 Bellman 闭合后的可复现实验，不是最终结果。

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `AGV_transfor.m` | 已冻结的 SFPPB 边界、误差变换、滑模变量和辅助动态 |
| `AGV_ctrl.m` | 38 状态二维 Identifier–Critic–Actor/HJB 控制器 |
| `AGV_RBF.m` | 分离的 Identifier 与 Critic/Actor RBF 特征及解析 Jacobian |
| `AGV_plant.m` | AGV 横向动力学模型 |
| `assist1.m` | 原输入约束辅助 S-Function |
| `AGV_simulate.slx` | 保持不变的 Simulink 模型 |
| `AGV_plot.m` | 仿真结果绘图脚本 |
| `AGV_diagnose.m` | Identifier 与双 Bellman residual 离线诊断脚本 |

## Critic normalization 诊断结果

本轮严格冻结 `gamma_c`、`gamma_a`、`r`、`Q1`、`Q2`、RBF 宽度和初始策略，只增加离线诊断脚本 `AGV_diagnose.m`，并对 Critic 归一化做一次 A/B 对照。

诊断采用仿真保存的完整 S-Function 状态，离线重构

```text
F_true = d(z2-O2)/dt - C*delta - O2
```

这里必须使用控制器命令 `delta`，而不是饱和后的 `delta_applied`；两者在当前未触发饱和的实验中相同，但进入饱和后只有上式与 `O2_dot=-O2+C*(delta_applied-delta)` 一致。

为了降低 `gradient` 首尾单边差分的影响，统计窗口限定为 `t>0.02 s` 且 `t<t_end-0.01 s`。平方归一化基线在 `t_end=1.8945 s` 的结果为：

- 有效窗口 Identifier 残差范数 RMS 约 `0.77028`，最大约 `3.686`
- 原始末端的 `F_hat=[0.04738,-0.04885]`、`F_true=[-0.76773,-0.27967]` 仅保留为单边差分参考，不作为主要评价指标
- `||omega_c||` 最大约 `504.86`
- 临界点 `||dWc||`：平方归一化约 `1.61e-6`，单次归一化约 `0.41055`

诊断脚本现在同时计算两套 Bellman residual：

```text
epsilon_hat  = L + grad(Jc)'*[s1_dot; F_hat+C*delta+O2]
epsilon_data = L + grad(Jc)'*[s1_dot; d(z2-O2)/dt]
```

有效窗口的比较结果为：

- `RMS(|epsilon_hat|) = 32.39495`
- `RMS(|epsilon_data|) = 32.39869`
- `RMS(|epsilon_hat-epsilon_data|) = 0.043685`
- residual 差值只占 `epsilon_hat` RMS 的约 `0.13485%`，最大绝对差约 `0.086313`

因此 Identifier 的点对点动力学误差虽然明显，但沿当前值函数梯度投影后，对 Bellman residual 的影响很小。现阶段应优先研究 SFPPB 奇异尺度进入 `s1_dot` 和 `omega_c` 后造成的 Critic 数值条件问题，而不是立即扩充 Identifier。

`AGV_diagnose()` 默认绘制 `epsilon_hat`、`epsilon_data` 及其差值曲线；自动检查时可用 `AGV_diagnose(1.8945,false)` 禁用绘图。

单次归一化仍只作为受控失败对照，不作为主线控制律：

```matlab
% A: baseline
dWc = -gamma_c*critic_regressor*bellman_error/critic_normalizer^2;

% B: diagnostic only
dWc = -gamma_c*critic_regressor*bellman_error/critic_normalizer;
```

B 方案在安全窗口 `t=0.3 s` 已将最大 Critic 更新范数从约 `0.134` 放大到约 `0.703`，继续仿真时在约 `t=0.339474 s` 触发求解器困难并导致 `AGV_transfor` 输出异常；它没有改善稳定性。因此当前 `main` 已恢复 A 方案，20 s 仿真仍在约 `1.8949 s` 失败。

当前结论是：Critic 平方归一化确实存在近边界冻结，但简单删除一个平方又过于激进；下一步应研究数值条件更好的 SFPPB/HJB 状态坐标或有界 regressor，而不是继续搜索归一化指数。当前不应调学习率、扩 Identifier、修改 Actor、修改 `AGV_transfor.m` 或改变 Simulink 接口。

