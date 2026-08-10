# AGV-SFPPB

基于 MATLAB/Simulink 的 AGV 横向控制复现工程。项目以原始 `AGV_TFS` 为基础，正在研究“滑模柔性规定性能边界（SFPPB）+ Identifier–Critic–Actor 强化学习”在二维 AGV、单转向输入系统中的推广。

> 当前版本已经进入二维 HJB/Actor 接口验证阶段，但尚未通过 20 s 全程仿真，也不声称已经完成论文级稳定性证明或结果复现。

## 当前版本：二维 z2—单输入 HJB 原型

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
- 值函数采用一个等权二次初始项和 RBF 修正：

```text
grad(J_critic) = 0.04*z2_bar + (dphi_J/dz2_bar)'*Wc
grad(J_actor)  = 0.04*z2_bar + (dphi_J/dz2_bar)'*Wa
```

- Critic 使用归一化连续时间 Bellman 残差更新，而不是原来的纯衰减式更新；Actor 跟踪 Critic 的值函数权重。
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

当前二维 HJB 原型仍未通过 20 s 仿真，约在 **t=1.89121 s** 因 SFPPB 变换接近奇异而终止。

在最后安全检查点 `t=1.891 s`：

- `e_y=-0.1048254`，y 下边界为 `-0.1048438`，剩余裕量约 `1.84e-5`；
- `e_phi=-0.0018447`，距离最近 phi 边界仍约 `5.36e-3`；
- `s1=[-17.611,-2.228]^T`，`z1=[-8.798,-1.204]^T`；
- `z2=[-0.08497,-0.09340]^T`；
- `delta=0.06759 rad`，全程范围约 `[-0.00450,0.06759] rad`；
- 饱和前后控制量完全一致，没有触发 `0.5 rad` 输入限制，因此 `O2` 仍为零；
- 全部 Identifier–Critic–Actor 权重范数由 0 增长到约 `0.07081`，不再表现为旧 Critic 公式下的单纯衰减；
- 临界时等权初始值函数产生约 `0.07268 rad` 控制，Actor 修正仅约 `-0.00509 rad`，在线策略尚未在触边前形成足够强的边界修正能力。

因此，这一轮说明：二维 HJB 的接口、值函数梯度和特征拆分已经落到代码中，但“结构正确”尚不等于“在线策略已经稳定”。当前主要瓶颈已经转移为初始策略与在线学习速度，而不是固定投影尺度。

## 历史诊断结果

| 版本 | 失效时间 | 首要现象 |
| --- | ---: | --- |
| 修正前版本 | 约 1.563 s | phi 接近边界，未饱和转角一度约 49.38 rad |
| 接口局部修正版 | 约 7.061 s | phi 接近 0.005 上界 |
| nominal Gamma | 约 2.52 s | y 接近上边界 |
| D1 尺度修正 | 约 1.92 s | y 接近上边界，普通投影存在通道抵消 |
| 加权投影、尺度未闭合 | 约 7.063 s | y 获救，phi 接近上边界；混入约 46% 增益放大 |
| 加权投影、尺度闭合 | 约 7.05982 s | y 仍安全，phi 距上界约 `5.45e-6` |
| 当前二维 HJB 原型 | 约 1.89121 s | y 接近下边界；Actor 在线修正形成过慢 |

这些结果不能表述为“已稳定跟踪”或“已完成论文复现”。

## 当前存在的问题

1. 等权二次初始值函数不是已证明可稳定完整 AGV/SFPPB 闭环的可容许初始策略，在线 Critic/Actor 在约 1.89 s 内来不及补足该缺口。
2. `phi_J` 已经能够看到 `s1` 和 `z2_bar`，但当前 Bellman 模型只显式使用 `z2_bar` 动力学；如果把 `s1` 直接加入性能指标或值函数状态，还必须同步建立 `s1_dot`，不能只增加一个边界权重项。
3. 当前二维 Identifier 是第一版推广，仍需要验证 `F_hat` 对真实 `z2_bar_dot-C*delta-O2` 的估计质量。
4. 目前没有经验回放、离线预训练或持续激励机制；单次在线轨迹提供的 Critic 信息不足。
5. Level-1 MATLAB S-Function 会产生弃用警告，但这不是本轮数值失效原因，当前为保持原 AGV_TFS 风格暂不迁移接口。

## 下一步工作

1. 不再恢复或搜索固定 `P`、`c2` 和学习率组合。
2. 在保持当前二维策略公式的前提下，先得到一个有验证依据的可稳定初始策略，或采用离线/历史数据训练 Critic 后再进行在线策略迭代。
3. 若性能指标需要直接惩罚 `s1`，将 HJB 状态扩展为 `[s1;z2_bar]`，并从冻结的第一层公式解析建立 `s1_dot`；在此之前不使用缺少状态动力学的边界成本。
4. 增加 Identifier 估计误差、Bellman 残差、Actor/Critic 分项权重和 `O2` 的诊断输出；外部 3 路 Simulink 接口仍保持不变，可通过离线诊断脚本读取内部状态。
5. 以 20 s 无输出异常、误差始终位于 SFPPB 内、控制输入满足限制作为最低通过条件。

## 运行方式

1. 在 MATLAB 中将本目录设为当前文件夹。
2. 打开并运行 `AGV_simulate.slx`。
3. 仿真结束后运行 `AGV_plot.m` 绘制结果。

当前版本预期会在约 1.891 s 附近暴露上述问题；具体时刻会随求解器步长略有变化。这是二维 HJB 接口的第一轮可复现实验，不是最终结果。

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
