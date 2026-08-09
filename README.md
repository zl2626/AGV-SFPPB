# AGV-SFPPB

基于 MATLAB/Simulink 的 AGV 横向控制复现工程。项目以原始 `AGV_TFS` 工程为基础，正在将控制器改造成“滑模柔性规定性能边界（SFPPB）+ 直接 Identifier–Critic–Actor 强化学习”的结构。

> 当前版本属于“结构已经替换、正在稳定性调试”的中间版本。尚未声称完成 20 s 全程稳定仿真或完成论文级结果复现。

上一轮局部修正版只修改了控制器接口，不改变 Simulink 结构：输入饱和辅助状态加入 `C_norm` 缩放，暂时去掉 `w` 对 `alpha_1` 的直接作用，并将第三输出改为全部 Identifier–Critic–Actor 权重的总范数。

当前实验版本进一步只修改 `AGV_transfor.m`：为时变 SFPPB 边界加入解析导数，计算 nominal `Gamma`，并用 `Gamma` 替换继承自原 `AGV_TFS` 的 `T=0.01*TT` 项。该版本尚未通过 20 s 仿真。

## 当前完成情况

- 已建立独立工程目录 `AGV-SFPPB`，保留原 AGV 的 Level-1 S-Function 和 Simulink 端口结构。
- 已修改 `AGV_transfor.m`：
  - 加入 y 方向和航向角 phi 的 SFPPB 上下边界；
  - 保留原有误差变换、滑模变量、虚拟控制量和输出端口顺序；
  - 保留原车辆模型、RBF 网络和 Simulink 连接方式。
- 已修改 `AGV_ctrl.m`：
  - 改为 22 状态 Identifier–Critic–Actor 控制器；
  - 状态划分为 `WF(1:7)`、`Wc(8:14)`、`Wa(15:21)` 和输入约束辅助状态 `O(22)`；
  - 保留 13 输入、3 输出接口，并加入方向盘输入饱和映射；
  - 使用 `dO = -O + C_norm*(k_delta-delta)`，第三输出为 `norm([WF;Wc;Wa])`。
- 已在 `AGV_transfor.m` 的导数和输出计算中同时去掉 `w*eta_1` 对 `alpha_1` 的直接作用；`w` 状态、21 输出顺序和 Simulink 接口仍保留。
- 当前实验版本在 `AGV_transfor.m` 的导数和输出计算中加入 `kappa_dot`、`shift_dot`、nominal 边界导数和 `Gamma`；`AGV_ctrl.m` 未修改。
- 未修改 `AGV_plant.m`、`AGV_RBF.m`、`assist1.m`、`AGV_plot.m` 和 `AGV_simulate.slx` 的原有结构。

## 已完成的检查

- MATLAB S-Function 接口检查通过：`AGV_transfor` 为 5 状态、21 输出、6 输入；`AGV_ctrl` 为 22 状态、3 输出、13 输入。
- 两个 S-Function 的直接调用可以正常返回有限数值。
- t=0 时初始误差位于 SFPPB 边界内部，t=5 s 时边界能够收缩到设定终值。
- MATLAB 静态检查未发现语法错误；目前仅有少量未使用输入/局部参数提示。
- 原始 `AGV_TFS` 控制器作为基线可以完成 20 s 仿真；这说明原模型和原连接基本可运行。
- 上一轮 `3d91b26` 版本可以运行到约 `t=7.0613 s`；在 `t=7.0 s` 时，未饱和转角范围约为 `[-0.00482, 0.00154]`。
- 当前 `Gamma` 实验版本的接口和有限值检查通过，但分段仿真只能运行到约 `t=2.51 s`。

## 当前存在的问题

当前 SFPPB-RL 控制器仍然没有通过 20 s 全程仿真。需要区分三个版本的失效现象：

- 修正前版本在约 **t=1.563 s** 失败，曾出现 `e_phi≈0.03650`、上界约 `0.03714`，以及未饱和转角约 `49.38 rad`；
- 上一轮接口修正版本在约 **t=7.0614 s** 附近失败；在 `t=7.0613 s` 时，`e_phi≈0.0049875`，SFPPB 上界约 `0.005`，`w≈1.69`；
- 当前 nominal `Gamma` 实验版本反而在约 **t=2.52 s** 失败；在 `t=2.51 s` 时，`e_y≈0.0212063`、上界约 `0.0212797`，边界余量约 `7.34×10^-5`，`w≈11.22`；
- 当前失败表现仍为 `AGV_transfor` 输出不再是 21 维实数向量。这说明虽然 `Gamma` 的形式更接近时变边界推导，但在现有 `sigma_ni`、`alpha_1` 和未建模 flexible `eta_dot` 的组合下，符号/尺度或接口耦合仍需进一步核对；
- 因此，目前不能把该版本的结果表述为“已稳定跟踪”或“已完成论文复现”。

## 运行方式

1. 在 MATLAB 中将本目录设置为当前文件夹。
2. 打开并运行 `AGV_simulate.slx`。
3. 仿真结束后运行 `AGV_plot.m` 绘制结果。

当前实验版本预期会在约 2.52 s 附近暴露上述稳定性问题；具体时刻会随 MATLAB/Simulink 求解器步长略有变化。这属于待解决问题的复现实验，不是最终结果。

## 下一步工作

1. 先核对 `Gamma` 的符号、`sigma_ni` 的配合关系，以及 nominal 边界导数与 flexible 边界之间的接口。
2. 在公式接口确认前不放宽 `kappaT_phi`、不优先调 `c2` 或 Actor 学习率。
3. 以 20 s 无输出异常、无求解器失败为最低验证条件。
4. 公式稳定后，再完成与原始 `AGV_TFS` 的跟踪误差、控制输入和全部网络权重范数对比。

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `AGV_transfor.m` | SFPPB 边界、误差变换、滑模变量和辅助动态 |
| `AGV_ctrl.m` | 22 状态 Identifier–Critic–Actor 控制器及输入约束映射 |
| `AGV_plant.m` | AGV 横向动力学模型 |
| `AGV_RBF.m` | RBF 基函数计算 |
| `assist1.m` | 辅助 S-Function |
| `AGV_simulate.slx` | Simulink 仿真模型 |
| `AGV_plot.m` | 仿真结果绘图脚本 |
