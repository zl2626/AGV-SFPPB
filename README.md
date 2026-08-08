# AGV_TFS

AGV（自动导引车）横向运动控制仿真项目，基于 MATLAB / Simulink。

控制方案：反步法（backstepping）+ 预设性能控制（PPC / 障碍李雅普诺夫函数 BLF）+ RBF 神经网络自适应补偿。

## 文件说明

| 文件 | 说明 |
| --- | --- |
| `AGV_plant.m` | 车辆横向动力学被控对象模型（Level-1 S-Function，含时变轮胎侧偏刚度与外部扰动） |
| `AGV_transfor.m` | 误差变换与边界函数：性能函数、误差映射、虚拟控制律、PI 补偿 |
| `AGV_ctrl.m` | 控制器：RBFNN 自适应、控制律、饱和处理 |
| `AGV_RBF.m` | RBF 径向基函数计算 |
| `AGV_plot.m` | 仿真结果绘图脚本（LaTeX 图注） |
| `assist1.m` | 辅助 S-Function |
| `AGV_simulate.slx` | Simulink 仿真模型 |
| `Fig/`、`Fig2/` | 仿真结果图（.fig） |

## 运行方式

1. 在 MATLAB 中打开 `AGV_simulate.slx`，运行仿真；
2. 仿真结束后运行 `AGV_plot.m` 绘制结果曲线。
