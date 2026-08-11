# fig3 结果说明

- `sfppb_pi_01~13`：当前结构修订后的 nominal 工况结果，使用 `u_d=0.5`。
- `sfppb_pi_sat03_*`：旧版饱和工况结果。当前代码统一物理输入增益后，`u_d=0.3` 需要重新预调参，旧图暂不作为当前结果。
- `sfppb_pi_u_*`：旧版 U 形工况结果。当前结构修订后该工况会先触发性能边界越界，需要重新定义更合理的 U-turn 工况并重新验证。

nominal 图由当前目录下的 `AGV_plot.m` 生成，Reference 为红色虚线，Actual AGV path 为蓝色实线。
