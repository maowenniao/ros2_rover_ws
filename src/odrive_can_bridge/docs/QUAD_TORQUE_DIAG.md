# 四轴同时力矩诊断（Phase0）

本工具是 `odrive_quad_torque_diag`。它只负责在 **一个进程** 里对 node `0,1,2,3` 做与单轴同族的电流/力矩梯度。

**实跑需用户当次授权。** 没有当次、点名节点与旗标的明确授权时，只允许 dry-run 和 `--simulate`。不得部署到 Jetson，不得对电机执行 `--execute`，不得清错，不得保存配置。

单轴工具 `odrive_torque_diag` **仍然禁止**四轮同时力矩。多轴不要给旧工具加 `--nodes`。

## 板与轴

| 板 | USB 序列号 | 节点 |
|----|------------|------|
| 左 | `3580387c3335` | 0, 1 |
| 右 | `358038733335` | 2, 3 |

Execute 前两块板都要过只读 USB 门控（序列号、`node_id`、VELOCITY+PASSTHROUGH、健康 IDLE、Kt/vel_gain/vel_limit 为正、`current_lim ≥ 5`、Iq 与心跳广播周期为正、`enable_watchdog` 为假、转矩速度保护保持开启）。任一侧失败就 **不开 CAN**。Kt 用该轴所在板当次读到的 `torque_constant`。

缓存文件只能作为次选：`--usb-baseline-left` 与 `--usb-baseline-right` 必须加上 `--acknowledge-stale-usb-baseline`，且时间戳不超过 5 分钟。文件读取是只读的，运动中不热插拔 USB。

## 默认剖面与停止

- 电流默认 `0.1 → 5.0 A`，步长 `0.1 A`，斜率 `0.5 A/s`，每级 hold `1 s`，再对称降回 0。
- 默认 `--profile staggered`：按节点顺序把整段剖面错开 90°。`--profile sync` 才会四轴同时升降。
- 软停 **默认开**（估计速度连续 3 个 20 ms 节拍超过 `3.5 rad/s` 则全停）。关掉必须同时给出 `--disable-soft-speed-stop` 和 `--authorize-no-soft-speed-stop-multi-axis`。
- 调度间隔超过 `0.10 s` 则全停，不把电流跳到“追赶”后的目标。
- 只收心跳、编码器、Iq、母线广播。不发寻址空查询。Iq 超过 `0.5 s` 没有新广播就全停，缺失不得记成 `0 A`。
- 母线电压高于本段基线 `+2.0 V` 则全停。绝对上限需现场标定后才能当作 Phase2 条件。

## 故障

任一轴 `axis/motor/encoder/controller` 错误、心跳过期、外来主机帧或上述停止条件：

1. 全部目标轴 `Set_Input_Torque(0)`
2. 全部目标轴 `Set_Axis_State(IDLE)`
3. 保留故障，**不清错**，不保存，不写 `vel_limit` / `current_lim`，不自动再进闭环

进程用 `fcntl.flock` 独占（优先 `/run/lock/odrive_can_diag.lock`）。第二个实例 exit 2，不写 CAN。

## 执行旗标

`--execute` 在下面每一项都齐、双板 USB 通过、文件锁到手、且没有其它 CAN 主机之前，**零 CAN 帧**：

```
odrive_quad_torque_diag --execute \
  --nodes 0,1,2,3 \
  --power-attended \
  --wheels-raised \
  --authorize-ram-torque-mode \
  --acknowledge-kt-unverified \
  --authorize-simultaneous-multi-axis-torque \
  --acknowledge-bus-regen-risk
```

授权语句需要当次写明节点、profile、是否关软停、电流上限。授权只覆盖这一次。失败 fail closed，不清错。

Phase0 交付到此为止：pytest、dry-run、`--simulate`。仿真对象只用于把门控跑通，不代表空载轮子会以 16–17 rad/s 跑完。
