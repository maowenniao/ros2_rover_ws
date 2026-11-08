# 单轴力矩诊断

`odrive_torque_diag` 一次只接受一个 `--node`（0..3）。不要四轮同时给力矩。

多轴请用 `odrive_quad_torque_diag`，本工具仍禁止。见 `docs/QUAD_TORQUE_DIAG.md`。

实跑需用户当次授权。未授权时不得 `--execute`。本工具不清错、不保存配置、不写速度/电流上限。
