## Why

IMU 当前只允许选择 adapter 和 bus，无法从 xacro 设置 CAN 波特率与采样点。
需要沿用现有配置流程，将显式提供的目标总线时序交给 motor_driver；未传参时由 driver 使用默认值或已有配置。

## What Changes

- 新增可选硬件参数 can_bitrate、can_sample_point，仅传入显式提供的字段，不补默认值。
- on_init 校验，on_configure 通过 MakeAdapter 的 extend.buses 配置目标总线。
- 只暴露普通 CAN 仲裁段参数，不增加 CAN FD 参数或运行时配置服务。
- README 明确一条物理 bus 只能有一套一致配置，约束所有 CAN/CAN FD 设备；由 driver 按最后一次成功配置更新显式字段，不新增跨插件检查。
- 配套 xacro 使用 imu_can_bitrate、imu_can_sample_point 传参。

## Impact

- 修改本包配置、初始化、测试和 README，以及 robot_description 中现有 PMS/IMU xacro。
- 不修改 motor_driver、设备 CAN ID、设备固件配置或现有状态/命令接口。
- EtherCAT/IGH 支持复用时更新；SocketCAN 仅首次创建应用，修改后重启进程生效。
