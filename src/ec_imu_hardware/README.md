# ec_imu_hardware

IMU 的 ros2_control 硬件插件。CAN 时序从 xacro 的硬件参数读取，使用 motor_driver 配置设备所在总线。

## CAN 配置

| hardware 参数 | 未传入时 | 说明 |
| --- | --- | --- |
| `can_bitrate` | 不配置，由 driver 决定 | 仲裁段波特率，单位 bit/s，正整数且不超过 int 上限 |
| `can_sample_point` | 不配置，由 driver 决定 | 仲裁段采样点，有限数值，范围 `(0, 1]` |

两个参数均可选，插件不设置默认 CAN 时序。只传一个参数时只配置该字段；两个都不传时不发起总线配置更新，
由 driver 使用默认值或保留总线已有配置。xacro 参数留空时不生成对应 hardware 参数；
直接在 hardware 参数中填写空值、非法数值或溢出会使初始化失败。
PMS 和 IMU 均使用普通 CAN，本插件不提供 CAN FD 开关、数据段时序或设备 CAN ID 修改参数。

在本体 xacro 中声明参数，并在现有 IMU 的 `<hardware>` 节点内像 adapter 参数一样传入：

```xml
<xacro:arg name="imu_can_bitrate" default="" />
<xacro:arg name="imu_can_sample_point" default="" />

<!-- 放入现有 IMU hardware 节点，与 adapter_type、interface_name 同级 -->
<xacro:if value="${'$(arg imu_can_bitrate)' != ''}">
  <param name="can_bitrate">$(arg imu_can_bitrate)</param>
</xacro:if>
<xacro:if value="${'$(arg imu_can_sample_point)' != ''}">
  <param name="can_sample_point">$(arg imu_can_sample_point)</param>
</xacro:if>
```

`ec_h175_upper.xacro` 和 `ec_exoskeleton.xacro` 分别提供 `imu_can_bitrate`、
`imu_can_sample_point`、`pms_can_bitrate`、`pms_can_sample_point`，默认留空，不传 CAN 配置。
EtherCAT/IGH 沿用设备已有的 `slave_idx`、`bus_idx` 选择下游 CAN bus；SocketCAN 使用所选接口的 bus 0。

## 一条 bus 只能有一套 CAN 配置

**一条物理 bus 只能有一套一致的 CAN 配置。** 约束适用于这条 bus 上的所有传感器、电机和其他设备，
无论使用 CAN 还是 CAN FD，不仅限于 PMS 和 IMU。

同一 bus 的仲裁段波特率、采样点必须一致；涉及 CAN FD 时，数据段时序也必须统一配置。
这里的参数配置的是设备所在的总线，不是某个设备独享的通信参数。
改变总线时序不会自动修改设备固件中的通信设置，必须保证实际设备也匹配。

总线配置由 driver 统一处理。对于支持总线更新的适配器，同一 bus 最后一次成功传入的 CAN 配置生效：
显式字段覆盖当前值，未提供的字段保持当前值，并非拒绝不同配置。
PMS/IMU 不再实现跨插件配置检查；未传参的插件不会覆盖已有配置。
共用 bus 时可由一个使用方配置，其余使用方省略；若多处填写，仍应保持一致，避免初始化顺序改变最终时序。

不同物理 bus 可以独立设置，例如 EtherCAT 从站 1 的 bus 2 和 bus 3：

```text
imu_slave:=1 imu_bus_idx:=2 imu_can_bitrate:=500000 imu_can_sample_point:=0.75
pms_slave:=1 pms_bus_idx:=3 pms_can_bitrate:=250000 pms_can_sample_point:=0.8
```

若 PMS、IMU 及其他设备共用从站 1 的 bus 2，则所有使用方都必须匹配同一配置，例如：

```text
imu_slave:=1 imu_bus_idx:=2 imu_can_bitrate:=250000 imu_can_sample_point:=0.75
pms_slave:=1 pms_bus_idx:=2 pms_can_bitrate:=250000 pms_can_sample_point:=0.75
```

上例也要求该 bus 上的电机和其他设备使用相容的 `250000 / 0.75` 设置；不同从站或接口上的 bus 不受影响。

## 生效时机与驱动边界

- `on_init()` 只解析和校验；`on_configure()` 经 `MakeAdapter(..., extend)` 应用配置，然后获取设备。
- `extend` 只含目标 `buses` 条目中显式提供的字段；两个参数均未提供时传空对象，不触发总线更新。
- EtherCAT/IGH 支持新建及复用适配器时配置指定 bus。配置会影响该 bus 的通信，应在开始通信前完成。
- SocketCAN（`adapter_type=Can`）只在适配器首次创建时应用配置，不支持上述复用更新；修改后需要重启进程，并确保首次创建方提供所需配置。
- 驱动抛出配置错误时，`on_configure()` 返回 `ERROR` 并记录设备、接口、总线及原因，不进入激活状态。
- 沿用当前 motor_driver API，不增加运行时配置服务，也不修改既有状态或命令接口。
