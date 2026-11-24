## 配置与生效时机

硬件参数 can_bitrate、can_sample_point 均可选，不设置默认值。
can_bitrate 使用正 int 范围的十进制整数；can_sample_point 使用 (0, 1] 内的有限数值。
缺少参数不配置该字段；显式空值、非法内容、溢出等使 on_init 返回 ERROR。

on_configure 创建 extend 对象，只有 buses 下的目标总线条目包含显式提供的 bitrate 和/或 sample_point。
两个参数均缺少时传空对象，不创建 buses 条目，不触发复用适配器的总线更新。
EtherCAT/IGH 的键为 slave_idx:bus_idx，SocketCAN 为 0。调用带 extend 的 MakeAdapter 后
继续现有设备获取流程，不重复调用 UpdateBusExtend，不在 read/write 中修改时序。
驱动拒绝配置时记录设备、总线与原因，返回 ERROR，保持未配置、未激活。

## 兼容与边界

PMS、IMU 均使用普通 CAN，不添加 FD 开关或数据段参数。不同物理总线可独立配置。
一条物理 bus 只能有一套一致的 CAN 配置，适用于所有传感器、电机和其他 CAN/CAN FD 设备。
仲裁段波特率和采样点必须一致，CAN FD 数据段时序也必须统一。两个包 README 均写明此约束，
由 driver 统一处理总线配置，不新增跨插件注册表、锁、配置冲突检测或自动仲裁。
支持更新的适配器以最后一次成功传入的配置为准，显式字段覆盖当前值，缺失字段保留当前值。

依赖现有 motor_driver：EtherCAT/IGH 可在复用适配器时更新指定 bus；SocketCAN 只在首次
创建时应用配置，后续配置变更需重启进程并由首次创建方传参。
同 bus 可由一个使用方配置，其余省略；若多处提供参数，应保持一致，避免初始化顺序影响最终时序。

## 配套与验证

robot_description 的 ec_h175_upper.xacro、ec_exoskeleton.xacro 声明独立的 PMS/IMU 参数，
默认留空，仅非空参数生成对应硬件节点参数。保持现有启用条件、状态/命令接口和后端选择。
通过测试插件记录首次扩展配置与复用更新，覆盖缺省、单字段、自定义、非法值、总线隔离与失败；
运行硬件测试、xacro 展开测试、构建、格式及静态检查，不启动真实硬件。
