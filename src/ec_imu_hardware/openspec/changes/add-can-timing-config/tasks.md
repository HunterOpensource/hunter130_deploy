## 1. 规格

- [x] 1.1 确认可选参数、xacro 入口、driver 默认值及最后一次配置生效的行为。
- [x] 1.2 OpenSpec 严格校验。

## 2. 实现

- [x] 2.1 解析并校验 can_bitrate、can_sample_point。
- [x] 2.2 on_configure 通过目标 bus 的 extend 配置适配器并处理失败。
- [x] 2.3 补充 README，明确所有 CAN/CAN FD 设备共用 bus 的配置约束。
- [x] 2.4 覆盖缺省不配置、单字段、自定义、非法值、创建/复用、总线隔离与失败场景。
- [x] 2.5 完成构建、测试、格式和静态检查。

## 3. 配套 xacro

- [x] 3.1 接入现有两个本体 xacro 的可选 PMS/IMU 参数，空值不生成 hardware 参数。
- [x] 3.2 更新说明并验证缺省、单字段、自定义、共用总线及仅一方配置的场景。

## 验证记录

- IMU：构建成功，17 项测试全部通过。
- clang-tidy：无错误；原有 on_init(HardwareInfo) 接口报告 1 条弃用告警。
- 全包 C++ 文件 clang-format --dry-run --Werror、git diff --check 通过。
- robot_description 装配测试 130 项通过，其中 12 项覆盖 CAN 参数缺省、单字段、独立总线、共用总线及仅一方配置。
- OpenSpec 严格校验通过；验证使用测试插件，未启动真实硬件。
