## Why

当前切换回调只校验并更新模式，切换过程中缺失的命令会按普通命令错误处理，也没有
逐关节的接管等待期限。需要显式的内部 `Hold` 模式，在接口交接和有效首帧到来前停止
控制派发，并在超过 50 个控制周期时报告切换超时。

本变更基于当前代码和最新的 `adapt-json-config-and-brake-motors` 规格，保留用户新增的
规则：`None` 下 `Unsupported` 和 `Engaged` 关节每周期使用 `CurControl(0.0F, 1)`
请求反馈。`Hold` 是独立的切换状态，不复用 `None` 的抱闸或零电流分支。

## What Changes

- 增加内部 `ModeDefinition::Hold`。合法切换涉及的关节进入 Hold，其他关节继续正常
  read/write；最终释放全部接口的 stop-only 请求在 perform 后直接进入 `None`。
- Hold 期间跳过该关节的 `GetStatus`，保留最后的真实状态值；每次 write 结束将命令槽
  复位为 NaN。前 50 次未接管 write 不发送该关节的控制命令，也不报普通缺命令错误。
- 目标接口集合确认且经过一次命令清空边界后，只有完整有效的新首帧可以结束 Hold。
  各关节独立接管，同模式控制器交接也遵循这一规则。
- 第 51 次未接管 write 报一次 `controller switch timeout`，先通过 EngageBrake 抱紧
  已松闸关节；抱闸成功或不支持抱闸时，每次 write 发送 `CurControl(0.0F, 1)` 请求反馈。
  已抱紧关节不重复抱闸；失败返回 ERROR，不发送零电流。仍保持 Hold、跳过 GetStatus、
  清空命令槽；迟到的有效首帧仍可接管。超时本身不返回硬件 ERROR，实际 SDK 失败沿用现有错误传播。
- Hold 前 50 次等待不操作机械抱闸，超时后已松闸关节需要抱紧。退出 Hold 后沿用有效首帧
  松闸规则；直接进入 None 时沿用最新的抱闸及零电流反馈请求规则。
- 增加逐关节切换代次、周期计数和相应测试任务，明确线程边界及生命周期清理。

## Impact

- 能力域：`ec-joint-hardware`。
- 后续实现涉及 `types.hpp`、`JointRuntime`、模式名称辅助函数、模式切换回调、
  read/write、生命周期及包内 Mock/Fake 测试和 README。
- 本次交付仅更新 OpenSpec；实现和测试任务保留待办，不修改 C++ 或启动真实硬件。
- 不新增 ROS 接口、服务、hardware 参数或 JSON 字段，维持现有八个命令接口和六个
  状态接口；等待上限固定为 50 个周期。
- 本变更明确替代旧切换规则中的“保留命令缓存”和“立即派发”，不恢复
  `add-controller-switch-first-command-gate` 的整实例首帧等待。其他 JSON、CAN FD、
  抱闸、None 反馈和 SDK 所有权规则继续遵循最新的 adapt 变更。
