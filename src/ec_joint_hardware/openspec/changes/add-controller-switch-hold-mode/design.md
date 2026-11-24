## Context

当前 prepare 校验接口集合，perform 更新关节模式，read 调用 `GetStatus(false, 1)`，
write 按模式校验并派发，结束时清空命令槽。普通模式缺少命令会产生错误日志。
需要把切换中的暂时空命令与稳态控制器停写区分开，同时防止旧命令被新控制器接管复用。

本设计以用户最新修改的 `adapt-json-config-and-brake-motors` 为基础。该变更规定
None 下 Released 关节先抱紧，Unsupported 和 Engaged 关节持续零电流请求反馈。
新增 Hold 只覆盖切换等待行为，不能把 None 的反馈维持逻辑移入前 50 个等待周期，
也不能使 stop-only 路径失去新的反馈请求行为。

## Goals / Non-Goals

**Goals:**

- 逐关节保护接口交接，完整有效首帧才恢复对应控制模式。
- 前 50 个未接管周期静默，第 51 个未接管周期先抱紧已松闸关节，再开始零电流反馈请求并单次报超时。
- Hold 不消费 SDK 反馈生命周期、不改写真实状态，前 50 次等待不触发机械抱闸动作。
- 保留最新 None 行为、一次 Commit、其他关节独立运行和 SDK 错误传播。

**Non-Goals:**

- 不实现位置保持、阻尼保持或补齐 controller 的缺失参数。
- 不修改 SDK、反馈看门狗、电机心跳或控制器接口，不增加可配置等待参数。
- 不修改本次范围外的历史规格，不执行真实电机动作。本次只交付规格。

## Decisions

### 1. Hold 与认领模式分开存储

将 Hold 追加到 `ModeDefinition`，保留已有枚举值，`ModeName(Hold)` 返回 `hold`。
Hold 是硬件内部执行状态，不能由接口集合解析得到，不加入 `kModeDefinitions`。
空接口集合仍唯一对应 None，非法集合仍返回 `std::nullopt`。

每个 JointRuntime 分开记录已确认的认领集合/模式、待接管目标模式、实际执行模式，
以及切换代次、perform 确认、确认后的命令清空边界、等待次数和超时报告标记。
Hold 中再次计算增量认领时从已确认的认领集合出发，不能查找 Hold 的接口映射，
也不能把尚未确认的候选集合当作已有认领。

### 2. 逐关节进入及回调边界

先完整校验本次 start/stop 请求，再发布受影响关节的切换请求。受影响指 start 或
stop 列表引用本实例的该关节；同模式替换也算切换。无关关节不进入 Hold、不重置计数。
任一候选集合非法，prepare 返回 ERROR，不发布任何新请求、不清空命令或改动既有等待。

非空目标的合法 prepare 发布新代次。实时 read/write 在下一次观察到该请求时进入
Hold，计数及报告标记清零；命令清空在实时侧执行。prepare 不确认新认领模式，不从
服务线程直接改写由控制循环使用的 command double 或等待计数。

perform 仍先校验完整请求，然后为匹配的代次确认目标接口集合；确认不结束 Hold，
也不重置已经累计的等待次数。旧确认不能覆盖新请求；perform 失败不能部分确认。
如果 prepare 已进入 Hold 而 perform 未发生或失败，关节继续等待并可超时。

stop-only 且最终集合为空时不建立新的 Hold 等待；成功 perform 直接切到 None，
取消该关节原有等待并在实时侧清空命令。之后若再次认领非空集合，建立新的 Hold。
先 stop 再 start 的两次独立调用之间就是 None，不能跨请求猜测控制器即将接管。

请求发布与消费采用线程安全的代次/快照交接，不将裸 bool 当作跨线程同步，也不在
实时循环新增 sleep 或等待服务完成。控制循环拥有执行状态、命令复位和计数。
生命周期清理使此前所有代次失效，避免延迟确认恢复旧绑定。

### 3. 清空边界与有效首帧

进入 Hold 后，等待期间每次 write 都执行 ResetCommands，将全部八个命令槽复位为
NaN；controller 在两次 write 之间写入的数据仅作为当前周期候选值。

首次观察到匹配 perform 确认的 write 必须先丢弃已有命令，完成一次确认后的清空
边界，该次不能接管，即使已有完整合法值。其后的 write 才能用当前目标模式的全部
必填接口校验首帧。这样 prepare 与 perform 之间或旧 controller 最后一次 update 写入
的值不会被新模式继承；同模式切换也不例外。该边界周期计入同一个 50 周期窗口。

目标确认、清空边界完成、首帧完整有效三个条件同时满足时，该关节在本次 write
退出 Hold 并执行目标模式。连续量必须有限，PVT torque_mode 必须精确为 0/1/2，
其他校验沿用现有模式规则，不补齐任何字段，不等待其他关节。

首帧检查优先于超时分支：等待已满 50 次时，本次若可接管，直接执行首帧，不报
超时、不发送零电流。超时后同样允许迟到首帧接管，无须强制再发一次切换请求。
退出后的普通命令缺失继续按现有规则报错，不自动回到 Hold。

### 4. read 跳过 Hold，状态与命令分别处理

read 先观察切换请求，随后对仍处于 Hold 的关节完全跳过 GetStatus，包括超时后。
因此不会由该硬件 read 消费该关节的 SDK 反馈生命周期。不能通过传入不同 TTL、
重置计数或伪造反馈延长缓存；真正反馈到达时由 SDK 自身刷新缓存。

六个状态接口保留最后一次真实 SDK 反馈值，不写 NaN，不从命令反推状态，也不因
主动跳过读取而报 missing feedback。其他关节正常读取、发布和诊断。
本约束针对硬件 read 的调用，不改写 SDK 控制方法内部的反馈访问。

退出 Hold 后下一次 read 恢复正常 GetStatus。首帧触发 ReleaseBrake 时，仍按最新
adapt 规格获取保持位置；不能用 Hold 前保存的 state interface 代替 SDK 有效反馈。

### 5. 50 周期和超时行为

等待次数由 active write 对每个仍未接管的 Hold 关节增加一次，read 不另计数；
单独重复 read 不推进期限。进入后的第 1 至 50 次未接管 write 静默，第 51 次开始
超时，之后计数饱和，避免长期等待溢出。perform 确认不重新开始计时。

超时当次记录一次包含关节名、目标模式、代次和周期的 ERROR，固定文本包含
`controller switch timeout`。同一关节同一代次不重复记录。该次及其后每次尚未接管的
write 先调用 EngageBrake；Released 关节抱紧成功后进入 Engaged，Engaged/Unsupported
不重复调用 SDK 抱闸。返回 Engaged 或 Unsupported 时调用 `CurControl(0.0F, 1)`，
抱闸失败或异常时返回 ERROR 且不发送该关节零电流，所有退出路径继续清空命令槽为 NaN。
这是明确要求的超时动作，不是为不完整的 controller 命令补默认值。

仅超时日志不使 read/write 返回 ERROR；这样控制循环能持续请求反馈并接受迟到首帧。
SDK 控制、保持、抱闸或 Commit 的实际失败仍按已有规则返回 ERROR，不掩盖异常。
新的合法非空切换为受影响关节开启新代次，重新等待并允许该代次再报一次超时。

1000 Hz 下 50 个正常控制周期名义上为 50 ms，实际受调度及 SDK 同步操作耗时影响。
这里是周期预算，不是墙钟截止时间，也不证明与电机 500 ms 心跳之间的硬实时余量。

### 6. Hold、None 与抱闸的边界

| 状态 | 硬件 read | write | 机械抱闸 |
| --- | --- | --- | --- |
| Hold，第 1–50 次未接管 | 跳过 GetStatus | 无该关节控制调用，命令清为 NaN | 保持原 brake_state |
| Hold，第 51 次及以后未接管 | 跳过 GetStatus | 抱闸成功或不支持抱闸时每次 CurControl(0.0F, 1)，失败 ERROR；命令清为 NaN | Released 抱紧，Engaged 不重复抱紧 |
| None，Unsupported | 正常读取 | 每次 CurControl(0.0F, 1) | 不支持，无动作 |
| None，Released | 正常读取 | EngageBrake 成功后按 Engaged 请求反馈；失败返回 ERROR | 成功进入 Engaged |
| None，Engaged | 正常读取 | 每次 CurControl(0.0F, 1) | 保持 Engaged，不重复抱紧 |
| None，Engaging/Releasing | 正常读取 | 沿用现有静默跳过 | 不重复 SDK 操作 |
| 退出 Hold 的有效首帧 | 下一次 read 恢复 | 按目标模式派发 | 沿用首次有效命令松闸规则 |

Hold 必须使用独立分支，不能落入 None 或普通无效命令分支。超时先调用 EngageBrake，
沿用其直接发送和确认后恢复同步的规则；不调用 ReleaseBrake、PosControl 或 Stop，不重新探测抱闸能力。
已松闸关节退出 Hold 直接派发；Engaged 关节按既有“有效位置保持→松闸→控制”的
顺序执行，失败返回 ERROR。原有过渡抱闸状态及失败处理不因首帧接管而伪造成功。

### 7. Commit、异常和生命周期

每个正常完成的 active write 在逐关节处理后仍恰好一次 Commit。全部关节静默 Hold
也调用一次 Commit，但不为这些关节排入控制；混合批次可包含其他正常命令、None
零电流和 Hold 超时零电流。此提交不承诺跨实例或跨物理总线同步。

沿用统一退出路径清空命令，SDK 异常不越过回调边界、不重试、不额外提交。
初始化、重配开始、配置失败、停用、错误和关闭时清除全部待接管状态、代次关联、
计数和报告标记；生命周期退出的抱闸动作保持最新 adapt 规则。inactive 的 read/write
不计数、不发送超时零电流、不 Commit。再次激活不能继承上次 Hold 或旧命令。

### 8. 与现有规格的合并顺序

最新 adapt 变更的 JSON、CAN FD、抱闸及 None 反馈规则是本变更的前置基线。
本变更覆盖其“切换保留命令槽、立即按模式派发”的条款，以及旧主规格中 prepare
绝不改变执行状态的条款；prepare 仍不确认认领模式，但可发布 Hold 请求。

Hold 的 read 规则是旧“每个 active read 对每个关节 GetStatus”的明确例外；Hold
等待/超时是普通缺命令日志和派发规则的明确例外。退出 Hold 后恢复已有行为。
旧实例首帧命令门继续废弃，不能因本变更重新引入整批等待。
旧“控制器退出后不得发送控制命令”应按最新 None 规则理解为允许零电流反馈请求，
仍禁止位置保持、松闸及非零电流。本变更不恢复旧的 Engaged 静默跳过行为。

## Verification

本次只运行 OpenSpec 严格校验和文档差异检查。后续实现使用 Mock/Fake 验证：
逐关节和同模式切换、旧命令清空、未确认/迟确认、50/51 周期边界、超时后接管、
read 不调用 GetStatus 且状态不变、None 已抱闸时持续反馈、混合批次一次 Commit、
SDK 异常、请求交错及生命周期重置。构建、格式、静态分析及真实 SDK Fake 检查
须在实现阶段完成，文档校验不代表代码或实机验证通过。
