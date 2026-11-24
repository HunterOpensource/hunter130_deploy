## ADDED Requirements

### Requirement: 逐关节 Hold 切换状态与首帧接管

硬件 MUST 增加内部执行模式 `ModeDefinition::Hold`。完整校验通过的 prepare 请求中，
start/stop 引用且目标认领集合非空的关节 MUST 在实时 read/write 下一次观察该请求时
进入 Hold；同模式控制器交接 MUST 同样进入 Hold。其他关节 MUST 保持原运行状态。
prepare MUST NOT 确认新的认领模式；perform MUST 先完整校验，再确认匹配代次的目标
集合，MUST NOT 直接结束 Hold 或重置等待次数。

首个观察到 perform 确认的 write MUST 丢弃已有命令并完成一次 ResetCommands 清空
边界，MUST NOT 使用该周期已有值接管。此后的 write 只有在目标模式全部必填接口
完整合法时才 MUST 退出 Hold，并在同次 write 沿用目标模式的派发及抱闸规则。
连续量 MUST 有限，PVT torque_mode MUST 精确为 0/1/2，MUST NOT 补齐缺失参数。
每关节 MUST 独立接管，MUST NOT 等待其他关节的首帧。

最终认领集合为空的 stop-only 请求 MUST 在成功 perform 后直接进入 None，清除该
关节既有 Hold 状态并在实时侧清空命令，MUST NOT 新建 50 周期等待。后续重新认领
非空集合 MUST 建立新的 Hold 等待。

#### Scenario: 同模式控制器交接

- **WHEN** start/stop 引用某关节且最终认领模式与原模式相同
- **THEN** 该关节 MUST 进入 Hold，完成目标确认、清空边界及有效首帧检查后才能派发

#### Scenario: 切换只涉及部分关节

- **WHEN** 请求仅引用关节 A，关节 B 未出现在 start/stop 中
- **THEN** 仅 A MUST 进入新的 Hold；B MUST 继续其原有 read/write，已有等待次数不得重置

#### Scenario: 未确认的目标不能接管

- **WHEN** prepare 已生效，但匹配的 perform 未确认，即使命令槽中有完整合法值
- **THEN** 该关节 MUST 保持 Hold 并清空候选命令，等待次数 MUST 继续累计

#### Scenario: 旧命令不能跨确认边界复用

- **WHEN** perform 确认前后命令槽中残留旧控制器写入的完整命令
- **THEN** 首次观察确认的 write MUST 丢弃这些值并清为 NaN，该次 MUST NOT 接管
- **AND** 只有后续 write 的完整有效首帧才允许退出 Hold

#### Scenario: 各关节独立接管

- **WHEN** A 和 B 均已确认目标并经过清空边界，只有 A 收到完整有效首帧
- **THEN** A MUST 在本次 write 退出 Hold 并执行命令，B MUST 继续自己的等待

#### Scenario: 首帧缺失或非法

- **WHEN** Hold 关节的目标必填值含 NaN、无穷或非法 torque_mode
- **THEN** MUST 保持 Hold，不派发该候选命令，不触发松闸，MUST NOT 记录普通缺命令错误
- **AND** MUST 继续等待计数并适用独立的超时规则

#### Scenario: 仅释放全部接口

- **WHEN** stop-only 请求在成功 perform 后使该关节认领集合为空
- **THEN** 该关节 MUST 直接进入 None，取消既有等待并清空命令，MUST NOT 报该代次切换超时
- **AND** 后续 write MUST 遵循最新 None 抱闸及零电流请求反馈规则

#### Scenario: 释放后重新认领

- **WHEN** 关节已处于 None，后续独立请求认领非空合法集合
- **THEN** 该关节 MUST 建立新的 Hold 等待，并完成确认、清空边界及有效首帧检查

### Requirement: Hold 期间暂停反馈读取与静默控制

active read MUST 对 Hold 关节完全跳过 GetStatus，包括 Hold 超时之后；MUST NOT
为该关节递减或人为刷新 SDK 反馈生命周期。六个 state interface MUST 保留最后
一次真实反馈，MUST NOT 写入 NaN 或伪造新反馈，MUST NOT 因主动跳过读取报告
missing feedback。其他关节 MUST 继续正常读取与诊断，退出 Hold 后 MUST 恢复正常读取。

Hold 前 50 次未接管 write MUST 不对该关节调用任何控制、保持、停止或抱闸方法。
所有 Hold write 的统一退出路径 MUST 将该关节全部命令槽重置为 NaN；控制器在
两次 write 之间写入的值只作为当前周期候选，MUST NOT 保留用于拼接后续命令。
超时后仅允许下面明确要求的抱闸及零电流反馈请求。

该 read 规则 MUST 作为旧“最小生命周期、反馈看门狗与直接状态发布”中逐关节
GetStatus 规则的 Hold 例外，不改变 SDK 内部操作和真实反馈到达时的缓存刷新。

#### Scenario: Hold 不消费反馈缓存

- **WHEN** active read 处理一个 Hold 关节，无论是否超时
- **THEN** MUST 不调用其 GetStatus，不人为改动 SDK 生命周期，也不报缺失反馈错误
- **AND** 该关节全部状态接口 MUST 保持最后一次真实值

#### Scenario: 等待期间写入不会遗留

- **WHEN** Hold 关节尚未满足接管条件，controller 写入了全部或部分命令
- **THEN** write MUST 丢弃本次候选值，并将所有命令槽清为 NaN；前 50 次 MUST 不下发控制

#### Scenario: 恢复真实反馈

- **WHEN** 某关节在 write 中成功退出 Hold
- **THEN** 下一次 read MUST 恢复正常 GetStatus 和状态发布，不使用命令推测反馈

#### Scenario: 非 Hold 关节继续读取

- **WHEN** 同一实例中只有部分关节处于 Hold
- **THEN** read MUST 正常读取其余关节，保留其原有错误诊断及 SDK 异常处理

### Requirement: Hold 的 50 周期超时与零电流反馈请求

Hold 等待上限 MUST 固定为 50 次 active write，read MUST NOT 额外计数。首次观察
perform 确认的清空边界周期 MUST 计入同一窗口。第 1 至 50 次未接管 write MUST
静默等待，第 51 次未接管 write MUST 记录一次 ERROR，文本包含
`controller switch timeout`、关节名、目标模式、代次和等待周期；同一关节同一代次
MUST NOT 重复记录。计数 MUST 饱和，MUST NOT 因整数溢出重新开始等待。

超时当次及其后每次未接管 write MUST 先调用 EngageBrake，Released 关节 MUST 抱紧，
Engaged/Unsupported MUST 不重复调用 SDK 抱闸。仅返回 Engaged 或 Unsupported 时 MUST
调用 `CurControl(0.0F, 1)`；抱闸失败 MUST 返回 ERROR，MUST NOT 发送该关节零电流。
MUST 保持 Hold、跳过 read 中的 GetStatus、清空命令槽，MUST NOT 触发松闸、位置保持或 Stop。
超时本身 MUST NOT 使 read/write 返回 ERROR；实际 SDK 操作失败 MUST 沿用既有
错误传播规则。该零电流是明确的超时动作，MUST NOT 被用于补齐目标模式的缺失参数。

完整有效首帧检查 MUST 优先于超时判断，包括已经超时的关节。只要目标确认和清空
边界已完成，迟到的合法首帧 MUST 能结束 Hold，无须用户再发切换请求；接管当次
MUST NOT 再发送超时零电流。周期预算 MUST NOT 表述为墙钟时间或心跳余量的保证。

#### Scenario: 第 50 次仍静默

- **WHEN** Hold 关节连续 50 次 active write 都未接管
- **THEN** 这 50 次 MUST 不发送该关节控制，不记录超时；额外 read MUST 不推进计数

#### Scenario: 第 51 次开始超时动作

- **WHEN** 已等待 50 次，下一次 write 仍无法接管
- **THEN** MUST 报一次 controller switch timeout，先 EngageBrake，成功或不支持抱闸时调用 CurControl(0.0F, 1)
- **AND** 后续未接管 write MUST 保持抱闸并继续相同零电流请求，不重复抱紧或报本代次超时

#### Scenario: 边界周期首帧优先

- **WHEN** 已等待 50 次且已完成确认后清空边界，本次 write 收到完整有效首帧
- **THEN** MUST 直接接管并执行该帧，MUST NOT 报超时或发送超时零电流

#### Scenario: 超时后迟到首帧接管

- **WHEN** 关节已超时，随后满足确认、清空边界和完整有效首帧条件
- **THEN** MUST 在该次 write 恢复目标模式并执行命令，无须新的切换请求
- **AND** MUST 停止超时零电流，后续命令按普通模式校验

#### Scenario: 超时后控制与提交成功

- **WHEN** Hold 超时零电流及本次其他操作、Commit 均成功
- **THEN** write MUST 返回 OK，不能仅因切换超时使 hardware 进入错误生命周期

#### Scenario: 超时动作 SDK 失败

- **WHEN** Brake、CurControl(0.0F, 1) 或 Commit 抛出 SDK 异常
- **THEN** write MUST 记录实际异常并返回 ERROR，清空命令，不重试或额外提交

#### Scenario: 超时抱闸返回失败

- **WHEN** Released 关节的 EngageBrake 因 SDK Brake 返回 false 而仍返回 Released
- **THEN** write MUST 返回 ERROR 并清空命令，MUST NOT 继续发送该关节零电流或 Commit

### Requirement: Hold 生命周期与请求隔离

Hold 请求 MUST 以线程安全方式发布及消费，命令复位、执行状态和等待计数 MUST 由
实时侧维护，MUST NOT 在服务回调与实时循环之间无同步共享可变标志或命令槽。
新的合法非空切换 MUST 仅为受影响关节建立新代次、重置计数和超时报告标记；旧代次
确认 MUST NOT 覆盖新请求。非法或无关请求 MUST NOT 清除已有等待或延长期限。

初始化、重配开始、配置失败、停用、错误及关闭 MUST 清除待接管状态和命令，并使
旧代次失效；生命周期抱闸行为 MUST 继续遵循最新 adapt 规格。inactive 的 read/write
MUST 不累计等待、不发送超时零电流、不 Commit。再激活 MUST 不继承旧 Hold 或命令。

#### Scenario: 新切换开启新的等待窗口

- **WHEN** Hold 关节收到新的合法非空切换请求
- **THEN** MUST 建立新代次、清零等待计数及报告标记，旧确认 MUST 不能结束新 Hold

#### Scenario: 非法请求不延长已有等待

- **WHEN** 任一候选集合非法，或者请求完全不涉及一个正在 Hold 的关节
- **THEN** 该关节的代次、等待次数和报告标记 MUST 保持原值

#### Scenario: perform 未完成

- **WHEN** prepare 请求已进入 Hold，但对应 perform 未调用或完整校验失败
- **THEN** MUST 不确认新目标，继续 Hold，并在第 51 次未接管 write 执行超时规则

#### Scenario: 生命周期退出清除等待

- **WHEN** 等待中或已超时的硬件停用、进入错误或关闭
- **THEN** MUST 取消全部 Hold 等待并按原规则处理抱闸；后续 inactive read/write 无控制或提交
- **AND** 再激活 MUST 不恢复旧目标、计数或超时状态

#### Scenario: 配置失败后不存在旧请求

- **WHEN** 硬件重新配置或本次配置失败
- **THEN** 旧请求、确认、计数及命令 MUST 失效，不能在后续配置实例中生效

## MODIFIED Requirements

### Requirement: 类型安全的模式解析与单关节运行时职责

插件内部 MUST 以 ModeDefinition 枚举表示 None、六种控制模式及内部 Hold，MUST
保留已有枚举数值并追加 Hold。`ModeName(Hold)` MUST 返回 `hold`。模式到接口的
唯一映射 MUST 保持 `std::map<ModeDefinition, std::vector<std::string>>`，MUST NOT
为 Hold 新增认领接口；空集合 MUST 解析为 None，非法集合 MUST 返回 std::nullopt。
JointRuntime MUST 封装单关节模式解析、命令复位及状态，硬件 MUST 保留 URDF 顺序。

增量解析 MUST 从已确认的认领集合开始，只处理目标关节的键，先停止后启动并去重，
再复用完整集合解析，MUST NOT 从 Hold 查找接口集合。prepare MUST 先验证全部
关节再发布 Hold 请求，MUST NOT 确认新认领；perform MUST 先验证全部关节，再统一
确认匹配代次的目标，任一非法 MUST NOT 部分更新。实现 MUST NOT 保留重复的
ComputeCandidateInterfaces 或 MatchMode 路径。

#### Scenario: 空认领与非法认领可区分

- **WHEN** 完整接口集合为空，或无法组成合法模式
- **THEN** 空集合 MUST 返回 None，非法集合 MUST 返回 std::nullopt，均不能解析为 Hold

#### Scenario: 增量认领只影响目标关节

- **WHEN** 增量输入同时包含自身和其他关节的键
- **THEN** JointRuntime MUST 仅应用自身键，从已确认认领集合先停止后启动解析目标

#### Scenario: 模式切换执行保持原子性

- **WHEN** perform 的任一最终认领集合非法
- **THEN** MUST 返回 ERROR，不得部分确认任何新目标；已存在的 Hold 等待继续生效

#### Scenario: 重复接口不是合法模式

- **WHEN** 完整候选接口与合法集合名称相同但含有重复项，导致多重性或长度不同
- **THEN** 精确接口比较 MUST 拒绝该候选集合

#### Scenario: Hold 期间解析后续切换

- **WHEN** 实际执行模式为 Hold 时再次解析增量认领
- **THEN** MUST 使用已确认的认领集合，MUST NOT 查找 Hold 映射或使用未确认候选集合

### Requirement: 控制器切换后的命令派发

硬件 MUST 继续移除实例级 awaiting_first_command_ 及整批首帧门。Hold 关节 MUST
按逐关节确认、清空边界和有效首帧规则接管，MUST NOT 保留切换前命令缓存。
非 Hold 关节 MUST 沿用原模式校验、错误日志、抱闸及独立派发规则；首帧接管之后
普通缺命令 MUST NOT 自动进入 Hold。原“认领集合与模式参数映射”和“逐关节命令
校验与错误日志”中缺命令错误规则 MUST 仅对非 Hold 的已绑定控制模式生效。

#### Scenario: 只有部分关节写入有效命令

- **WHEN** 某关节已可正常派发或满足 Hold 接管条件，另一关节仍未就绪
- **THEN** MUST 独立执行就绪关节的有效命令，MUST NOT 因另一关节未就绪整批跳过

#### Scenario: 无效命令不触发首次松闸

- **WHEN** 尚未松闸的关节收到不完整或非法命令
- **THEN** MUST 不松闸、不派发该命令，其他合法关节独立派发；Hold 关节按等待/超时规则处理

#### Scenario: 接管后控制器停写

- **WHEN** 已退出 Hold 的关节在后续普通周期缺失命令
- **THEN** MUST 沿用普通缺命令日志及跳过规则，MUST NOT 自动启动新的 50 周期等待

### Requirement: 切换控制器不操作抱闸

prepare 发布 Hold 请求及 perform 确认目标/进入 None 时 MUST NOT 调用 Brake 或
改变 brake_state。Hold 前 50 次等待 MUST NOT 操作抱闸；第 51 次及以后未接管时
MUST 通过 EngageBrake 抱紧 Released 关节，再执行超时零电流；MUST NOT 触发
ReleaseBrake。有效首帧退出 Hold 后 MUST 沿用既有松闸与派发规则。
最终集合为空后，后续 write MUST 遵循最新“None 模式抱紧支持关节”：Released
先 EngageBrake，成功后的 Engaged 和 Unsupported 每周期 CurControl(0.0F, 1)，
Engaging/Releasing 静默跳过，抱闸失败返回 ERROR 且不发送该关节零电流。

#### Scenario: 已松闸关节切换控制器

- **WHEN** 已松闸关节进入 Hold 并在超时动作前用有效首帧接管
- **THEN** MUST 保留 Released，接管时直接执行命令，MUST NOT 重复位置保持或松闸

#### Scenario: 已松闸关节经历 Hold 超时

- **WHEN** Released 关节在 Hold 第 51 次 write 仍无有效首帧
- **THEN** MUST 先通过 EngageBrake 抱紧，确认成功后才发送零电流反馈请求
- **AND** 后续 Hold write MUST 不重复抱紧；迟到首帧按原有保持、松闸、派发顺序接管

#### Scenario: 控制器退出

- **WHEN** 成功 perform 后关节不再被控制器认领
- **THEN** 支持抱闸的 Released 关节 MUST 在后续 None write 抱紧
- **AND** Engaged 和 Unsupported MUST 每周期零电流请求反馈，不发送位置保持、松闸或非零电流

#### Scenario: 已抱闸关节经历 Hold 超时

- **WHEN** Engaged 关节在 Hold 超时且未收到可接管首帧
- **THEN** MUST 保持 Engaged，只执行超时零电流反馈请求，MUST NOT 发送保持或松闸命令
- **AND** 后续合法首帧接管时才按既有有效首帧顺序获取位置、保持、松闸和派发

#### Scenario: None 已抱闸时持续请求反馈

- **WHEN** stop-only 后关节在 None 保持 Engaged 超过 status_max_life_cycle 个周期
- **THEN** MUST 每周期 CurControl(0.0F, 1) 并提交，正常 read 获取反馈
- **AND** MUST 不套用 Hold 的静默或跳过 GetStatus 规则，也不重复抱紧

### Requirement: 统一同步提交与直接 SDK 错误传播

每个正常完成的 active write MUST 在逐关节处理之后恰好调用一次 Commit，包括
全部关节都在静默 Hold 时。批次 MAY 同时包含普通控制、None 零电流和 Hold 超时
零电流，MUST NOT 为单个 Hold 关节增加提交。同步边界 MUST 限于该实例的 manager，
MUST NOT 声称跨实例或跨物理总线的原子同步。

SDK 控制、保持、抱闸或 Commit 失败 MUST 沿用现有异常边界，记录关节名（如适用）、
模式及错误并返回 ERROR，MUST NOT 重试、额外提交、自动全刹或伪造反馈。所有 write
退出路径 MUST 清空命令。仅 Hold 等待或超时日志 MUST NOT 使 write 返回 ERROR。

#### Scenario: 同一硬件实例中已选择关节的控制只提交一次

- **WHEN** 本周期有多个可派发的有效命令且操作成功
- **THEN** MUST 各调用一次对应模式 API，并在本实例所有关节处理后恰好 Commit 一次

#### Scenario: PVT 批次只提交一次

- **WHEN** 有关节成功调用 PVTControl，其他操作也成功
- **THEN** MUST 在本次 write 末尾恰好 Commit 一次

#### Scenario: 无参数化提交开关

- **WHEN** 本实例 active write 正常完成
- **THEN** MUST 恰好 Commit 一次，MUST NOT 新增关闭或分批提交参数

#### Scenario: 两个硬件实例不构成全局同步域

- **WHEN** 两个 EcJointHardware 实例分别管理不同设备组或物理总线
- **THEN** MUST 各提交自己的 manager，不承诺跨实例或跨总线原子同步

#### Scenario: SDK 控制异常

- **WHEN** 某关节 SDK 控制方法抛出异常
- **THEN** MUST 记录关节、模式及异常，返回 ERROR，不重试、自动停止或额外提交

#### Scenario: 混合 Hold 与正常控制批次

- **WHEN** A 静默 Hold，B 超时 Hold，C 正常执行控制，D 在 None 且 Engaged
- **THEN** MUST 不对 A 下控制，B 和 D 各零电流请求一次，C 正常派发，成功时总计 Commit 一次

#### Scenario: 全部关节静默 Hold

- **WHEN** 本实例所有关节均在 Hold 前 50 次等待且本次操作正常完成
- **THEN** MUST 不为任何关节排入控制，仍恰好 Commit 一次并返回 OK
