## 1. 本次 OpenSpec 交付

- [x] 1.1 读取用户最新 adapt 规格，保留 None 下 Unsupported/Engaged 持续零电流请求反馈规则。
- [x] 1.2 明确逐关节 Hold、perform 确认后清空边界、有效首帧接管及 stop-only 直接 None。
- [x] 1.3 明确 read 跳过 GetStatus、命令 NaN、前 50 次静默、第 51 次超时和迟到首帧接管。
- [x] 1.4 编写 proposal、design、差量规格与后续实施/验收任务。
- [x] 1.5 通过本变更与全量 OpenSpec 严格校验及文档差异检查。

## 2. 后续模式与请求实现

- [ ] 2.1 追加 Hold 枚举及 ModeName，保留现有数值、八命令/六状态接口及空集合 None 语义。
- [ ] 2.2 分离已确认认领集合、待接管目标与执行状态，支持 Hold 中基于已确认集合增量解析。
- [ ] 2.3 实现完整校验后发布的逐关节代次请求和线程安全实时消费，处理同模式、无关及非法请求。
- [ ] 2.4 实现 perform 目标确认、确认后清空边界，防止旧命令和旧确认复用；失败不得部分确认。
- [ ] 2.5 实现 stop-only 直接 None 和新非空请求重置本关节窗口。

## 3. 后续 read/write 与生命周期实现

- [ ] 3.1 Hold read 跳过 GetStatus，保留真实状态，不报主动跳过引起的 missing feedback。
- [ ] 3.2 每关节独立验证首帧，接管优先于超时，统一清空全部命令槽为 NaN。
- [ ] 3.3 用 active write 计数，固定前 50 次静默、第 51 次单次超时日志，计数饱和。
- [x] 3.4 超时后先 EngageBrake，成功或不支持抱闸时每次 CurControl(0.0F, 1)，失败 ERROR 且无零电流；允许迟到首帧恢复目标模式。
- [ ] 3.5 保留最新 None 分支：Released 抱紧成功后和 Engaged/Unsupported 持续零电流反馈；失败 ERROR 且无零电流。
- [ ] 3.6 混合及全 Hold 批次正常结束只 Commit 一次；SDK 失败、命令清理及 inactive 行为保持约束。
- [ ] 3.7 初始化、重配/失败及生命周期退出清除 Hold 和旧代次，保留原退出抱闸行为。

## 4. 后续 Mock/Fake 验收

- [ ] 4.1 验证逐关节、同模式交接、stop-only、None 后认领，以及不等待其他关节。
- [ ] 4.2 验证 prepare/perform 间旧命令、确认周期旧值、清空后有效首帧和非法首帧。
- [ ] 4.3 验证第 50/51 次边界、read 不计数、确认不重置、未确认超时、边界及超时后首帧优先。
- [ ] 4.4 验证超时日志每关节每代次一次、持续零电流、计数饱和、零电流及 Commit 异常。
- [ ] 4.5 验证 Hold 无 GetStatus/TTL 消费、状态保持、退出恢复；其他关节正常读取。
- [x] 4.6 验证 Hold 前 50 次不改变抱闸、超时抱紧 Released 且不重复，失败不发零电流；接管时 Released 直接控制、Engaged 按有效首帧顺序松闸。
- [ ] 4.7 验证最新 None 的 Engaged 长期零电流反馈、Released 抱紧成功/失败及过渡状态。
- [ ] 4.8 验证正常/None/Hold 混合及全 Hold 一次 Commit，不修改 SDK 双耦合协调。
- [ ] 4.9 验证非法请求原子性、新旧代次交错、旧确认隔离和生命周期清理后不继承旧请求。

## 5. 后续实现交付检查

- [ ] 5.1 更新 README，说明 Hold/None 区别、NaN 仅指命令、50 周期含义及超时恢复。
- [ ] 5.2 使用已安装 SDK 完成包构建、相关 C++/Python 测试、clang-format、clang-tidy 和适用的 Xacro 检查。
- [ ] 5.3 运行 OpenSpec 全量严格校验，记录实际软件验证结果；不以文档校验代替实现验证。

## 本次文档验证记录（2026-10-22）

- `openspec validate add-controller-switch-hold-mode --strict --no-interactive`：通过。
- `openspec validate --all --strict --no-interactive`：6 项通过，0 项失败。
- 差量规格包含 8 条 requirement、42 个 scenario；包含新版 None 持续反馈回归场景。
- `git diff --check` 及四个新增 Markdown 的 `git diff --no-index --check`：通过。
- 本次新增四个 OpenSpec 文件；未改写用户已有的三个 adapt 文件，未修改或验证 C++ 实现。

## Hold 超时抱闸验证（2026-10-26）

- 超时先调用 EngageBrake，确认 Engaged 或 Unsupported 后才发送零电流；抱闸返回失败或异常时返回 ERROR。
- 新增 2 个回归测试通过：首次超时抱紧、持续反馈不重复抱紧、迟到首帧保持后松闸接管，以及抱闸 false/异常不发送零电流。
- 包构建成功；C++/Python 共 104 项测试，103 项通过，1 项已有失败。
- 已有失败为 RealSdkCoupledPairCachesBrakesAndPreservesPerMotorCanfd 的激活后命令记录为空断言；
  在修改前的 641597e 独立构建中复现同一断言失败，未修改该测试。
- 修改文件 clang-format --dry-run --Werror、git diff --check、OpenSpec 全量严格校验通过。
- clang-tidy 无错误，仅报告原有 on_init(HardwareInfo) 弃用告警；本次验证使用 Mock/Fake，未启动真实硬件。
