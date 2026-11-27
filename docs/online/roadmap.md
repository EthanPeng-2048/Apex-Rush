# 联机模式落地路线（v4）

状态：设计稿 · 关联：[architecture.md](architecture.md) · [protocol.md](protocol.md) · [decisions.md](decisions.md)

范围：纯真人 1–8，主机权威；我的车本地预测、别人的车插值；不做 AI / 中途加入 / 重连（ADR-09）。

---

## 1. 里程碑与门禁

### M0 · 子模块：车辆状态存取（硬前置）

`VehicleSnapshotState` + `SaveState()/RestoreState()`（`rush-engine` 分支 `feat/vehicle-snapshot-state`）。

| 门禁 | 判定 |
|---|---|
| 逐位一致 | 存档 → 跑 10 tick → 恢复 → 同输入重跑 10 tick = 不中断跑 20 tick |
| 不破坏既有 | `--drive-test` 三后端 0 failures；`--bench 100` 分段计时 ±1 % |
| 最小面 | 只加 2 函数 + 1 结构体，`Reset` 语义与调用路径不变 |

> 跨仓库推送属共享状态操作，实施前先确认。

### M1 · 协议层（纯函数）

`wire.h` / `wire_encode` / `wire_decode` / `wire_quantize`。

| 门禁 | 判定 |
|---|---|
| 尺寸断言 | Input 22 · Snapshot(8) 345 · Hello 27 · Welcome 60 · LobbyUpdate(8) 161 · RaceEnd(8) 91 |
| 往返 | 随机 10 万组 `Decode(Encode(x)) == x` |
| 坏输入 | 空包/截断/长度撒谎/越界/保留位 → `Malformed`，无崩溃 |
| 金标准 | 每报文一组固定字节向量 |

### M2 · 传输抽象

`ITransport` + `EnetTransport` + `LoopbackTransport`（注入延迟/抖动/丢包/重复）+ `NullTransport`。
门禁：三种可靠性语义一致、10 万包压力、`PayloadTooLarge`、100 次连接循环后 peer 归零。

### M3 · 发现层

`IRoomDiscovery` + `UdpDiscovery` / `StaticDiscovery` / `NullDiscovery`。
门禁：3 s 内可见、房间码过滤（不匹配不显示）、`QUERY` 单播兜底、TTL 过期、同机双开、多网卡。

### M4 · 会话层

`INetSession` + `HostSession` / `ClientSession` / `net_factory`。
门禁：FSM 覆盖（六种拒绝、连接超时、`HostGone`）、名单一致、请求重发只接受一次、`FakeClock` 全绿 <2 s。

### M5 · 输入与自车预测

`InputSample` 上报、`ApplyInputMapping()` 抽取、`InputSmoother`、`RemoteInputMailbox`、`input_history`。

| 门禁 | 判定 |
|---|---|
| 映射唯一 | 两端调用同一函数（`drsZone`/季节门控/三种边沿） |
| 平滑一致 | 同采样 + 同 tau → 两端 `CarInput` 序列逐 tick 相等 |
| 回滚正确 | 0 ms 延迟：校正后与主机真值逐位相等；80 ms 延迟：重放 ≤16 tick |
| 延迟（A5） | 与同帧率单机 p95 差 ≤2 ms；`predict=false` 对照必须变差 |
| 断流 | 某车输入停 300 ms → 该车输入归零 |

### M6 · 别人的车：插值 + 碰撞代理

快照环插值、自适应缓冲、`mass = 0` 代理体（`SetTransform` 跟随）、`Render3D` 双来源合成。

| 门禁 | 判定 |
|---|---|
| 平滑 | 抖动 20 ms + 丢包 5 %：帧间位移无跳变、无穿地 |
| 代理 | 撞上远端车会被挡/弹，**不穿透**；远端车不被推动 |
| 延迟 | LAN 缓冲 ~40 ms；拔出网线 250 ms 后冻结并提示 |
| 带宽（A3） | 8 客户：下行 ≤15 KB/s、上行 ≤4 KB/s |

### M7 · 规则权威与结算

`IRaceAuthority`（Local/Host/Remote）、圈/排名/完赛/损伤/复位/退赛。

| 门禁 | 判定 |
|---|---|
| 一致 | `RaceEnd` 各端 `lap/position/raceTime/bestLap` 逐字段相等 |
| 结束 | 全员完赛结束；一人断线不阻塞；主机手动结立即生效 |
| 重开 | 客户端 ENTER 无效；主机重开 → 全员回 Countdown、时钟归零 |

### M8 · UI 与收口

LAN 菜单（搜索/房间码滚轮/列表/大厅）、HUD 提示、统计叠层（RTT/丢包/缓冲深度/重放深度）、`--net-host` / `--net-join` / `--net-port` / `--net-selftest`。

| 门禁 | 判定 |
|---|---|
| 满房 | 8 客户端 60 s 无掉线、无幽灵车、贴身缠斗可玩 |
| 回归（A9） | `--drive-test` 0 failures；`--bench 100` 不变 |
| 隔离 | 不进联机时无 socket；`enable=false` 无联机入口 |
| WASM | `cmake --preset web` 仍可构建、浏览器单机可玩 |

---

## 2. 验收

| # | 标准 | 判定 |
|---|---|---|
| A1 | 8 客户端进入 `Racing`，60 s 无掉线 | 满房实测 |
| A2 | `RaceEnd` 逐字段相等 | 各端导出载荷比对 |
| A3 | 下行 ≤15 KB/s、上行 ≤4 KB/s | `PeerStats` |
| A4 | 网络端到端 p95：有线 ≤5 ms、Wi-Fi ≤25 ms | `PeerStats.rttMs` |
| **A5** | **我的车：操控响应 p95 与单机差 ≤2 ms** | 240 fps 录屏；必测 `predict=false` 对照 |
| A6 | 别人的车：插值无跳变；代理可撞不可穿 | 录屏 + 抖动注入 |
| A7 | 房间码不匹配的房间不出现 | UI + 单测 |
| A8 | 断线 3 s 内提示、`RETIRED`、不阻塞结束 | 拔网线 |
| A9 | `--drive-test` 0 failures、`--bench` 不变 | CI 门禁 |
| A10 | 客户端只有 1 台被模拟的车 | 启动断言（`Vehicle` 实例数 = 1，代理体 = N−1） |

---

## 3. 测试

| 编号 | 类型 | 覆盖 |
|---|---|---|
| T-1 | 单元（纯函数） | 量化/编解码/金标准/坏输入/尺寸 |
| T-2 | 单元（子模块） | 状态存取逐位一致 |
| T-3 | 单元（会话） | FSM/拒绝/超时/恰好一次（`LoopbackTransport` + `FakeClock`） |
| T-4 | 收敛（单进程） | 注入 0/30/80 ms：重放深度、校正幅度、无振荡 |
| T-5 | 集成（双进程） | `--net-host/--net-join` 真 ENet 回环，退出码门禁 |
| T-6 | 回归 | `--drive-test`、`--bench 100` |
| T-7 | 实测 | 录屏量 A5/A6 |
| T-8 | 手工 | 实机 3 台 + 交换机 + Wi-Fi：满房/拔线/防火墙/贴身缠斗 |

T-5 不并入 ctest 默认集（需端口），用 `--net-selftest` 显式触发。

---

## 4. 风险

| 风险 | 对策 |
|---|---|
| 子模块状态存取写漏 → 回滚持续偏差 | M0 逐位一致门禁 + `--drive-test` 0 failures |
| 插值缓冲不当 → 抖动或延迟过大 | 自适应 33–150 ms；`interp_buffer_ms` 可配；A6 门禁 |
| 代理体位置更新漏帧 → 撞到"幽灵墙" | 代理只在有插值样本时启用，断流即冻结并提示 |
| 主机 CPU 8 车 | `--bench 100` 余量；`snapshot_hz` 可降 |
| 防火墙拦广播 | `QUERY` 单播兜底 + 首启提示 |
| 范围蔓延（AI / 中途加入 / 分布式权威） | ADR-09 + 附录"已撤销"清单，改动需新增 ADR |

---

## 5. 文档落地（实现阶段）

| 动作 | 文件 | 时机 |
|---|---|---|
| 改写 §7 指向本设计 | `docs/architecture.md:115-122` | M1 |
| 文档索引加一行 | `README.md:83-94` | M1 |
| `[net]` 键说明 | `README.md` 参数表 | M8 |
| 状态改「已实施」+ 补 A3/A4/A5/A6 实测 | 本文与 protocol 头部 | M8 通过后 |
