# 联机模式架构（v4 · 简化定稿）

状态：设计稿，分支 `feat/online-lan` · 关联：[protocol.md](protocol.md) · [decisions.md](decisions.md) · [roadmap.md](roadmap.md)

> **v4 说明**：v3 加了输入复制、冲击同步、tick 软同步、拉回阈值——那是补丁堆。v4 回到 CS2 一类现代竞技游戏的骨架，**四件事**：
> ① 主机是唯一权威；② 客户端只预测自己的车；③ 别人的车按快照插值；④ 低频事件走可靠通道。
> 其余机制全部删除（见 [decisions.md](decisions.md) 附录"已撤销"）。

---

## 1. 骨架

```
        主机（也是玩家）                          客户端（1..7）
  ┌──────────────────────────┐          ┌──────────────────────────┐
  │ 模拟全部车 · 240 Hz       │           │ 模拟 1 台车（自己的）      │
  │ 裁决：圈/排名/完赛/损伤    │  快照 ↓    │ 预测 + 回滚校正            │
  │ 30 Hz 快照 · 事件          │ ────────▶ │ 别人的车：插值 + 碰撞代理   │
  └──────────────────────────┘  输入 ↑    └──────────────────────────┘
                                 60 Hz
```

| 对象 | 主机 | 客户端 |
|---|---|---|
| 静态赛道碰撞体 | 有 | 有（自己的车要撞墙） |
| 自己的车 | 权威模拟 | **本地预测**（输入立即生效） |
| 别人的车 | 权威模拟 | 不模拟；按快照插值，另加一个只挡不推的碰撞代理 |

**单一权威不是补丁，而是删逻辑**：不用协商、不用收敛分歧、不用每人权威的状态切换；圈数/接触/损伤只有一个来源。每人权威自己车会带来 N 份世界 + N 套重同步，而且"谁先过线"没有裁决者——那才是过度设计。

CS2 对照：服务器权威 / 客户端预测自身移动 / 其他玩家插值 / 可靠事件。本作一一对应，唯一增量是远端车的**碰撞代理**（车会被撞到，不能被穿过去），成本约 20 行。

---

## 2. 延迟

| 指标 | 值 | 说明 |
|---|---|---|
| 我的车：操控响应（均/p95） | **≈19 / ≈36 ms** | 与同帧率单机一致（差 ≤2 ms，验收 A5） |
| 我的车：校正频率 | 30 Hz（快照） | 回滚重放，误差不累积 |
| 别人的车：视觉延迟 | **RTT/2 + 插值缓冲**（LAN 约 25–50 ms） | 与 CS2 中"别人的位置"同一取舍（N-05） |
| 网络端到端 | 有线 p95 ≤ 5 ms | NF-03 |

预测只做一件事：**把网络往返从我的响应通路里摘掉**。别人的车仍然是"稍微过去的位置"，这是竞技游戏的标准做法——玩家的操作依据本来就是自己屏幕上看到的画面，不是主机真值。

---

## 3. 传什么

| 类 | 内容 | 方向 | 传法 | 频率 | 尺寸 |
|---|---|---|---|---|---|
| C0 | 发现 / 握手 / 名单 / 规则 | 双向 | 可靠 | 事件 | 27–161 B |
| C1 | 输入（3 轴 + 按钮 + 边沿计数 + ack） | 客户端 → 主机 | 不可靠 + 3 帧冗余 | 60 Hz | 22 B |
| C2 | 快照（8 车热状态 + 自车全状态块 + 相位/时钟） | 主机 → 客户端 | 不可靠 sequenced | 30 Hz | 345 B |
| C3 | 事件（圈/完赛/结算/损伤/复位/名单） | 双向 | 可靠有序 | 事件 | 4–91 B |

带宽（8 人满房）：每客户端下行 **11.2 KB/s**、上行 **3.0 KB/s**；主机出口 **90 KB/s**。

**不传**：赛道几何、碰撞体、车辆参数常量表、别人的慢变状态、任何客户端成绩 → 用 `buildSig + mapHash` 校验代替。

细节（字段、量化、位布局）见 [protocol.md](protocol.md)。

---

## 4. 我的车：预测与校正

**每 tick（240 Hz）**：取最新输入 → `ApplyInputMapping()`（与主机同一份实现：`drsZone`、季节门控）→ `InputSmoother`（两端同一 tau）→ `Vehicle::Update` + `world.Step` → 记历史 `{tick, seq, CarInput}`（32 tick 环）。

**收到快照**（含 `appliedInputSeq` + 自车全状态块）：

1. 在历史里找 `seq == appliedInputSeq` 的 tick `T_a`；找不到（断流 >133 ms）→ 直接 `RestoreState` + 大 tau；
2. `RestoreState(权威状态)`；
3. 重放 `T_a+1 … 现在` 的输入（上限 16 tick）；
4. 校正前后的位姿差交给渲染层做平滑（τ=80 ms；>2 m 或 >15° 直接吸附）。

这是 CS2 的"重放未确认指令"，唯一的本作特化是状态恢复需要 M0 的 `SaveState/RestoreState`（`Vehicle::Reset` 会清零胎耗/ERS/损伤，不能当回滚用）。

**没有的东西**：没有"限幅"、没有误差死区、没有预测置信度衰减。物理永远等于"权威状态 + 我的输入"，画面用偏移吸收跳变——一层就够。

---

## 5. 别人的车：插值 + 碰撞代理

1. **插值**：维护快照环，渲染时间 = 最新快照时间 − 缓冲；缓冲自适应 33–150 ms（LAN 默认 ~40 ms = 2 帧间隔 + 余量），抖动大时自动加长。
2. **碰撞代理**：每台远端车在本地世界里有一个 `mass = 0` 的物理体，每 tick 用插值位姿 `SetTransform` 推到那个位置。
   - 我的车撞上去会被挡/弹开 —— 因为在我屏幕上车就在那里，这是玩家唯一能依据的信息；
   - 代理体本身不会被推动（它是静态体），所以**不存在"本地算出来的远端状态又要被覆盖"的问题**——远端车从头到尾只有主机一个作者。
3. **主机的裁决**：主机用真实输入解算两台车，把结果放进快照。我这边如果因为插值差异算得不一样，自车预测会被正常校正（§4），远端车由插值直接跟随权威，二者都不会"回弹"。

这套结构没有 `ContactSync`、没有冲量阈值、没有双向拉回——因为远端车在本地根本没有第二个作者。

---

## 6. tick 与线程

| 时钟 | 频率 | 说明 |
|---|---|---|
| `hostTick` | 240 Hz | 主机权威时间线（只用于标注与统计） |
| `localTick` | 240 Hz | 客户端本地模拟，各自单调，**不做时钟同步** |

客户端用**输入序号**与权威对齐（`appliedInputSeq`），不需要共享时间线——因为别人的车不由我们模拟。这是 v3 的 tick 软同步被删掉的原因。

线程（与现有一致，无新增线程类型）：

| 数据流 | 生产 | 消费 | 机制 |
|---|---|---|---|
| 远端输入（主机） | 主线程收包 | 仿真线程 | 每 peer 邮箱（沿用 `PlayerIntent` 模式，`Game.h:286-294`） |
| 快照 → 校正 | 主线程 | 仿真线程 | 单槽 mailbox（只留最新） |
| 事件 | 仿真线程 | 主线程 | `EventBus`（同 `CollisionBus` 约定） |
| 快照外发 | 主线程读 `frameSnap_` | 网络 | `CopyFront()` |

---

## 7. 会话

| 主机 | 客户端 |
|---|---|
| `Idle → Advertising → Lobby → Loading → Countdown → Racing → Results → Lobby` | `Idle → Searching → Joining → Lobby → Loading → Countdown → Racing → Results` |

`RacePhase` 只新增一个 `Lan`；联机期 `phase_/phaseTimer_/raceClock_` 只有主机写；比赛期禁暂停（ESC = 离开确认）；重开仅主机；结束条件 = 全员完赛或主机终止；Racing 期 `Hello` 拒绝；断线标 `RETIRED` 不阻塞结束。

### 7.1 身份与座位

- **玩家身份**：`PlayerId = FNV-1a 64(规范化名字)`（`src/net/identity.h`）。名字来源优先级：`--player` > `[net] player_name` > 本机登录页（引擎 `TextField`）> 角色默认名（`HOST` / `PLAYER-<n>`，无人值守路径用）。
- **座位**：`seat` = 花名册按 `playerId` 升序的序号（`src/net/roster.*`），**主机也只是一个 seat**。开赛时主机 `FreezeRoster()` 算一次并随 `rules` 下发；会话槽位同时重编号为 seat，于是快照自车块 / `appliedInputSeq` / 输入投递 / `RacerBySlot()` 共用一套编号。
- **车与座位绑定**：`Racer{playerId, seat}`；`LocalRacer()` / `LocalSeat()` 是所有 HUD / 相机 / 音频 / 输入 / 预测 / 结束判定的唯一入口（离线 seat = 0）；`GridPose(track, racer)` 是唯一的格位公式（出生、Enter 重置、联机 RaceStarted 重置都走它）。
- **槽位空洞**：只可能来自"开赛前有人退出"（如 {1,3}）。座位只按 `playerId` 排序生成，空洞不会把谁挤到别人的格位；远端输入按 slot 投递并容忍空洞。
- **冲突**：规范化同名（或哈希碰撞）→ 主机 `Reject(DuplicatePeer)`，不自动改名；需要换 id 的一方用 `MakePlayerId(name, variant)` 追加 `#<n>` 盐重哈希。
- **物理一致性**：客户端用主机 `rules` 覆盖赛季门控（含 ERS 位）、辅助等级与伤害开关，避免两个赛季的物理同场。

---

## 8. 异常（只留必要的）

| 场景 | 处理 |
|---|---|
| 快照断流 > 250 ms | 别人的车冻结在最后位姿 + "连接不稳定"角标；我的车继续可开 |
| 断流 > 3 s | 回 Searching |
| 某 peer 输入断流 > 250 ms | 该车输入归零（不让幽灵全油门） |
| peer 超时（ENet 5 s） | `RETIRED` + `PeerLeft` |
| 版本/构建/地图不符 | `REJECT(reason)` |
| 主机退出 | 回主菜单 |

---

## 9. 验收（详见 roadmap §2）

A1 满房 60 s 无掉线 · A2 `RaceEnd` 逐字段一致 · A3 带宽 ≤15 / ≤4 KB/s · A4 网络 p95 · **A5 我的车与单机差 ≤2 ms** · A6 远端插值无跳变、代理可撞不可穿 · A7 房间码过滤 · A8 断线 · A9 `--drive-test` 0 failures / `--bench` 不变 · A10 客户端只有 1 台被模拟的车。

---

## 10. 要动的代码

| # | 位置 | 改动 |
|---|---|---|
| 0 | `rush-engine/modules/physics.cppm` | **M0**：`VehicleSnapshotState` + `SaveState/RestoreState`（纯增量） |
| 1 | `Game.h:34-49` `Racer` | 加 `kind`（Local/Remote） || 2 | `Game::Init`（`Game.cpp:187-205`） | 按角色建会话；客户端建世界、只 spawn 自己的车 |
| 3 | `Game::Update`（`Game.cpp:435-514`） | `net.Poll` → 输入 → `SubmitLocalInput` → 相位 → `CopyFront` |
| 4 | `Game::UpdateSim`（`Game.cpp:524-569`） | 每 tick 预测自车；远端代理体跟随插值位姿 |
| 5 | `ApplyPlayerIntent`（`Game.cpp:729-798`） | 抽 `ApplyInputMapping()`，两端共用 |
| 6 | 新增 `InputSmoother` | 两端同一实现与 tau |
| 7 | `PublishSnapshot`（`Game.cpp:571-604`） | 加 `hostTick`，供网络节流与自车全状态块取值 |
| 8 | `Render3D`（`Game.cpp:645-677`） | 自车 = 快照 + 平滑偏移；他车 = 插值位姿 |
| 9 | `UpdateRace`（`Game.cpp:963-1039`） | 抽 `IRaceAuthority`（Local/Host/Remote） |
| 10 | `UpdatePlayerInput` / `StartSimWorker` | 白名单与停机条件加 `Lan` |
| 11 | `RacePhase`（`Game.h:25-32`） | 加 `Lan` |
| 12 | `Track::Build`（`Track.cpp:189-204`） | 客户端走完整路径（含静态碰撞体） |
| 13 | `main.cpp` / `CMakeLists.txt` | `--net-host/--net-join`；`APEX_ENABLE_NET` |

不动：`third_party/`、L1、任何现有公开接口语义。

---

## 11. 配置（`[net]`）

`player_name` · `room_code` · `input_hz`(60) · `snapshot_hz`(30) · `interp_buffer_ms`(40，自适应上限 150) · `history_ticks`(32) · `smooth_tau_ms`(80) · `snap_threshold_m`(2.0) · `max_players`(8) · `game_port`(45998) · `discovery_port`(45999) · `enable`(true)。
