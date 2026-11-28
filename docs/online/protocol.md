# 联机协议（v4 简化定稿 · protoVer 2）

状态：设计稿 · `protoVer = 2` · 关联：[architecture.md](architecture.md) · [decisions.md](decisions.md)
本文件定义线上字节。实现必须与本文逐字节一致；改动一律升 `protoVer`。

> **protoVer 2（身份与座位）**：每条握手报文都带 8 B `playerId`（FNV-1a 64 over 规范化名字，小端），`rules` 带上**冻结的花名册**（每席位 `playerId + seat + name`）。`Rules` 因此从 10 B 变成 211 B，`Hello`/`Welcome`/`Announce`/`RaceStart` 同步变大；`Input` 与 `Snapshot` 布局不变。

---

## 1. 传输

| 项 | 值 |
|---|---|
| 框架 | ENet over UDP（仓库已 vendored 并链接，`CMakeLists.txt:58-59,75`） |
| 通道 0 | 不可靠：输入用 `UNSEQUENCED`，快照用 `UNRELIABLE_SEQUENCED` |
| 通道 1 | `RELIABLE` 有序：握手与全部事件 |
| 单包上限 | 1200 B（< ENet MTU，永不分片） |
| 位序 | LSB-first，显式 little-endian |
| 端口 | 游戏 45998，发现 45999（可配） |

**两个通道的理由**：全可靠会让输入/快照排队重传（队头阻塞把 2 ms 变成上百 ms）；全不可靠会丢圈成绩与结算。

---

## 2. 报文总表

头部统一 `protoVer u8` + `msgType u8`。

| ID | 名称 | 方向 | 通道 | 频率 | 尺寸 |
|---|---|---|---|---|---|
| `0x10` | `Input` | 客户端 → 主机 | 0 / unseq | 60 Hz | **22 B** |
| `0x20` | `Snapshot` | 主机 → 客户端 | 0 / seq | **30 Hz** | **345 B**（8 车） |
| `0x40` | `Hello` | 客户端 → 主机 | 1 / rel | 一次 | 35 B |
| `0x41` | `Welcome` | 主机 → 客户端 | 1 / rel | 一次 | 269 B |
| `0x42` | `Reject` | 主机 → 客户端 | 1 / rel | 一次 | 4 B |
| `0x43` | `LobbyUpdate` | 主机 → 广播 | 1 / rel | 名单变化 | 3 + 20·N |
| `0x50` | `RaceStart` | 主机 → 广播 | 1 / rel | 每场 | 219 B |
| `0x51` | `LapComplete` | 主机 → 广播 | 1 / rel | 每圈每车 | 12 B |
| `0x52` | `PlayerFinished` | 主机 → 广播 | 1 / rel | 每车 | 12 B |
| `0x53` | `RaceEnd` | 主机 → 广播 | 1 / rel | 每场 | 3 + 11·N |
| `0x54` | `CarRecovered` | 主机 → 广播 | 1 / rel | 事件 | 4 B |
| `0x55` | `PartDamaged` | 主机 → 广播 | 1 / rel | 事件 | 6 B |
| `0x56` | `PeerLeft` | 主机 → 广播 | 1 / rel | 事件 | 4 B |
| `0x57` | `SessionEnd` | 主机 → 广播 | 1 / rel | 一次 | 3 B |
| `0x59` | `Restart` | 主机 → 广播 | 1 / rel | 事件 | 8 B |
| `0x58` | `Bye` | 客户端 → 主机 | 1 / rel | 一次 | 2 B |

---

## 3. 发现（N-01 / N-02 / N-03）

独立 UDP socket 绑定 `discoveryPort`（`SO_REUSEADDR`，允许同机双开）。

| 项 | 值 |
|---|---|
| 广播 | 主机 1 Hz 发 `ANNOUNCE` 到广播地址 |
| 查询 | 客户端进联机界面即发 `QUERY`（之后每 2 s），主机**单播**回 `ANNOUNCE` |
| 过期 | 3 s 未刷新即从房间表移除 |
| 过滤 | `magic == "APXR"` 且 `discoVer`、`protoVersion` 相同；`roomCode == 0` 表示不过滤 |

### `ANNOUNCE`（70 B）

`magic[4]"APXR"` · `discoVer u8=1` · `msgType u8=1` · `protoVersion u8` · `buildSig u32` · `mapHash u32` · `roomCode u16` · `gamePort u16` · `players u8` · `maxPlayers u8` · `phase u8` · `mapName char[24]` · `hostName char[16]` · `hostPlayerId u64`

### `QUERY`（8 B）

`magic[4]` · `discoVer u8` · `msgType u8=2` · `roomCode u16`（0 = 全部）

房间码：4 位十进制（0000–9999），UI 用 **4 位数字滚轮**输入（←/→ 换位、↑/↓ 调值），不引入文本输入系统。码不匹配的房间**完全不显示**。房间码不做鉴权，兼容性由 `buildSig + mapHash + 容量 + 相位` 判定。

---

## 4. 握手

```
客户端                                主机
 Connect(gamePort)  ─────────────▶
 Hello{protoVer, buildSig, mapHash, name, color, playerId}  ──▶  校验版本→构建→地图→相位→容量→身份
                    ◀─────────────  Welcome{peerId, slot, sessionId, rules, mapHash, playerId}
                    ◀─────────────  LobbyUpdate
 …大厅等待…          ◀─────────────  RaceStart{startHostTick, countdownCs, rules(含冻结花名册)}
```

- `Hello`（35 B）：`buildSig u32` · `mapHash u32`（无该地图填 0）· `name char[16]` · `colorPref u8` · `playerId u64`
- `Welcome`（269 B）：`peerId u8` · `slot u8` · `sessionId u32` · `rules`（见 §4.1）· `mapHash u32` · `mapName char[24]` · `gridPos f32×3` · `gridYaw u16` · `playerId u64`
- `Reject`（4 B）：`reason u8`（1 协议 / 2 构建 / 3 地图 / 4 满房 / 5 比赛中 / 6 重复身份 / 7 内部）· `detail u8`
- `LobbyUpdate`（3 + 20·N）：`count u8` + 每项 `peerId u8, slot u8, name char[16], color u8, ready u8`
- `RaceStart`（219 B）：`startHostTick u32` · `countdownCs u16` · `rules`（见 §4.1）

### 4.1 `rules`（211 B）

`laps u8` · `carCount u8` · `damage u8` · `seasonBits u8`（bit0 DRS / bit1 active aero / bit2 overtake / bit3 ERS）· `absLevel u8` · `tcLevel u8` · `escLevel u8` · `steerLevel u8` · `physicsEngine u8` · `snapshotHz u8` · **花名册**：`rosterCount u8` + 定长 `kMaxCars=8` 条 `playerId u64, seat u8, name char[16]`（共 8×25 = 200 B，未用席位写 0，解码侧不信任长度字段）。

### 4.2 身份与座位

- **playerId** = FNV-1a 64 over 规范化名字（去首尾空白、转小写、截到 15 B）。名字来源优先级：`--player <name>` > `[net] player_name` > 本机登录页输入 > 按角色的默认名（`HOST` / `PLAYER-<n>`）。
- **seat** = 花名册按 `playerId` 升序排列后的序号（0..N-1）。主机也是一个 seat，**不再假定主机 = 0**。座位号同时就是发车格号。
- **冻结时机**：主机开赛前算一次（`FreezeRoster`），随 `rules` 下发；P0 不支持中途加入，整场比赛不变。假设仅"开赛前有人退出"，于是会话槽位可能出现空洞（如 {1,3}）；座位只按 `playerId` 排序生成，所以空洞不会把谁挤到别人的格位。
- **冲突策略**：规范化后同名（或 64 位哈希真碰撞）会让两个玩家算出同一个 `playerId`。主机**直接拒绝**第二个（`Reject(6)`），不做自动改名 —— 撞名只花一次改名，绝不出现两人共享一台车。需要主动换 id 的一方可调 `MakePlayerId(name, variant)` 追加 `#<n>` 盐再哈希（两端规则相同，确定性）。
- **槽位即座位**：冻结后主机把每个 peer 的会话槽位重编号为它的 seat，于是快照自车块、`appliedInputSeq`、输入投递、`RacerBySlot(seat)` 共用同一个编号，不再有第二套映射。

发车格不传输：双方都用 `Track::StartPosition(seat)`（`Track.cpp:662-672`）本地重算，签名已校验 → 一致。

---

## 5. 输入 `Input`（22 B）

客户端 → 主机，通道 0 `UNSEQUENCED`，60 Hz，携带最近 **3 帧**。

| 偏移 | 字段 | 类型 | 说明 |
|---|---|---|---|
| 0 | `protoVer`, `msgType` | u8, u8 | |
| 2 | `firstSeq` | u16 | 最新帧序号 |
| 4 | `count` | u8 = 3 | 冗余帧数（先旧后新） |
| 5 | `frames[3]` | 3 × 4 B | 见下 |
| 17 | `drsPress` `wingPress` `overtakePress` | u8 ×3 | 单调计数 |
| 20 | `ackSnapshotSeq` | u16 | 已收到的最大快照序号 |

单帧 4 B：`throttle u8`(1/255) · `brake u8`(1/255) · `steer i8`(1/127) · `buttons u8`（bit0 手刹 / bit1 倒车 / bit2 lookBack / bit3–7 保留写 0）

语义：

- **轴是电平**：主机取最新值并在 1–2 tick 内线性趋近（`InputSmoother`，两端同一实现）。丢包只让旧值多活 16–33 ms。
- **边沿是计数**：主机按有符号差分 `(int8_t)(now - lastSeen)` 处理回绕，只应用一次 → 冗余重发下恰好生效一次。
- 客户端不上报位置/圈数/损伤——报文结构本身就是 N-04 的落地。
- `buildSig = fnv1a(版本串 ‖ protoVer ‖ VehicleParams 默认值摘要)`：常量被改过的两方禁止同场。

---

## 6. 快照 `Snapshot`（345 B）

主机 → 客户端，通道 0 `UNRELIABLE_SEQUENCED`，默认 30 Hz。

### 6.1 包头（30 B）

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0 | `protoVer`, `msgType` | |
| 2 | `hostTick` | u32，240 Hz 计数 |
| 6 | `snapSeq` | u16，插值排序与 ack |
| 8 | `phase` | u8 |
| 9 | `phaseTimerCs` | u16（1/100 s） |
| 11 | `raceClockMs` | u32 |
| 15 | `carCount` | u8 |
| 16 | `anchor` | f32 ×3，本包 `pos` 的相对原点 |
| 28 | `appliedInputSeq` | u16，**本包状态所反映的**收包方输入序号（回滚界标）：由仿真线程在构建 `HostWorld` 时逐 seat 填入，随状态一起走，客户端从该 tick 起重放未确认输入 |

### 6.2 每车（35 B × 8）

`slot u8`（= **seat**，见 §4.2）· `pos i16×3`（相对 anchor，1/32 m，±1024 m）· `yaw u16`（360/65536°）· `pitch i8` `roll i8`（0.25°）· `vel i16×3`（1/64 m/s）· `yawRate i16`（1/16 °/s）· `rollRate i8` `pitchRate i8`（1 °/s）· `steerDeg i8`（0.5°）· `speedKmh u16`（0.05）· `rpm u16`（1）· `flags1 u8`（`gear`4bit / `wingMode` / `drsActive` / `ersDeploying` / `finished`）· `ersCharge u8`（1/255）· `flags2 u8`（`groundedMask`4bit / `drsAvailable` / `retired` / `onTrack` / 保留）· `lap u8` · `position u8` · `wheelTravel u8×4`（1/255 悬挂行程）

> 量化校验：300 km/h ≈ 83 m/s 时 3.1 cm ≈ 0.37 ms 位移；`yaw` 0.0055° 在 4 m 车宽上端部误差 <0.4 mm。

### 6.3 自车全状态块（35 B，仅收包方自己的车）

用于回滚重放的状态恢复。

`slipRatio i8×4`(1/127) · `slipAngleDeg i8×4`(0.5°) · `wheelSpinRate i16×4`(1/16 rad/s) · `wheelWear u8×4` · `wheelTemp u8×4`(1 °C) · `partHealth u8×4`(1/255) · `damageTotal u8` · `shiftCooldownTicks u8` · `ersDeployTicks u8` · `rngState u32`

总计：30 + 35×8 + 35 = **345 B**。

> 悬挂长度/载荷/接触法线**不传**——它们每 tick 由射线与弹簧重算，不是积分状态。传"积分状态 + 位姿动量"才是最小充分集。

---

## 7. 事件（可靠有序）

| 报文 | 字段 |
|---|---|
| `LapComplete` | `slot u8` · `lap u8` · `lapTimeMs u32` · `bestLapMs u32` |
| `PlayerFinished` | `slot u8` · `position u8` · `raceTimeMs u32` · `bestLapMs u32` |
| `RaceEnd` | `count u8` + 每车 `slot, position, finished, raceTimeMs, bestLapMs` |
| `CarRecovered` | `slot u8` · `reason u8` |
| `PartDamaged` | `slot u8` · `part u8` · `health u8` · `impactKmh u8` |
| `PeerLeft` | `peerId u8` · `reason u8` |
| `SessionEnd` | `reason u8` |
| `Restart` | `startHostTick u32` · `countdownCs u16` |

**恰好一次**：这些事件都由主机在状态迁移的瞬间产生一次（不是每帧轮询），配可靠有序通道即为恰好一次。

---

## 8. 带宽（8 人满房，输入 60 Hz / 快照 30 Hz）

| 方向 | 计算 | 结果 |
|---|---|---|
| 每客户端下行 | (345+28)×30 | **11.2 KB/s（90 kbps）** |
| 每客户端上行 | (22+28)×60 | **3.0 KB/s（24 kbps）** |
| 主机总出口 / 入口 | 8×11.2 / 8×3.0 | **90 KB/s / 24 KB/s** |
| 握手一次性 | `Welcome` 269 + `RaceStart` 219 + `LobbyUpdate`×(3+20·N) | **<2 KB/场** |
| 事件流 | 圈/完赛/损伤 <200 B/s | 可忽略 |

`snapshot_hz=60` 时下行 22.4 KB/s，仍在预算内。

---

## 9. 编解码不变量（单测逐条覆盖）

1. 越界即 `Malformed`，不返回部分填充。
2. 解码必须恰好消费完 payload（长度字段撒谎 → 错误）。
3. 量化钳制到字段量程；`NaN/Inf → 0`（Debug 断言）。
4. `yaw` 解码归一化 `[0,360)`，插值走最短弧。
5. 保留位：写 0、读忽略。
6. 单包 ≤1200 B，超出即 `PayloadTooLarge`，永不分片。
7. `Encode*/Decode*` 纯函数、无全局状态。
8. 序号 u16 回绕，比较用有符号差分；只接受更新者。
9. 每个报文有固定金标准字节向量（防无意改协议不升版本）。
10. 花名册定长：`kMaxCars` 条记录无论实际人数都占满，`rosterCount > kMaxCars` 直接 `Malformed`。
