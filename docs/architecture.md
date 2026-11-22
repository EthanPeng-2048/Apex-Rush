# 系统架构（现行）

状态：**现行真源**（取代根目录旧 `design.md` 架构章节；玩法需求仍可参考 [history/design-v1.md](history/design-v1.md)）  
关联：[input.md](input.md) · [rpc-refactor.md](rpc-refactor.md) · [perf-async-engine.md](perf-async-engine.md) · [history/kvrec-migration.md](history/kvrec-migration.md)

---

## 1. 一览

```
┌─────────────────────────────────────────────────────────────┐
│ L3  apex-rush（src/）                                        │
│     菜单 / 比赛循环 / UI / ENet 会话 / miniaudio               │
│     输入注册表（事件名+手势+键位）· 各 stage Drain(InputBus)    │
│     import rush.engine.{core,physics,render,input,events,…}  │
└────────────────────────────┬────────────────────────────────┘
                             │ 只依赖 rush::engine 公共模块面
┌────────────────────────────▼────────────────────────────────┐
│ L2  rush-engine（子模块）                                     │
│     MapData · Config · Vehicle · IPhysicsWorld · CollisionBus │
│     InputThread · InputRegistry · InputBus（纯事件，无回调）   │
│     Snapshot · SimThread · JobSystem · Raylib/Null 渲染后端   │
└───────┬──────────────────┬──────────────────┬───────────────┘
        │                  │                  │
┌───────▼──────┐  ┌────────▼────────┐  ┌──────▼──────────────┐
│ 物理后端 ×3  │  │ 存储 / 工具链    │  │ 并行与运行时         │
│ qu3e         │  │ kvrec(.rushmap/ │  │ exstl ThreadPool    │
│ nudge        │  │  .rushconf)     │  │ JobSystem           │
│ rush-physics │  │ mapconv (tools) │  │ SnapshotTripleBuffer│
│ -core (rpc*) │  │                 │  │ SimThread · InputThread│
└──────────────┘  └─────────────────┘  └─────────────────────┘
```

游戏永不直接 `#include` L1 头；物理后端经 `IPhysicsWorld` 多态替换。

---

## 2. 目录与所有权

| 路径 | 层 | 所有权 |
|------|----|--------|
| `src/` | L3 | 玩法规则、菜单、网络会话、音频接入、**输入事件注册与消费** |
| `rush-engine/modules/*.cppm` | L2 接口 | 对外 API（不暴露 kvrec/exstl 类型） |
| `rush-engine/src/` | L2 实现 | `vehicle_impl` · `*_impl` 适配 · `collision_bus` · `input_*` |
| `rush-engine/third_party/exstl` | L1 | `Result` + ThreadPool |
| `rush-engine/third_party/kvrec` | L1 | 二进制 KV 记录（地图/配置） |
| `rush-engine/third_party/{qu3e,nudge,rush-physics-core}` | L1 | 碰撞内核；**无车体** |
| `third_party/enet` | L3 依赖 | 主机权威 UDP |
| `tools/mapconv` | 工具 | CSV 目录 → `*.rushmap`（Release 不构建） |

---

## 3. 引擎模块面（C++20 named modules）

| 模块 | 职责 |
|------|------|
| `rush.engine.core` | 数学类型、`MapData` / `LoadMapData` / `ListMaps`、`Config`（KVRec 落盘）、路径 |
| `rush.engine.physics` | `IPhysicsWorld`、`Vehicle`、`BodyDesc`/`RayHit`/`ContactEvent`、后端工厂 |
| `rush.engine.render` | 抽象渲染面 + Raylib / Null 实现（**不含输入**）；`Vehicle::RenderPose` 无世界快照绘制 |
| `rush.engine.input` | 独立 **input 线程** + `InputRegistry`（模块自注册事件名/手势/多键）+ **`InputBus` 纯事件**（无回调）+ `InputSystem`/`Consume`；`[input]` 配置换键；详见 [input.md](input.md) |
| `rush.engine.events` | `EventBus` 通用队列 + `CollisionBus`：物理只 `Publish`，游戏 stage `Drain`（无生产者回调） |
| `rush.engine.job` | `JobSystem`：`Configure(0)` 串行确定性；N=exstl 池并行 |
| `rush.engine.runtime` | `SimThread` + `SnapshotTripleBuffer`；比赛期仿真与渲染重叠 |

游戏侧入口示例：`import rush.engine.core; import rush.engine.physics; import rush.engine.input; ...`

---

## 4. 异步运行时

1. **事件总线**：接触只进 `CollisionBus`，游戏在独立 stage 消费——模块间无阻塞等待；**输入同构**：input 线程只 `Publish` 到 `InputBus`，菜单/比赛 stage 各自 `Drain`（见 [input.md](input.md)）。
2. **输入线程**：设备采样与手势识别在 **独立 input 线程**，不绑渲染帧；WASM / `--drive-test` 退化为帧内 `PumpOnce`（无 pthread）。
3. **JobSystem**：AI 批量 `ParallelFor`（只写各车 input）；`Configure(0)` 供 drive-test / WASM 确定性。
4. **仿真线程**：比赛阶段 `SimThread` 推进物理，主线程消费三缓冲快照做输入与渲染；菜单仍单线程。跨线程玩法输入仍经 `PlayerIntent` 邮箱（主线程 Drain 后写入）。
5. **车辆子步仍串行**：`IPhysicsWorld` 非线程安全；并行 vehicle 只允许 L2 编排，禁止下沉 L1。

性能与度量见 [perf-async-engine.md](perf-async-engine.md)。

---

## 5. 物理后端（可插拔）

| 后端 | CMake / 类型 | 角色 |
|------|----------------|------|
| **qu3e**（默认） | `Qu3eWorld` | 轻量 OBB 刚体；bench / 实机默认 |
| **nudge** | `NudgeWorld` | 功能超集试用；实测远慢于 qu3e，非默认 |
| **RPC** | `RushPhysicsCore` → `rush-physics-core`（`rpc*`） | qu3e first-party fork，深度碰撞定制 |

配置切换：`[physics] engine = qu3e | nudge | rpc`。

**硬边界**：

- L1 **只拥有碰撞**（broadphase、窄相、岛、顺序冲量、raycast）。
- **车体永不下沉**进任何 L1 库（见 `rush-physics-core/vehicle/README.md`）。
- `rush::Result` / `EX_FAIL` 只活在 L2 适配层；L1 保持 `bool`/`assert`/空返回。
- 符号前缀 `rpc*` 保持，便于与 stock qu3e 并链。

RPC 重构进度与基准见 [rpc-refactor.md](rpc-refactor.md)。

---

## 6. 数据与配置

| 数据 | 格式 | 路径 |
|------|------|------|
| 地图 | KVRec v2 → `*.rushmap` | `assets/maps/*.rushmap` |
| 地图作者源 | CSV 目录 | `assets/maps/<name>/{track,terrain,decor,zones}.csv` |
| 用户配置 | KVRec v2 → `config.rushconf` | EXE 同级（gitignore） |
| CSV→地图 | `mapconv <csv_dir> <out.rushmap>` | 仅 `debug-tools` 预设 |

迁移已完成；过程记录在 [history/kvrec-migration.md](history/kvrec-migration.md)。

---

## 7. 网络（L3）

- **拓扑**：主机 = 服务器 + 玩家；ENet 主机权威；客户端只发输入。
- **通道 0（不可靠）**：高频状态与输入；**通道 1（可靠）**：加入/离开、圈速、损伤等低频事件。
- **发现**：UDP 广播房间码 + ENet 端口；客户端过滤后连接。
- **平滑**：远端车 100–150 ms 缓冲插值；慢变状态随快照或可靠事件同步。

详细协议仍以 [history/design-v1.md](history/design-v1.md) §7 为需求参考；实现以 `src/` + ENet 代码为准。

---

## 8. 构建预设

| 预设 | 目录 | 用途 |
|------|------|------|
| `release` | `build/` | 游戏，`APEX_BUILD_TOOLS=OFF`，O3+ThinLTO |
| `debug` | `build/` | 调试游戏 |
| `debug-tools` | `build-tools/` | 游戏 + mapconv |
| `web` | `build-web/` | Emscripten WASM（无 pthread → Job/线程强制串行） |

ctest：`drive-test` 调用 `apex-rush --drive-test`（超时 900 s）。

---

## 9. 相关决策摘录

| 决策 | 结论 |
|------|------|
| 车体是否进物理引擎 | **否**（2026-11-22 裁定）；L1 只碰撞 |
| 地图/配置格式 | KVRec，扩展名 `.rushmap` / `.rushconf` |
| 默认物理后端 | qu3e；RPC/nudge 可切换 |
| 渲染 | Raylib 低模路线；菜单与比赛渲染路径经引擎 render 模块 |
| 输入 | **独立 input 线程**；模块自注册事件名+手势+多键；**纯事件无回调**（设计见 [input.md](input.md)） |
| 分发 | 单 EXE + 可选同级配置；局域网自包含 |
