# Rush Physics Core（RPC）重构方案

状态：**已实施（P0 + P1 的 T6/T7）**；P2（D8/D9 并行与求解器缓存）保持设计稿，待单独评审  
目标读者：执行重构的实现者  
规范依据：`~/.config/mimocode/AGENTS.md`（通用 AI 编程架构与工程规范）  
关联：[architecture.md](./architecture.md) · [perf-async-engine.md](./perf-async-engine.md) · `rush-engine/third_party/rush-physics-core/` · `rush-engine/src/physics/rpc_impl.cpp`

---

## 0. 优先级与冲突裁决

按 AGENTS.md §零：

| 优先级 | 来源 | 本方案中的体现 |
|--------|------|----------------|
| 1 | 用户即时指示 | 实施前由用户圈定 P0/P1/P2 范围；本档可被当场改写 |
| 2 | 项目特定要求 | L1 只做碰撞、不进车体；L1 不用 `rush::Result`；`IPhysicsWorld` 是唯一对外缝 |
| 3 | 通用规范 | 分层 / SRP / 接口先行 / 结果对象 / 可测试 / 童子军军规 |

**硬约束（项目级，不可被通用规范推翻）：**

1. **L1 `rush-physics-core` 只拥有碰撞**：broadphase、box 窄相、contact island、顺序冲量、raycast 原语。  
2. **车体永不下沉**进 RPC（见 `vehicle/README.md`）。  
3. **`rush::Result` / `EX_FAIL` 只活在 L2** `IPhysicsWorld` 适配层；L1 保持 `bool`/`assert`/空返回（与 qu3e/nudge 一致）。  
4. **不引入异常**控制业务流；L1 失败路径用可观察的“无效果 + assert（Debug）”。  
5. **符号前缀 `rpc*`** 保持，以便与 stock qu3e 并链。

---

## 1. 背景与目标

### 1.1 现状一句话

RPC 是 qu3e 的 first-party fork（`q3*` → `rpc*`），作为 L1 碰撞后端之一，经 `RushPhysicsCore`（`rpc_impl.cpp`）实现 L2 `IPhysicsWorld`，与 `Qu3eWorld` / `NudgeWorld` 可插拔替换。

### 1.2 为什么要重构

| 驱动 | 证据 |
|------|------|
| **性能** | 百车 bench 后瓶颈回落 `world.Step`（见 perf 文档）；RPC 内核仍有可量化热点 |
| **架构债** | 接触管理 / 求解 / 查询回调职责纠缠；`TestCollisions` 过长；适配层 O(n) 反查 |
| **可测试性** | L1 几乎无独立单测缝；Listener/Query 与 Scene 隐式耦合 |
| **可维护性** | fork 后未做结构化演进；`@TODO`、重复 include、调试路径混入热路径 |

### 1.3 目标（可验收）

1. **架构**：L1 内部分层清晰，依赖单向；对外仍仅暴露 `rpc.h` 伞头 + Scene/Body/Query 最小面。  
2. **性能**：在 `--bench` 同负载下，`step`/`contact` 段相对基线有可重复改善（目标见 §7）；无行为回归（drive-test 0 failures）。  
3. **接口**：热路径查询/接触回调仍为虚接口（可 Mock）；新增内部策略接口时接口先行。  
4. **错误**：L2 继续 `rush::Result`；L1 不引入异常、不引入 Result。  
5. **整洁**：消灭热路径上的整份 `rpcManifold` 拷贝、无条件 `Validate()`、适配层线性 `handleOf` 等坏味道。

### 1.4 非目标

- 车辆动力学、多形状凸包、CCD、连续碰撞。  
- 把 `IPhysicsWorld` 或 `rush::Result` 下沉进 L1。  
- 重写为全新引擎（Box2D/Jolt 级重写不在本方案）。  
- 默认开启全局 sleep 行为变更（`allowSleep` 策略属玩法/适配层配置，见 §5.4）。

---

## 2. 分层与依赖（必须遵守）

### 2.1 系统分层

```
L3  apex-rush（游戏）
      │  仅 import rush.engine.* / 链接 rush::engine
      ▼
L2  rush-engine
      │  IPhysicsWorld + RushPhysicsCore 适配
      │  Result / CollisionBus / Vehicle（车体只在此层）
      ▼
L1  rush-physics-core（rpc*）     ← 本方案重构对象
      │  纯碰撞：scene / dynamics / broadphase / collision / math / common
      ▼
     （无更下层物理依赖；仅标准库 + 自有 memory）
```

### 2.2 L1 内部依赖方向（目标态）

```
scene (rpcScene)
   │ owns / orchestrates
   ▼
dynamics (ContactManager, Island, ContactSolver, Body, Contact)
   │ uses
   ▼
broadphase (BroadPhase → DynamicAABBTree)     collision (Box, Collide/SAT)
   │                    │
   └────────┬───────────┘
            ▼
common (Memory, Geometry, Settings, Types)  +  math (Vec3/Mat3/Quat/Transform)
```

**禁止：**

| 禁止项 | 说明 |
|--------|------|
| 下层 → 上层 | math/common 不得 include scene/dynamics |
| 同层横跳无理由 | broadphase 不得直接调 ContactSolver；collision 不得依赖 Scene |
| L1 → L2 | `third_party/rush-physics-core` 不得 include `rush/engine`、exstl Result、模块接口 |
| L3 → L1 | 游戏不得 `#include "rpc.h"` |
| 业务硬编码第三方 | L2 以上只依赖 `IPhysicsWorld`，不 `new rpcScene`（除适配 TU） |

### 2.3 目录归属（重构后仍保持）

```
rush-engine/third_party/rush-physics-core/
├── rpc.h                 # 伞头：唯一推荐外部入口
├── scene/                # 编排 + 查询 + Listener 面
├── dynamics/             # Body / Contact* / Island / Solver
├── broadphase/           # BroadPhase + DynamicAABBTree
├── collision/            # Box 窄相 SAT + 裁剪
├── math/  common/  debug/
└── vehicle/              # 已退役占位（保持 README，不复活）
```

---

## 3. 现状架构与坏味道审查

### 3.1 模块职责（现状 → 目标）

| 模块 | 现状职责 | 目标职责（SRP） | 问题 |
|------|----------|-----------------|------|
| `rpcScene` | 步进编排、Body 生命周期、查询、配置 | **仅**编排 Step 管线 + Body 仓库 + 转发查询 | Step 内联分岛+求解+同步，过长 |
| `rpcContactManager` | 配对、约束表、窄相、热启动、Listener | 约束表生命周期 + **调用** Narrowphase/事件 | `TestCollisions` ~80 行混杂四件事 |
| `rpcIsland` | DFS 分岛 + 积分 + 调 Solver + 睡眠 | 分岛结果容器 + 步进驱动 | 与 Scene 分岛循环职责重叠 |
| `rpcContactSolver` | 顺序冲量 | 保持；接口已较清晰 | 每迭代全量扫；可并行化预留 |
| `rpcBroadPhase` | 移动缓冲、配对、去重 | 保持 | **无条件 `Validate()`** 在热路径 |
| `rpcBoxtoBox` | SAT+Clip | 保持纯函数式窄相 | 可测性好，勿搅入 Manager |
| `rpcBody` | 刚体状态 + boxes + 力 | 保持 | `SetTransform` 语义与 `m_worldCenter` 耦合需文档化 |
| `rpcMemory` | Stack/Heap/Paged | 保持 | Heap first-fit 仅冷路径，优先级低 |
| `rpcContactListener` / `rpcQueryCallback` | 虚回调 | **保持虚接口**（可 Mock） | 适配层实现质量见 P1 |

### 3.2 技术债清单（审查结论）

| ID | 位置 | 坏味道 | 违反的规范 | 优先级 |
|----|------|--------|------------|--------|
| D1 | `rpcContactManager::TestCollisions` | 过长函数；整份 `rpcManifold` 拷贝；窄相+热启动+事件揉在一起 | SRP、函数短小 | **P0** |
| D2 | `rpcBroadPhase::UpdatePairs` | Release 仍调用 `m_tree.Validate()`（扫 free list） | 童子军、性能 | **P0** |
| D3 | `rpcBox::ComputeAABB` | 8 顶点变换，未用 OBB 中心+|R|e | 性能、拒绝不必要底层浪费 | **P0** |
| D4 | `rpc_impl.cpp` `handleOf` | 每个接触事件 O(n) 扫 body 表 | 性能、最小依赖 | **P0**（L2） |
| D5 | `rpc_impl.cpp` 映射 | `std::map` 三份（bodies/boxes/boxToBody）热路径查找 | 性能 | P1 |
| D6 | `rpcScene::Step` | 分岛 DFS 内联；栈 Reserve 每帧按 worst-case | 可读、可测 | P1 |
| D7 | `rpcContactManager.h` | `static void SolveCollision(void*)` 声明无定义/无用 | 整洁 | P1 |
| D8 | `rpcContactSolver::Solve` | 迭代内反复读写 `m_velocities`；无块状缓存 | 性能（进阶） | P2 |
| D9 | 岛屿级并行 | 整帧单线程 `Step` | 性能（进阶，需确定性策略） | P2 |
| D10 | L1 错误通道 | 静默失败仅 assert；与 L2 Result 契约靠文档 | 显式错误（边界说明） | 文档化即可 |
| D11 | `SetAllowSleep(false)` | 适配层全局关睡眠（与 qu3e 相同） | 配置策略非结构债 | 见 §5.4，非默认重构项 |
| D12 | 重复 `#include "rpcIsland.h"` 等 | Scene.cpp 重复 include | 整洁 | 随手修 |

---

## 4. 目标架构与接口先行

### 4.1 设计原则映射（AGENTS.md → RPC）

| 规范条 | 在本重构中的落法 |
|--------|------------------|
| 严格分层 | §2；重构 PR 不得引入跨层 include |
| 模块化 SRP | `TestCollisions` 拆为：持久化校验 → 窄相 → 热启动匹配 → 事件边沿 |
| DIP / 可插拔 | 对外已是 `IPhysicsWorld`；对内新增 `INarrowphase` / `IIslandSolver` 仅当 P2 需要替换算法时 |
| 通用化 | 不为 SAT 手写散落 if-else 大山——保持单一 `rpcBoxtoBox` 策略函数；禁止在 Manager 里复制第三套窄相 |
| 结果对象 | L2 `rush::Result` 不变；L1 不用异常 |
| 可测试性 | Listener/Query 虚接口保留；Narrowphase 纯函数可单测；Scene 可注入 Fake Listener |
| 童子军 | 每 PR 顺手清 D12、死声明 D7 |
| 先设计后编码 | 本文件即设计；实施前按 §7 里程碑拆任务 |
| 接口先行 | §4.3 先定内部接口再动实现 |
| 双实现证明 | P2 策略接口需 **SAT 实现 + 测试用 Fake/参考实现** 或 **串行 + 并行岛求解两实现** |

### 4.2 目标模块图

```
                    ┌─────────────────────────────────────┐
                    │  rpcScene  (Facade / 编排)           │
                    │  Step · Create/RemoveBody · Query*   │
                    └───────────┬─────────────┬───────────┘
                                │             │
              ┌─────────────────┼─────────────┼─────────────────┐
              ▼                 ▼             ▼                 ▼
     ┌────────────────┐ ┌──────────────┐ ┌────────────┐ ┌──────────────┐
     │ BodyRegistry   │ │ ContactWorld │ │ QueryService│ │ IslandBuilder│
     │ (链表可保留)   │ │ 约束表+事件  │ │ AABB/Point │ │ DFS → islands│
     └────────────────┘ └──────┬───────┘ │ /Ray       │ └──────┬───────┘
                               │         └────────────┘        │
                    ┌──────────┴──────────┐                    ▼
                    ▼                     ▼            ┌────────────────┐
           ┌────────────────┐    ┌─────────────────┐    │ IslandStepper  │
           │ BroadPhase     │    │ Narrowphase     │    │ 积分+调 Solver │
           │ + DynamicTree  │    │ (纯: BoxtoBox)  │    └────────────────┘
           └────────────────┘    └─────────────────┘
                    │                     │
                    └──────────┬──────────┘
                               ▼
                    common / math / memory
```

> **落地形态说明（避免过度设计）：**  
> P0/P1 **不要求**真的拆出十个新类文件；要求的是 **函数级/结构体级边界** 与依赖方向。  
> 仅当 P2 做算法可插拔时，才引入命名接口（§4.3.3）。

### 4.3 接口定义（接口先行）

#### 4.3.1 已有对外接口（保持，L2）

```cpp
// rush.engine.physics — 不在本重构中修改签名
class IPhysicsWorld { /* CreateBody / Step / Raycast / DrainContacts / Friction... */ };
class RushPhysicsCore final : public IPhysicsWorld;
```

可插拔证明（已存在，满足 AGENTS §三.4）：`Qu3eWorld` · `NudgeWorld` · `RushPhysicsCore`。

#### 4.3.2 L1 已有虚接口（保持）

```cpp
class rpcContactListener {
  virtual void BeginContact(const rpcContactConstraint*) = 0;
  virtual void EndContact(const rpcContactConstraint*) = 0;
};

class rpcQueryCallback {
  virtual bool ReportShape(rpcBox*) = 0;  // false = 停止查询
};
```

**测试缝：** 单测注入 recording fake listener，断言 Begin/End 边沿；Query 返回 false 测早停。

#### 4.3.3 P2 内部策略接口（仅并行/算法替换时启用）

```cpp
// 接口先行；实现类在算法就绪后再写
struct INarrowphase {
  virtual ~INarrowphase() = default;
  // 纯函数语义：只写 manifold，不碰约束表/Listener
  virtual void Collide(rpcManifold* m, rpcBox* a, rpcBox* b) = 0;
};

struct IIslandSolver {
  virtual ~IIslandSolver() = default;
  virtual void Solve(rpcIsland& island) = 0;
};

// 实现 1：现有 SAT+Clip / 顺序冲量（默认）
// 实现 2：Fake（单测）或 ParallelIslandSolver（性能，Job/线程由 L1 自持 std 线程或由 L2 注入）
```

**依赖注入：** `rpcScene` 构造或 setter 注入默认实现；禁止 Scene 内 `new` 具体算法散落各处。

#### 4.3.4 P0 拆分后的内部函数边界（非虚，先结构后类型）

```cpp
// rpcContactManager 内（或同 TU 匿名命名空间）
bool ContactStillValid(rpcContactConstraint* c);      // CanCollide + broadphase overlap
void NarrowphaseUpdate(rpcContactConstraint* c);      // SolveCollision + ComputeBasis
void WarmStartMatch(rpcContactConstraint* c);         // 特征对匹配 + 摩擦重投影
void FireContactEdges(rpcContactConstraint* c);       // Begin/End 边沿 → Listener
```

`TestCollisions` 仅编排上述四步——**先拆函数，不先上虚接口**。

---

## 5. 分阶段重构计划

### 5.1 阶段总览

| 阶段 | 范围 | 风险 | 依赖 | 建议 PR 粒度 |
|------|------|------|------|----------------|
| **P0 热路径清理** | D1–D4、D12 | 低（行为应不变） | 无 | 2–3 个 PR |
| **P1 结构与适配** | D5–D7、Scene Step 可读性、查询路径 | 中 | P0 | 每项一 PR |
| **P2 算法/并行** | D8–D9、§4.3.3 接口 | 高（确定性） | P0+P1 基准稳定 | 单独设计评审后做 |

**用户圈定范围时建议：** 只要性能数字 → **仅 P0**；要结构可维护 → **P0+P1**；要冲多核 → **P0+P1+P2**。

---

### 5.2 P0 — 热路径与明显坏味道（推荐默认范围）

#### P0-1 拆 `TestCollisions` + 消灭整份 manifold 拷贝（D1）

**现状问题：**

```cpp
rpcManifold oldManifold = constraint->manifold;  // 含 8×contact 大结构
constraint->SolveCollision();
// 再 O(n²) 匹配 warm start
```

**目标做法：**

1. 只快照热启动所需字段：

```cpp
struct WarmStartSnapshot {
  rpcVec3 tangent[2];
  i32 count;
  struct {
    i32 fpKey;
    r32 normalImpulse;
    r32 tangentImpulse[2];
  } pts[8];
};
```

2. 按 §4.3.4 拆四步；Listener 边沿逻辑保持与现网一致（`eColliding` / `eWasColliding`）。  
3. **不变式：** 同一初始状态下，重构前后 manifold 法向/点数/热启动冲量一致（用单测或 dump 对比）。

**验收：** drive-test 0 failures；contact 段耗时不劣化（bench）。

#### P0-2 `Validate()` 移出 Release 热路径（D2）

```cpp
// rpcBroadPhase::UpdatePairs 末尾
#ifndef NDEBUG
  m_tree.Validate();
#endif
```

或 `Validate()` 整函数体 `#ifndef NDEBUG`。  
**验收：** Release bench `step` 不含 free-list 扫描；Debug 仍可断言树完整。

#### P0-3 OBB AABB 快速公式（D3）

`rpcBox::ComputeAABB`：由 8 顶点 `rpcMul` 改为

```
center = world.position
extent = |R| * e   // 列绝对值点乘半轴
min/max = center ∓ extent
```

**验收：** 与旧公式在随机姿态下 max 误差 &lt; 1e-5（单测）；broadphase `SynchronizeProxies` 变快。

#### P0-4 适配层 O(1) body 反查（D4）

`RushPhysicsCore::Impl`：

- 增加 `std::unordered_map<const rpcBody*, BodyHandle> bodyToHandle`（或复用 `boxToBody`：`contact->A/B` → handle，**二选一，优先 box 已有映射**）。  
- `BeginContact` 禁止再线性扫 `bodies`。  
- `CreateBody` / `DestroyBody` 同步维护反查。

**验收：** 100 车 bench contact 段下降；AddressSanitizer 下 create/destroy 无悬垂。

#### P0-5 顺手整洁（D12、D7）

- 去掉重复 include、未使用 `static SolveCollision` 声明。  
- 不改行为。

**P0 明确不做：** 改 sleep 策略、改迭代次数、改 SAT 算法、并行。

---

### 5.3 P1 — 结构与适配层质量

| ID | 项 | 做法 | 验收 |
|----|----|------|------|
| D5 | `std::map` → 更合适容器 | handle 为稠密 `uint32`：`vector` 槽位 + freelist，或 `unordered_map`；**接口不变** | create/destroy/raycast 路径 bench |
| D6 | Scene Step 可读 | `BuildIslands` / `SolveIslands` / `SyncAndFind` 三个私有方法；栈分配策略注释 worst-case | 行为不变；函数 &lt; ~80 行 |
| — | 查询路径 | `QueryAABB/Point/RayCast` 保留 wrapper；确保 `ClosestHitCallback` 忽略规则有单测 | 射线 ignore、toi≤0 跳过 |
| — | 错误契约文档 | 在 `rpc.h` 顶部注释：L1 无效输入 = Debug assert + 无效果；L2 负责 Result | 与 `physics.cppm` 注释一致 |
| — | Listener 效率 | 文档要求 Begin 内禁止分配/创建 Body；适配层只 `push` POD 事件 | 与现 CollisionBus 模式一致 |

**替换示例（可插拔证明，AGENTS §三.4）：**  
适配层测试使用 `FakeContactListener` 记录事件，不依赖真实 Scene 步进细节。

---

### 5.4 P2 — 性能深水区（需单独决策）

| ID | 项 | 要点 | 风险 |
|----|----|------|------|
| D8 | 求解器缓存 | 接触约束 SoA、迭代内寄存器缓存 `vA/wA` | 中；需数值对比 |
| D9 | 岛屿并行 | 岛之间无共享写 → 可并行 `Island.Solve` | **确定性**：drive-test 串行 JobSystem=0 必须仍确定；并行仅 bench/运行时可选开关 |
| — | 窄相并行 | `TestCollisions` 按 constraint 列表并行写独立 manifold | Listener 线程安全：要么先并行窄相再单线程发事件，要么适配层事件 SPSC 队列 |
| D11 | Sleep 策略 | `BodyDesc.allowSleep` 已按车传入；场景级 `SetAllowSleep(false)` 是否过猛 | **玩法决策**，非纯工程；默认保持现网，避免静手感变化 |

**P2 开关建议：** `rpcScene` 增加 `SetParallelIslands(bool)`，默认 false；L2 仅在非 drive-test 路径打开。

**接口先行：** 先合入 §4.3.3 `IIslandSolver` + 串行实现 = 现行为，再加并行实现。

---

## 6. 错误处理策略（AGENTS §二.5）

| 层 | 策略 |
|----|------|
| L3/L2 对玩法 | `rush::Result`（已有）；热路径 `value_or` / 忽略 `Result<void>` |
| L2 适配 RPC | 非法 handle / 非法 BodyDesc → `EX_FAIL`（保持） |
| L1 内部 | **不用异常、不用 Result**；契约：非法指针/handle = Debug `assert` + 操作无效返回；Release 尽量不 UB（调用方契约） |
| 查询未命中 | 不是错误：`RayHit.hit==false` / `ReportShape` 不调用 |
| Listener | 无返回值；错误不得靠 Listener 抛出 |

**禁止：** 用异常做“接触失败 / 查询 miss”控制流。

---

## 7. 可测试性与基准（AGENTS §二.6）

### 7.1 测试分层

| 层级 | 对象 | 手段 |
|------|------|------|
| L1 单测（新增，建议） | `ComputeAABB`、`BoxtoBox`、`WarmStartMatch`、AABB tree insert/update | 纯函数/小场景；不链游戏 |
| L1 集成 | `rpcScene` + FakeListener | 固定 dt 步进，断言 Begin/End 次数 |
| L2 驱动 | `--drive-test`（三后端） | **合并门禁：0 failures** |
| L2 性能 | `--bench [cars]` | 记录 `step`/`contact`/墙钟；RPC 与 qu3e/nudge 对比 |

### 7.2 基准协议

```text
cmake --preset release && cmake --build --preset release
build\apex-rush.exe --bench 100
build\apex-rush.exe --drive-test
```

| 指标 | 基线来源 | P0 目标（经验向，以实测为准） |
|------|----------|-------------------------------|
| `per-step step` | 重构前同机同 commit | **≥ 5–15%** 改善（P0 组合） |
| `per-step contact` | 同上 | 适配层反查后 **明显** 下降 |
| 墙钟 600 帧 | 同上 | 不劣化；P0 后记录进 perf 文档附录 |
| drive-test | 现网 | **0 failures** 三后端 |

**确定性：** P0/P1 不得改变同 seed 车体轨迹语义（允许浮点级容差的单测，不允许玩法级漂移）。P2 并行必须提供串行回退。

### 7.3 Mock/Fake 清单

- `rpcContactListener` 录音 Fake  
- `rpcQueryCallback` 计数 / 早停 Fake  
- （P2）`IIslandSolver` 记录调用的 Fake  

---

## 8. 实施任务拆分（先设计后编码 → 可执行 backlog）

| Task | 阶段 | 内容 | 涉及文件 | 完成定义 |
|------|------|------|----------|----------|
| T1 | P0 | Warm-start 快照 + 拆 `TestCollisions` | `rpcContactManager.cpp` | 函数拆分 + 无整份 manifold 拷贝 |
| T2 | P0 | `Validate` 仅 Debug | `rpcBroadPhase.cpp` | Release 不扫 free list |
| T3 | P0 | `ComputeAABB` 公式化 | `rpcBox.cpp` | 单测/误差检查通过 |
| T4 | P0 | 适配层 O(1) 反查 | `rpc_impl.cpp` | contact 路径无 O(n) |
| T5 | P0 | include/死代码整洁 | Scene/ContactManager | 童子军 |
| T6 | P1 | handle 容器与 Step 函数拆分 | `rpc_impl.cpp` / `rpcScene.cpp` | 可读 + bench |
| T7 | P1 | L1 错误契约注释 | `rpc.h` | 文档与 L2 一致 |
| T8 | P2 | `IIslandSolver` 接口 + 串行实现 | dynamics | 接口先行合入 |
| T9 | P2 | 可选并行岛 | dynamics | 开关默认关；drive-test 仍串行确定 |

每个 Task：**先 diff 设计（本文件对应节）→ 再编码 → bench + drive-test → 童子军清理。**

---

## 9. 风险与回滚

| 风险 | 缓解 |
|------|------|
| 热启动匹配改错 → 堆叠抖动 | T1 前后 dump manifold 冲量对比；drive-test |
| AABB 公式与 local 变换不一致 | 随机姿态对照旧 8 顶点路径单测 |
| 反查 map 悬垂 | DestroyBody 先摘映射再 `RemoveBody`；ASan |
| 过度抽象（过早 IIslandSolver） | **P0/P1 禁止**；P2 才引入 |
| 改 sleep/迭代“顺手优化”导致手感变化 | 非目标；禁止夹带 |
| 与 qu3e 分叉扩大难维护 | 仅 RPC 路径演进；不回改 stock qu3e；行为对齐靠 L2 测试 |

**回滚：** 按 Task 粒度 PR；P0 各 Task 可独立 revert。

---

## 10. 与 AGENTS.md 对照表（审查用）

| AGENTS 要求 | 本方案章节 | 状态 |
|-------------|------------|------|
| 优先级链 | §0 | 已定义项目硬约束 |
| 严格分层 / 单向依赖 | §2 | L1 内 + L1/L2 边界 |
| SRP / 最小依赖 | §3.1、§4.3.4 | 拆 TestCollisions |
| DIP / 可插拔 | §4.3、IPhysicsWorld 已有 | P2 策略接口 |
| 拒绝散落底层硬编码 | §4.1 | 窄相保持单一策略函数 |
| 结果对象、禁异常控制流 | §6 | L2 Result / L1 assert |
| 依赖注入 + Mock | §4.3.2、§7.3 | Listener/Query/未来 Solver |
| 童子军 / 拒绝屎山 | §3.2 D7/D12、§8 T5 | 随手清理 |
| 先设计后编码 | 本文件 | 实施前确认范围 |
| 接口先行 | §4.3 | P2 强制；P0 函数边界 |
| 双实现证明可插拔 | §4.3.1、§4.3.3 | 三后端已有；P2 Fake+串行/并行 |

---

## 11. 建议决策（给圈范围用）

**推荐默认：P0 全做 + P1 的 T6/T7。**

理由：

1. 完全落在 AGENTS「性能 + 整洁 + 不过度设计」交集；  
2. 不改对外 API、不改玩法配置；  
3. 直接打击 bench 已知瓶颈方向（`Step`/`Contact`）与适配层热点；  
4. 为 P2 留出干净的函数边界，避免先上并行再拆屎山。

**可选：**

- 只要数字：**仅 P0（T1–T5）**  
- 要多年可维护：**P0 + P1 全部**  
- 要多核：**P0+P1 后再开 P2 设计评审**

---

## 12. 实施结果与性能对比

### 12.1 任务完成情况（2026-11-22）

| Task | 内容 | 落点 | 状态 |
|------|------|------|------|
| T1 | Warm-start 快照 + 拆 `TestCollisions` | `rpcContactManager.{h,cpp}`：`WarmStartSnapshot` + `ContactStillValid`/`NarrowphaseUpdate`/`WarmStartMatch`/`FireContactEdges` 四步 | ✅ |
| T2 | `Validate` 仅 Debug | `rpcBroadPhase.cpp`：`#ifndef NDEBUG` | ✅ |
| T3 | `ComputeAABB` 公式化 | `rpcBox.cpp`：center ± \|R\|·e；单测 `tests/test_compute_aabb.cpp`（10000 随机姿态，max error = 0） | ✅ |
| T4 | 适配层 O(1) 反查 | `rpc_impl.cpp`：`BeginContact` 走 `boxToBody`，删除 O(n) `handleOf` | ✅ |
| T5 | include/死代码整洁 | 删除 `static SolveCollision` 死声明、`rpcScene.cpp` 重复 include | ✅ |
| T6 | handle 容器 + Step 拆分 | `rpc_impl.cpp` 三份 `std::map` → `unordered_map`；`rpcScene::Step` 拆 `SolveIslands`/`BuildIsland`/`SyncAndFind` | ✅ |
| T7 | L1 错误契约注释 | `rpc.h` 顶部注释块（L1 assert/无效果、L2 Result、Listener 效率约定） | ✅ |
| T8/T9 | P2 策略接口 + 并行岛 | **未实施**（按 §5.4 需单独设计评审） | ⏸ |

附加（实施中发现的测试基建缺陷）：`Config::SetTestMode(bool)` — 测试/自动化模式下 `Config::Load/Save` 变为 no-op，
`--drive-test`/`--bench`/截图/dump 路径永不读写用户 `config.rushconf`（此前 settings/race-setup 的
`Config::Save` 会把 `race.autostart=true` 等测试 override 泄漏进用户配置，导致后续 menu 测试连环失败）。
正常游玩不受影响。验收：drive-test 前后 `config.rushconf` SHA256 一致。

### 12.2 基准协议

```text
cmake --preset release && cmake --build --preset release
build\apex-rush.exe --bench 100     # 各后端 100 车 × 600 帧 × 2400 physSteps
build\apex-rush.exe --drive-test    # 门禁：0 failures
```

同机（Windows，clang -O3 -flto=thin），每组跑两次取稳定值。门禁：`--drive-test` 三后端
0 failures ✅；`rpc-compute-aabb` 单测 PASS ✅。

### 12.3 性能对比（rpc，100 车）

| 指标（per-step / wall） | 基线 | 重构后 | 变化 |
|-------------------------|------|--------|------|
| `step` | 0.7466 ms | 0.6301–0.6319 ms | **−15.5% ~ −15.6%** ✅（目标 5–15%，达成上限） |
| `contact` | 0.0016 ms | 0.0016–0.0017 ms | 持平（量级已极小） |
| `vehicle`（含 raycast 反查受益） | 0.8831 ms | 0.6575–0.6725 ms | **−24% ~ −26%**（T4/T6 反查 O(n)→O(1) 的直接收益） |
| 墙钟 600 帧 | 5.640 s | 4.835–4.872 s | **−13.6% ~ −14.3%** ✅ |
| 吞吐 | 425.6 steps/s | 496.4 steps/s | **+16.6%** |

对照（同轮次，未改动的 qu3e 后端，用于排除机器噪声）：

| 指标 | qu3e 基线 | qu3e 同轮 | 变化 |
|------|-----------|-----------|------|
| `step` | 0.7512 ms | 0.7474–0.7527 ms | ±0.5%（噪声） |

即：rpc 的 step 段从与 qu3e 持平（0.747 vs 0.751）变为明显更快（0.630 vs 0.750）；
100 车下 RPC 现已比 stock qu3e 快约 16%（step 段）。qu3e 同轮无系统性漂移，改善可归因于本次重构。

拆解归因：
- `step` −15.5% ← T1（整份 manifold 拷贝 → 快照）+ T2（Release 免 Validate 扫 free list）+ T3（OBB AABB 公式）+ T6（unordered_map 查询）；
- `vehicle` −24% ← T4/T6（raycast/接触事件反查 O(n) → O(1)）；
- `contact` 持平（0.0016 ms 量级，T1 的拷贝节省被噪声淹没）。

### 12.4 回滚

按 Task 粒度可独立 revert；P0/P1 未改对外 API（`IPhysicsWorld` 签名、`rpc*` 符号前缀、
sleep/迭代配置均未动），P2（T8/T9）未开工。

---

## 13. 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| v0.1 | （初稿） | 按 AGENTS.md 输出：现状审查、目标分层、接口先行、P0–P2、测试与基准、任务拆分 |
| v0.2 | 2026-11-22 | 实施完成 P0（T1–T5）+ P1（T6/T7）；新增 `Config::SetTestMode` 修复测试污染配置；记录 §12 性能对比（step −15.5%，墙钟 −14%）；P2 维持设计稿 |
