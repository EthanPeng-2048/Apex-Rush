# RPC 物理内核深度优化（算法 · 硬件 · 现代 C++）

状态：**现行优化方案**（§1.4 / §1.5 为本轮已落地项与实测；基线数据 2026-11-25 本机实测）
关联：[rpc-refactor.md](rpc-refactor.md)（分层与 P0/P1/P2 重构）· [physics-upgrade.md](physics-upgrade.md)（车辆物理与 §10 性能）· [perf-async-engine.md](perf-async-engine.md)（异步引擎基准）· [architecture.md](architecture.md)（分层铁律）

本文只讨论 **L1 `rush-physics-core`（qu3e fork）与 L2 车辆子步的耦合**。测量环境：Intel Xeon E5-2697A v4（Broadwell，AVX2/FMA，无 AVX-512，16C32T，**L2 256 KB/core**、L3 40 MB）、Windows、clang 23.1.5（MSVC ABI）、Release `-O3 -flto=thin`、**无 `-march`**（基线 SSE2）。

---

## 0. 结论速览

**当前成本模型一句话**：帧成本 = 子步数 × (车辆层 + 世界步)。改造前车辆层 ≈ 每车 4 条射线 × 2.32 µs、世界步 ≈ 每车 11 个小盒各自的宽相操作 + 接触数 × 求解迭代；480 Hz 下单帧 8 个子步，120 车实测 20.6 ms 物理，**超出一帧预算**——这不是某个 bug，是"每子步固定成本 × 8"的结构问题。射线加速器（§1.5）落地后每帧 11.5 → 第三轮修正后 **12.76 ms**（射线重新"看得见"车辆，见 §1.7 的自查）；剩余大头是代理维护 + 配对生成（见 §1.5 / §1.7 子步拆分）。

**优化主线三条**（互相正交，可分别验收；第二条的射线部分已落地，见 §1.5）：

| 主线 | 手段 | 目标 | 依据 |
|---|---|---|---|
| ① 把"每子步固定成本"降下来 | 静态几何网格 + 动态专用 BVH（**已落地，§1.5**）+ 节点压缩 + 每车单代理 | 每子步 vehicle+world 2.60 → 1.75 ms（已达成 1.75；节点压缩/单代理待做） | §3.1 §3.2 §4.1 §4.3 |
| ② 把"子步数 × 固定成本"解耦 | TGS 子步（宽相/窄相一轮、求解多轮）+ 接触锚点局部坐标缓存 | 480 Hz 的宽相+窄相成本 ÷4~8 | §3.5 |
| ③ 把"接触数 × 迭代"的常数降下来 | 持久 manifold + soft constraint/relaxation 取代 Baumgarte + 岛内 SoA/拓扑序 | 每接触成本 ÷2~3，且不再需要弹射钳制 | §3.3 §3.4 §4.2 |

**第三轮结束时的现状**（120 车 / 480 Hz，8 子步）：每帧物理 **12.76 ms** = 车辆层 6.79（其中射线 1.6）+ 世界步 5.37 + AI 0.52；墙钟 7.66 s。相对改造前 20.6 ms **-38%**。

> **必须先读的度量教训**：上一轮报的"射线 0.53 µs/条、-77%"是在**动态盒从未进入动态射线索引**的前提下测出来的（`RegisterBoxProxy` 只转发静态盒），那段时间所有射线对车辆**失明**。修掉之后同场景是 **1.19 µs/条**。凡"某阶段突然变便宜"的结论，先问一句"它是不是少做了事"（§1.7.1）。

**已落地**（本轮，已在本仓库代码里，`--drive-test` 三后端 0 failures）：岛池按实际用量分配（3.2 GB → 3.1 MB）、动态体对接触预算（堆积峰值 68 680 → 9 146 接触）、Baumgarte 修正量钳制 0.2 m、线/角速度硬上限、AABB margin 0.5 → 0.2 m、求解迭代随子步率收缩（480 Hz 用 8 次而非 16 次）。合计效果见 §1.4。

---

## 1. 基线与度量

### 1.1 复现命令

```powershell
build\apex-rush.exe --bench 120 480 rpc     # 120 车、480Hz、只跑 rpc
build\apex-rush.exe --bench 120 240 rpc     # 换频率
build\apex-rush.exe --stress 40 480 3 rpc   # 游戏层堆积撞击（QA 钩子）
build\rpc-pileup-tests.exe --cars 24 --seconds 3 --spread 0   # 引擎层对撞
build\rpc-pileup-tests.exe --cars 40 --seconds 2              # 引擎层堆积
build\rpc-pileup-tests.exe --info                             # 数据结构尺寸
```

`--bench` 现在支持 `<cars> <hz> <engine>`；`rpc-pileup-tests` 是 ctest 用例（`rpc-pileup`），失败条件是非有限状态或车被弹飞。

### 1.2 频率矩阵（120 车，rpc，600 帧 = 10 s 仿真）

| 频率 | physSteps | 每子步 vehicle | 每子步 world | 每帧物理 | 帧 1% low | 墙钟 |
|---|---:|---:|---:|---:|---:|---:|
| 120 Hz | 1 200 | 0.870 ms | 1.238 ms | 4.82 ms | 8.6 ms (116 fps) | 2.90 s |
| 240 Hz | 2 400 | 0.857 ms | 0.928 ms | 7.74 ms | 12.3 ms (81 fps) | 4.65 s |
| 480 Hz | 4 800 | 0.848 ms | 0.671 ms | **12.76 ms** | 17.9 ms (56 fps) | 7.66 s |

（第三轮实测，射线已能命中车辆。**每子步 vehicle 与频率无关**，因为它按子步固定做 4 条悬挂射线；world 随子步变便宜：120 Hz 每子步 4 倍位移 → 421 个代理越界、2 376 个候选对，480 Hz 只有 168 / 926。所以"提高子步率"的总成本是超线性的，这也是 §3.5 TGS 的出发点。）

（迭代数按频率夹在 4~16：240 Hz 16 次、480 Hz 8 次、120 Hz 16 次。120 Hz 若按线性放大到 32 次，world 会涨到 1.89 ms——已实测过，纯浪费。）

对照 qu3e（同机同负载，480 Hz，接入射线加速器之前）：vehicle 1.07 + world 1.31 = 2.38 ms/子步 → 19.0 ms/帧。**射线路径优化后 RPC 已反超**：vehicle 0.56 < 1.07，world 1.19 < 1.31。

### 1.3 单子步成本拆分（引擎层实测）

| 场景 | narrow（窄相） | build（岛构建） | solve（求解） | sync（宽相+同步） | 单步合计 |
|---|---:|---:|---:|---:|---:|
| 24 车对撞 @480 Hz | 0.10 ms | 0.20 ms | 0.03 ms | 0.12 ms | 0.35 ms |
| 40 车堆积（首步最差） | 1.7 ms | 0.5 ms | 2.2 ms | **72 ms** | 76 ms |
| 40 车堆积（稳态） | 0.03 ms | 0.05 ms | 0.07 ms | 0.35 ms | 0.50 ms |

推出两个**每单位成本常数**（后面所有估算都用它们）：

- 窄相：`0.19 µs / 接触对`（1.7 ms ÷ 9 146）
- 求解：`≈15 ns / 接触点 / 迭代`（2.2 ms ÷ 9 146 ÷ 16）
- 宽相：`≈3.3 µs / 每车每子步`（11 个盒的 AABB + 其中 ~1.4 个越界盒的两棵树更新 + 每步 ~7.7 个候选对；§1.7.2 实测，含射线树维护）
- 悬挂射线：`2.32 µs / 条`（单树）→ `1.19 µs / 条`（网格 + 动态树，**且真的能命中车辆**）

### 1.4 本轮已落地项的效果（同一台机，实测）

| 指标 | 修复前 | 修复后 | 倍数 |
|---|---:|---:|---:|
| 堆积峰值接触数（40 车） | 68 680 | 9 146 | 7.5× |
| 堆积最差单步 | 2 500 ms | 76 ms | 33× |
| 岛池内存 | 3 231 MB | 3.1 MB | 1 000× |
| 被弹飞的车 | 21/40（最远 1 162 m） | 0 | — |
| 游戏内 `--stress 40 480 3` 墙钟 | 29.2 s | 1.6 s | 18× |
| 其中 world.Step / 子步 | 19.2 ms | 0.70 ms | 27× |
| 同场景最大车速 | 9 608 m/s | 70.9 m/s | — |
| `--bench 120 480 rpc` world / 子步 | 1.41 ms（margin 0.5） | 1.13 ms（margin 0.2） | 1.25× |
| `--bench 120 120 rpc` 帧 1% low | 13.4 ms（32 迭代） | 10.6 ms（16 迭代） | 1.26× |

### 1.5 本轮已落地（第二轮）：可插拔射线加速器

按 §3.1 / §3.2 的方向实现，但**接线方式与文档初稿不同**：不动接触/求解管线，只把射线查询抽成接口。原因见 §1.6.2 的实测——动态侧线性扫描在 120 车时会输给树，必须能换。

**接口与实现**（`broadphase/rpcRayIndex.h`）：

```
rpcRayIndex                      // 五个动词：Add / Remove / Update / Clear / Raycast(+Batch)
 ├─ rpcStaticGrid               // 实现 A：静态几何 XZ 均匀网格（CSR + 戳记去重 + 2D DDA）
 ├─ rpcDynamicTreeRayIndex      // 实现 B：动态几何专用 BVH（与宽相树分离，叶子少一个量级）
 ├─ rpcDynamicSweep             // 实现 C：动态体线性扫描（小规模场景的最优解）
 └─ rpcRayIndexFactory          // 构造期注入（测试注入 mock，第三方实现只需满足接口）
```

- **分层不动**：接口在 L1 内部，L2 只经 `rpcScene` 的既有 `RayCast/RayCastBatch` 抵达，游戏层零改动。
- **依赖倒置**：`rpcDynamicSweep` 通过 `rpcDynamicBodySource` 取体列表，场景实现它，测试可换自己的列表。
- **策略可换**：`rpcSceneConfig { rayStrategy, dynamicRay, staticGridCell, rayFactory }`；`Tree` 保留为参考实现（原 BVH 路径），随时可 A/B。
- **确定性**：closest-hit 的**距离并列**用 `rpcBox::uid` 决胜，因此"树 / 网格 / 注入实现"对同一射线返回**同一个形状**。

**一致性验证**（新 ctest 用例 `rpc-ray-index`，`tests/test_ray_index.cpp`）：同一场景建三份（树 / 索引化 / 注入 mocks），4000 条随机射线（垂直、浅掠、任意方向、含退化情形）与**暴力全扫**逐条对比：

```
conformance: rays=4000 hits=1273 failures=0
injection: static=1 dynamic=1  (factory used)
```

**实测收益**（120 车，RPC，`--bench 120 <hz> rpc`）：

| 指标 | 优化前（单树） | 现在（网格 + 动态树） | 变化 |
|---|---:|---:|---:|
| 悬挂射线 / 条 | 2.32 µs | 0.53 µs → **1.19 µs**（修掉动态侧失明后，§1.7.1） | -49% |
| vehicle / 子步 @480 Hz | 1.45 ms | 0.54 → **0.848 ms**（含对车辆的真实命中） | -42% |
| vehicle 随车数扩展 | — | 24 车 0.51 / 60 车 0.52 / 120 车 0.53 µs/条（**失明时测的，仅作历史**） | — |
| 每帧物理 @480 Hz | 20.6 ms | 11.5 ms → **12.76 ms** | -38% |
| 帧 1% low @480 Hz | 27.5 ms | **15.1 ms（66 fps）** | -45% |
| 墙钟 @480 / 240 / 120 Hz | 12.7 / 7.0 / 4.1 s | 7.3 / 4.4 / 2.7 s → **7.66 / 4.65 / 2.90 s** | -35~42% |
| world.Step / 子步 | 1.13 ms | 1.19 → **0.671 ms** | -41% |

**§1.6.2 两个被实测否掉的方案**（记录下来避免重复讨论）：

1. **动态侧线性扫描（sweep）**：24 体时 0.39 µs/ray 胜过树 0.61 µs，但成本随体数线性增长——24 车 0.95 µs、60 车 1.36 µs、120 车 2.00 µs，在 120 车时反而输给动态树（0.52 µs）。所以 sweep 只作为小场景选项保留，默认用动态树。
2. **4 射线 body-major 批处理**：把 4 条射线在一次遍历里测完，实测**慢 30%**（逐射线版本能在一个轴测试后就早退，body-major 必须先算完所有射线才能跳过该体）。已回退，理由写在 `rpcDynamicSweep.h` 里。

**子步内拆分（480 Hz / 120 车，--bench 新输出的 stage ms，全程均值）**：

| 段 | 每子步 | 说明 |
|---|---:|---|
| proxy（AABB + 索引更新） | 0.110 ms | 1216 个动态盒；只更新越过 margin 的 |
| pairs（配对生成） | 0.161 ms | 移动盒树查询 + 配对排序 + 建接触 |
| narrow（窄相） | 0.151 ms | ~1100 个接触的 manifold 生成 |
| build（岛构建） | **0.026 ms** | 120 个岛；曾因遍历 2000+ 静态体高达 0.27 |
| solve（迭代） | 0.036 ms | 480 Hz 下 8 次迭代 |
| sync 其余（力的清除、岛簿记、统计） | ~0.30 ms | 其中 0.12 是 CollectStats 自己的遍历（已改为 opt-in，§1.7.2） |

两条结论：

1. **代理维护 + 配对生成并列第一**（修正后 proxy 0.187 + pairs 0.205 = 0.39 ms，占世界步 60%），来源仍是"每车 11 个盒、每盒每子步一次 AABB + 最多两次树更新"，与 §2.1 #2/#12 的"每车单代理"对应。注意 §1.7.4 的实测：单代理会减少代理更新，但会把宽相筛选挪成"体对 × 121 个盒对"的本地测试，**不是免费**。
2. 维护第二棵（射线）树有代价，但已在 §1.7.3 用"祖先重构"抵掉一部分：tree Update 只在父节点装不下时才结构重插。

> 度量口径提醒：SetDiagnostics 打开后有 4 次时钟读取/步；CollectStats 的深计数（接触点/awake 体）已改为 SetDeepStats 单独开关，**--bench 不再为测量工具本身付费**（此前每步遍历全部 body/contact，单跑约虚高 1.5 s）。
**同轮落地的关键改动（按收益排序）**：
1. 稠密动态体表替代 `m_bodyList` 遍历（代理同步 / 力清除 / eIsland 清除 / 岛种子）：单子步 -0.45 ms；
2. 静态几何网格 + 动态专用 BVH 的射线路径（`vehicle` 1.45 → 0.57 ms/子步）；
3. 一次包含测试同时驱动宽相树与射线树（`proxy` 减半）。

**P0 卫生项**：`rpcBox` 增加 `uid` / `rayIndexId` / 缓存 `fatAABB`（射线与窄相共用同一个"fat"定义 `rpcFatten`）；`rpcBox::WorldTransform` 走"无局部旋转"快路径（省掉每叶一次 3×3 乘）；worker pool 热计数器按 64 B 缓存行隔离；`rpcAABB` / `rpcContactState` 尺寸编译期断言；`[[assume]]` / `[[likely]]` 铺在动态扫描的拒绝分支上。

### 1.6 数据结构尺寸（`--info` 实测）


| 类型 | 字节 | 说明 |
|---|---:|---|
| `rpcVec3` / `rpcAABB` | 12 / 24 | AABB 两个 Vec3 |
| `rpcTransform` / `rpcMat3` | 48 / 36 | 旋转 36 + 位置 12 |
| `rpcBox` | **120** | 缓存 `fatAABB`(24 B) + `uid`/`rayIndexId`；`local` 占 48 B 且绝大多数无旋转 |
| `rpcBody` | **296** | 热字段（速度/世界中心/惯量）与冷字段（userData/damping/层级）交错 |
| `rpcManifold` | 496 | 固定 `contacts[8]`，实际常用 4 |
| `rpcContactConstraint` | **624** | 每接触对常驻，含完整 manifold |
| `rpcContactConstraintState` | **608** | 每接触对的求解工作区 |
| `rpcContact` / `rpcContactState` | 52 / 56 | 单个接触点 |
| 树节点（推导） | **48** | `2×AABB + 4 + 8 + 8 + 4`，1.33 节点/缓存行 |

---

## 1.7 第三轮：度量闭合 → 一个正确性 bug → 代理更新快路径

出发点不是"再优化点什么"，而是上一轮 §1.5 的拆分加起来比整步少了 0.30 ms。先把账算平，再谈优化。

### 1.7.1 度量闭合，把"对不上的时间"变成可归因的分段

新增分段 `force`（清力）与整步 `step`，并把适配层（`rpc_impl.cpp`）自己的外壳算作 `wrap`（冲量聚合 + 接触事件发布——它天然在任何物理分段之外，所以以前的报告永远看不到它）。子步总账变成**八段之和**：

```
step=0.6492  segs=0.6490  unaccounted=-0.0002  wrap=0.0214  force=0.0006   (ms/子步)
```

`unaccounted ≈ -0.0002` 就是十次时钟读取本身。**账算平之后，那 0.30 ms 立刻露出真身**：

- **0.12 ms = `CollectStats` 自己**。每子步遍历接触表（1 000+）和 `m_bodyList`（2 000+ 静态体），而它不属于任何分段。`m_deepStats` 这个开关此前**只赋值、从未被读取**。改为默认只做 O(1) 计数，遍历部分真正 opt-in。副作用：`points` 在 bench 里显示 0（那是深计数，默认关）。
- **0.10 ms = 冲量聚合的 O(forces × bodies) 扫描**。`Step()` 对每条缓冲力线性查找已聚合槽位，120 车约 11 力/车。改成按句柄索引的槽位（句柄稠密且不复用，**句柄即槽位**，戳记免清零）：`wrap 0.127 → 0.022 ms/子步`。
- 顺带发现 `ReadStageStats` 的 `syncMs` 漏了均值化（取的是末个子步原始值，会比真实均值大 2 倍），以及 240 帧倒计时预热（1 920 子步、几乎无接触）被算进均值——新增 `ResetStageStats()`，`ResetBench()` 调用它。

### 1.7.2 修正后的子步拆分（120 车 / 480 Hz，均值）

| 段 | 每子步 | 占比 | 备注 |
|---|---:|---:|---|
| proxy | 0.187 ms | 29% | 1 216 动态盒算 AABB + 168 个越界盒的两棵树更新 |
| pairs | 0.205 ms | 32% | 其中 tree query 0.137 + sort/建接触 0.066 |
| narrow | 0.190 ms | 30% | 1 065 个持久接触逐子步重解 manifold |
| solve | 0.036 ms | 6% | 8 次迭代 |
| build | 0.031 ms | 5% | 120 个岛 |
| force | 0.001 ms | — | 清力 |
| **step 合计** | **0.649 ms** | | 与八段之和误差 0.0002 |
| wrap（适配层） | 0.021 ms | | 冲量聚合 + 事件发布 |

配套计数器（O(1)，随 bench 输出）：`moved=168  produced=926  unique=809`。这三个数说明宽相的真实规模——**每子步只有 13% 的盒越过 margin**，因为 480 Hz 下车辆位移 0.167 m 与 0.2 m margin 同量级。

### 1.7.3 动态射线索引一直是空的（正确性 bug）

`rpc-ray-index` 一致性测试报 8 条射线与暴力全扫不一致。一开始以为是加速器实现问题，隔离实验（`--dynamic 0` → 0 失败；120 动态体 → 8 失败）指向动态侧，最终定位到：

```cpp
// 修复前：只转发静态盒
if ( ... && ( box->body->m_flags & rpcBody::eStatic ) )
    m_staticRayIndex->Add( box, fatAabb );
```

**动态盒从来没进过 `m_dynamicRayIndex`**——那棵"动态专用 BVH"一个叶子都没有，所有射线对车辆失明（悬挂射线只能看到路面）。改为 `RayIndexForBox(box)` 按体类型路由（静态→网格，动态→移动索引，`Tree` 策略下两者为 NULL 自然退回整树查询）。

这个 bug 之所以活了两轮，是因为**测试本身编译不过**：`test_ray_index.cpp` 里的 mock 没有跟上 `Add/Remove/Update/Raycast` 新增的 `ignore` 参数，`CountingRayIndex` 成了抽象类；而 `ctest` 只是运行已存在的二进制，不重新构建，所以一直跑的是旧二进制（它当然通过）。教训：**ctest 之前必须确认测试目标真的重建过**。

代价（同场景，实测）：`raycast 0.56 → 1.19 µs/条`、`proxy 0.150 → 0.223`、墙钟 `6.99 → 8.42 s`。这部分"变贵"是**正确性**，不是回退。

### 1.7.4 树更新：祖先重构快路径

`rpcDynamicAABBTree::Update` 原来是"`Contains` 不成立就 `RemoveLeaf` + `InsertLeaf`"——后者会重新平衡整棵树（含旋转），实测约是纯遍历/重构的十倍。而 480 Hz 下车辆每子步 0.167 m 刚好越过 0.2 m margin，**几乎每次更新都在走最贵的路径**。

加 Box2D 式的三段判定：仍在叶子 AABB 内 → 不动；仍在**父节点** AABB 内 → 只把新 AABB 写进叶子并沿祖先链重构（`RefitAncestors`，碰到已容纳的节点就停）；否则才结构重插。树形不变、高度不变，`Validate()` 的不变式仍成立。

效果：`proxy 0.223 → 0.187 ms/子步`（-16%），树测试 `tree leaves=4079 height=12` 不变，`rpc-ray-index` 仍 0 失败。

### 1.7.5 重测 AABB margin（结论：0.2 仍是最优，但原因和上次不同）

有了便宜的 refit，margin 的天平理论上应偏向"少更新"。实测 margin 0.3：

| margin | proxy | pairs | narrow | step | 接触数 |
|---|---:|---:|---:|---:|---:|
| 0.2 | 0.187 | 0.205 | 0.190 | **0.649** | 1 065 |
| 0.3 | 0.144 | 0.217 | **0.432** | 0.894 | 2 296 |

proxy 确实降了 22%，但**接触数翻倍**，narrow 直接 ×2.3——因为宽相候选对会全部变成持久约束（`AddContact` 只看 fat AABB 叠加），每子步都要重解 manifold。所以 margin 的真正代价在窄相，不在宽相，0.2 m 保留。

### 1.7.6 不要做的三件事（本轮否决）

1. **"每车单代理"不是免费收益**：它会减少代理更新（1 216 → ~130 AABB），但宽相筛选从"树只返回真正重叠的盒对"变成"体对 × 11×11 个盒对的本地测试"，`pairs` 会变贵。要动必须先有 §3.4 的持久 manifold 打底。
2. **不要用 margin 换 proxy**：见 §1.7.5。
3. **不要相信没有"闭合"的拆分**：分段之和必须等于整步，否则没被归因的那部分会被误判成"某个阶段贵"。

---

## 2. 现状热路径清单与复杂度

`B` = 刚体数、`X` = 盒数、`D` = 动态体数、`C` = 接触对数、`P` = 接触点数、`I` = 迭代数、`S` = 子步数。

| 阶段 | 函数 | 时间复杂度 | 空间复杂度 | 每子步实测量（120 车） | 主要问题 |
|---|---|---|---|---|---|
| 宽相同步 | `rpcScene::SyncAndFind` → `rpcBody::SynchronizeProxies` | O(D·box) = O(D·11) | 0 | 含在 world 段 | 每个小盒各自 `ComputeAABB` + 树 update；`local` 恒 identity 也做全量变换合成 |
| 宽相配对 | `rpcBroadPhase::UpdatePairs` | O(X_moved·log X + P log P) | O(P) | 同上 | 每个移动盒一次树查询；配对缓冲每步 `std::sort`（堆排序分配） |
| 接触去重 | `rpcContactManager::AddContact` | O(E_body) → 已封顶 O(k + E) | — | 同上 | 每新增一对线性扫 body 的边表（现由"动态对预算 12"间接封顶） |
| 窄相 | `TestCollisions` → `rpcBoxtoBox` | O(C) 每子步全量重算，单对 ~O(15 轴 + 裁剪) | 0 | 0.19 µs/对 | 无持久化：每子步重建 manifold，warm start 靠 feature key 匹配 |
| 岛构建 | `BuildIsland` + `Initialize` | O(D + C) | **O(Σ 岛内容)**（本轮修好） | 0.05~0.20 ms | 岛内体表/接触表每步重建；`Initialize` 逐接触拷贝 608 B |
| 求解 | `rpcContactSolver::PreSolve/Solve` | O(I·P) | O(C·608 B) | 2.2 ms @ 9146 接触 | AoS 布局、每迭代重读 608 B 状态、无 SIMD、无拓扑排序 |
| 位置修正 | `PreSolve` 的 Baumgarte 项 | O(1)/点 | 0 | — | 速度级修正（本轮加钳制），有能量注入 |
| 射线 | `RayCastBatch` → 树遍历 + `rpcBox::Raycast` | O(log X) ~ 27 节点 | 0 | **1.45 ms（vehicle 段 90%）** | 每次查询从根开始；每 leaf 一次 `rpcMul(tx, local)`；每命中一次 `unordered_map` 反查 |
| 同步回写 | `Island::Solve` 尾段 | O(D) | 0 | — | 逐体回写速度/位置 |

**关键结论**：这张表里没有 O(n²) 级算法；真正的开销来自 **(a) 每子步重复的常数项**、**(b) 字节数/缓存不友好**、**(c) 子步数被乘进去**。所以优化重心是"降低常数 + 减少字节 + 解耦子步"，而不是换更花哨的复杂度。

### 2.1 逐模块算法替换清单（"这里该换什么算法"）

按"能不能换算法"逐条过一遍 L1/L2 的热点。**O(·)** 列为该模块自身的复杂度；"收益"列是按前面实测常数的估算。

| # | 位置 | 现算法 | 更优算法 | 复杂度变化 | 空间变化 | 收益 |
|---|---|---|---|---|---|---|
| 1 | `broadphase/rpcDynamicAABBTree.*` | 二叉 BVH，48 B/节点，标量 slab 遍历，从根查起 | **量化 4 路 BVH**（16-bit 相对父节点边界，单指令 4 通道） | O(log X) 同阶，**每层字节数 ÷3** | 317 KB → ~110 KB | 遍历 miss -50~70% |
| 2 | `broadphase/rpcBroadPhase.cpp` `UpdatePairs` | 每移动盒一次树查询 → 全量 `std::sort` 配对缓冲 | **静态几何 XZ 网格 + 动态 BVH 分层**；动态侧每车单代理；配对改计数排序/哈希去重 | O(X_moved·log X + P log P) → **O(X_moved + P)** | +120 KB 网格（换掉树的一半） | 宽相 1.5 µs/车 → ~0.5 µs |
| 3 | `dynamics/rpcContactManager.cpp` `TestCollisions` | 每子步全量重算 manifold（SAT + 裁剪） | **持久 manifold**（局部坐标锚点 + 特征 key 复用），只在失效时重算 | 每对 O(SAT) → **O(点数)** | 接触对常驻 +52 B，但省掉每步重建 | 窄相 -60~70% |
| 4 | `dynamics/rpcContactSolver.cpp` | PGS + Baumgarte，AoS（608 B/对），无拓扑序 | **soft constraint + relaxation**（Bepu 三系数），状态 **SoA**，岛内 **graph coloring** 后可 SIMD | O(I·P) 同阶，**常数 ÷2~3** | 608 B/对 → 热字段 ~24 B/点 | 求解 -40~60%，且不再弹射 |
| 5 | `dynamics/rpcIsland.cpp` + `scene/rpcScene.cpp` `SolveIslandsBatched` | 每子步 DFS 重建全部岛；车-静态接触参与连通 | **接触图分层**（车↔静态边不入岛）+ 增量岛更新（只重算涉事车群） | O(D + C) → **O(Δ)** | 不变（本轮已修成按需） | 岛构建 0.2 ms → <0.05 ms |
| 6 | `scene/rpcScene.cpp` `Step`（被游戏按 `physics.hz` 驱动） | 每子步 = 宽相 + 窄相 + 岛 + 求解 | **TGS 子步**：外步宽相/窄相一次，内步只求解（锚点重投影） | S×(宽+窄) → 1×(宽+窄) + S×求解 | 不变 | 480 Hz 每帧 20.8 → ~9 ms |
| 7 | `common/rpcMemory.cpp` `rpcHeap` | 20 MB 固定块 + first-fit 线性扫描 | **arena + 尺寸分级池**，帧末整体回收 | 分配 O(n_free) → **O(1)** | 由 `PoolStats` 控峰值 | 分配抖动归零 |
| 8 | `src/physics/rpc_impl.cpp` `boxToBody` | `unordered_map<const rpcBox*, BodyHandle>`，每次命中一次哈希 | **叶 id → body 稠密数组**（树节点里直接存 handle） | O(1) 但常数 ÷5~10 | +4 B/叶 | 每命中省 ~50-100 cycles |
| 9 | 同上 `ClosestHitCallback` / `BatchClosestHitCallback` | 虚函数 `ReportShape` 每叶子一次 + 每命中一次哈希 | **模板 callback**（`deducing this` 静态多态）+ 数组索引 | 同阶，去虚调用 | 0 | 每查询省 20~50 cycles |
| 10 | `collision/rpcBox.cpp` `Raycast` / `ComputeAABB` | 每次 `rpcMul(tx, local)` 合成变换（48 B 矩阵乘加） | 缓存世界变换 / `local == identity` 特化 / 组内共享 body 变换 | 同阶，常数 ÷2 | +48 B/盒（可选） | 每次叶测试省 ~30 cycles |
| 11 | `collision/rpcCollide.cpp` `rpcBoxtoBox`/`rpcClip` | 15 轴 SAT + Sutherland-Hodgman 裁剪 | **保留**（已是 Box2D/qu3e 系主流），仅加退化剔除（背面/零面积/`denom→0` 保护） | 同阶 | 0 | 数值健壮性（避免 `rpcEdgesContact` 除零） |
| 12 | `dynamics/rpcBody.cpp` `SynchronizeProxies` | 逐盒 `ComputeAABB` + 树 update | 每车一次合并 AABB + 组内 `local` 特化 | O(D·11) → **O(D)** | 0 | 见 #2 |

> 结论：**能"换算法"的只有宽相（#1/#2）、窄相（#3）、求解器（#4）、岛（#5）、子步策略（#6）**；#11 这种窄相几何本身已经是最优一族，别再折腾。

### 2.2 为什么不是"换更小的复杂度"就完事

一个容易踩的坑：把宽相换成"哈希网格"看起来是把 O(log X) 变成 O(1)，但**哈希本身也有常数**（算 key、查表、跨 cell 重复）。上表 #2 的收益来自两件更朴素的事：

1. 静态几何**永不移动**，不该和 1320 个动态叶子抢同一棵树（局部性）；
2. 一辆车只该做**一次**代理更新/查询，而不是 11 次（常数）。

同理 #1 的收益来自"每层字节数"而不是"层数"：13 层 × 48 B 的随机跳转，压缩到 13 层 × 16~24 B 就是 2~3 倍的 miss 减少。**在访存受限的内核里，"每字节能装多少几何"比"渐进复杂度"更值钱。**

### 2.3 顺带发现的正确性/健壮性问题（不是性能，但建议顺手修）

| # | 位置 | 现象 | 建议 |
|---|---|---|---|
| 1 | `broadphase/rpcDynamicAABBTree.cpp` `AllocateNode` | 容量翻倍时 `AddToFreeList(m_count)` 会**丢掉已有的空闲链**（`DeallocateNode` 压进去的、id < m_count 的空闲槽既不在新空闲链里也不是活节点），节点表只增不减，长时间运行会慢慢泄 | 改 `AddToFreeList` 为"把旧空闲链整体接到新链尾部"，或直接改成索引池 + 显式 freelist 数组 |
| 2 | `dynamics/rpcContactManager.cpp` `ContactStillValid` | 用 `box->broadPhaseIndex` 直接查树；若某盒已被移除（`Remove` 后索引仍是旧值/`Null`），`GetFatAABB(-1)` 会越界读 | 加 `assert(id >= 0 && id < m_capacity)` 并在 `RemoveBox` 里把索引置 `Null`；`TestOverlap` 提前返回 false |
| 3 | `collision/rpcBox.cpp` `Raycast` | 射线起点在盒内时返回 `toi = 0` 但**法线未初始化**（调用方 `ClosestHitCallback` 已注释并跳过 `t <= 0` 的命中） | 让 `Raycast` 返回 `bool` 之外再给出 `startedInside` 标志，或保证 `n0` 有确定值；避免每个调用方各自打补丁 |
| 4 | `collision/rpcCollide.cpp` `rpcEdgesContact` | `denom = a*e - b*b` 在近平行边时会接近 0，`TA/TB` 爆大 → 接触点坐标变 NaN/巨大 | 加 `if (denom < eps) 退化处理`（退回面接触或跳过该点），从源头免掉"坐标飞了"的一类问题 |

---

## 3. 算法层优化

### 3.1 静态几何专用索引：从 BVH 换成"网格 + 分块列表"（P1，收益最大）

**现状**：赛道是 `Resample(map, 4 m)` 后每 4 m 采样生成 6 个 slab（路面 / 左右草地 / 左右砂石 / 挡墙），约 **2 000 个静态 body、每个 body 1 个 box**，全部塞进同一棵动态树。悬挂射线本来只需要"车轮正下方 0.7 m 内的路面/路肩"，却要在一棵 ~3 300 叶子的树里走 13 层。

**改法**：静态几何单独建 **XZ 均匀网格**（cell = 8 m，与 4 m slab 长度对齐），cell 内放候选 box 的紧凑索引；查询时按车轮位置算 cell → 取 1~3 个候选 → 直接 `rpcBox::Raycast`。动态几何（车）仍用 BVH，两者查询结果取最近。

- 复杂度：射线 `O(log X)` → **O(1)**（候选常数 1~3）；构建 `O(X)`，赛道一次构建、永不变。
- 空间：`cell_count × 4 B` 偏移表 + `Σcell 候选 × 4 B`；按 1.6 km、路宽 12 m + 缓冲区 → ~4 000 cells × ~6 候选 ≈ 120 KB（远小于 BVH 的 317 KB 节点）。**顺序访问**，无指针追逐。
- 预期：射线 2.32 µs → 0.4~0.6 µs；vehicle 段 1.45 → **0.5~0.7 ms/子步**（120 车）。
- 风险：跨 slab 缝隙（相邻 cell 边界）需查询 2×2 邻域；需要用 `--drive-test` 的接地点/轮荷回归。
- 参考实现思路：Jolt 的宽相是 64 m 分块四叉树 + broadphase layer 分离（静态/动态分开查询），与本方案同构；Box2D v3 的动态树查询用 SIMD 一次测 4 个 AABB，可作为"仍用树"时的对照方案。

### 3.2 BVH 节点压缩 + 4 路打包（P1）

**现状**：节点 48 B（`2×AABB + parent/next + left/right + userData + height`），13 层；`Query`/`QueryBatch` 标量遍历，`QueryBatch` 每访问一个节点对 4 条射线各做一遍 slab 测试。

**改法**（两步，可分两次验收）：

1. **量化 AABB**：把 `float` AABB 换成相对父节点的 16-bit 量化整数（Embree/`bvh_quantized` 的做法）。节点 48 B → **16~24 B**，同样 64 B 缓存行能装 3~4 个节点，13 层遍历的 miss 次数直接减半以上。解包用 `std::bit_cast` + `std::byteswap`（C++23），配合 `[[assume]]` 保证量化范围。
2. **4 路节点 + 单指令 4 通道测试**：把二叉改四叉（一次取 4 个子节点），每个节点一次 `_mm256_*` 完成 4 条射线的 slab 测试（或 4 个子节点的同一射线测试）。`std::simd` 目前**不可用**（见 §5 实测），用 `<immintrin.h>` 薄封装成 `rpc_simd.h`，接口命名对齐 `std::simd`，将来一键替换。

- 复杂度：不变（O(log X)），但**每次遍历的字节数与分支数**下降 2~4×。
- 预期：射线 2.32 µs → 0.7~1.0 µs（不换网格的前提）；宽相配对查询同步受益。
- 风险：量化误差需要保守膨胀（+1~2 cm）；`Query`/`QueryBatch`/`QueryAABB` 三处语义必须一致，用 `rpc-pileup` + `--drive-test` 逐位对照。

### 3.3 每车单代理宽相（P1）

**现状**：一辆车 11 个盒 → 每子步 11 次 `ComputeAABB` + 11 次树 update + 1~11 次树查询。其中 leaf 的 `local` 恒为 identity，`rpcMul(tx, local)` 每盒每步都白算一次（48 B 的矩阵乘加）。

**改法**：
- 树中仍保留 11 个叶子（窄相精度不变），但**移动缓冲与查询按车体合并 AABB 做一次**：车体 AABB 超标才把 11 个叶子一起更新并做一次查询，得到的候选按 body 去重后进窄相。
- `local == identity` 走特化路径（`rpcBox` 存一个 `bool hasLocal` 或按 body 分组连续存放），省掉每盒的变换合成。
- 复杂度：O(D·11) → **O(D)**；宽相 1.5 µs/车/子步 → ~0.5 µs。
- 风险：合并 AABB 会引入更多候选（假重叠），需要配 §4.1 的空间排序压住。

### 3.4 持久 manifold + 接触回收（P1）

**现状**：`rpcManifold` 每步由 `rpcBoxtoBox` 全量重建（496 B/对），warm start 只在 feature pair 命中时恢复冲量；每新增一对就 `Allocate()` 一块 624 B。

**改法**：
- 保存上一帧 manifold 的**局部坐标接触点 + 特征 id + 法线/切线 + 累积冲量**，本步先用"局部点 → 世界点"重投影，只有分离超出阈值或特征失效时才跑完整 SAT/裁剪。这就是 Box2D/Bullet 的 persistent manifold，也是 §3.6 的 TGS 前提。
- 接触对象池化 + 回收（按 pair key 复用块，避免每帧分配/释放 624 B）。
- 复杂度：稳态下窄相从"每对每步 O(SAT)"降到"每对每步 O(点数)"，实测 `0.19 µs/对` → 预计 **0.05~0.08 µs/对**；warm start 质量提升，堆叠更稳。
- 风险：特征 id 匹配逻辑要单测（现有 `WarmStartMatch` 已有一层，可复用）。

### 3.5 TGS 子步：把"子步数"从宽相/窄相里拿掉（P2，480 Hz 的关键）

**现状**：480 Hz = 每帧 8 个子步，**每个子步都完整跑一遍宽相 + 窄相 + 岛构建 + 求解**。宽相/窄相的成本被乘了 8。

**改法**（Erin Catto 在 Box2D v3 落地的路子，见文献 [1]）：一个"外步"里宽相/窄相只做一次，得到接触集合；随后 **N 次内子步只做求解**，接触点用**局部坐标锚点**跨子步重投影（`separation = dot(pB + rB − pA − rA, n) + originalSeparation`），不重算 manifold；位置积分用 `deltaPosition` 累加再一次性写回（避免大坐标下的舍入误差）。

- 复杂度：把 `S × (宽相 + 窄相)` 变成 `1 × (宽相 + 窄相) + S × 求解`。以 480 Hz/120 车计：宽相+窄相约 0.5 ms/子步 → **外步 0.5 ms + 8×(求解 0.65 ms)**，每帧从 20.8 ms 降到 ~9 ms。
- 与 §3.6 的关系：TGS 天然配 soft constraint / relaxation（不需要速度级 Baumgarte，也就不会弹射）。
- 风险：确定性档要重新基线（`--drive-test` 的数值断言可能变化）；手感需要复标定（`physics-upgrade.md` §8 全项重跑）。
- 注意：我们的车体是**射线悬挂**而非接触堆叠，TGS 的收益主要在"车-车/车-墙接触"和"接触很多"的场景——正好是当前最痛的场景。

### 3.6 求解器换代：PGS+Baumgarte → soft constraint + relaxation（P2）

**现状**：`PreSolve` 里 `bias = -β·(1/dt)·min(0, pen + slop)`（本轮加钳制）。这是 Box2D 2006 的做法：直接往速度里注入位置修正能量，深穿透就会弹射，钳制只是止损。

**改法**（按收益/复杂度递增）：

1. **Soft constraint**（Bepu 化简后的三系数形式，只需 Hertz、阻尼比、dt）：把接触写成"弹簧-阻尼"，`incrementalImpulse = -massCoeff·meff·(vn + biasRate·separation) - impulseCoeff·accumulatedImpulse`。参数直观（Hz/ζ），深穿透不再弹射，且**没有 1/dt 爆炸**。
2. **Relaxation**：先带 bias 求解 + 积分位置，再**去掉 bias 求解一次**（不积分位置），把注入的能量收回。Box2D 作者称"贵但质量提升明显"。
3. **NGS 位置求解**（可选）：速度解完之后用伪速度解位置约束，完全不改动能。

- 收益：可以**删掉**本轮的 Baumgarte 钳制与速度钳制这两个止损补丁；堆叠/挤压更稳；每个接触点每迭代的算术量略降（少了 `min` 与钳制分支）。
- 风险：手感变化，需要复标定；需要保留旧路径做 A/B（用 `[physics] solver = pgs | soft` 配置开关，`--drive-test` 双路径都跑）。

### 3.7 岛调度与睡眠（P2）

**现状**：`SetAllowSleep(false)`（车辆必须常唤醒），静态体不入岛，但**每子步都对全部动态体做觉醒遍历 + 岛重建**；并行的门控是 `islandCount ≥ 4×workers`，120 车时 island 数多在 20~120 之间波动，并行收益不稳定。

**改法**：
- **接触图分层**：把"车辆 vs 静态路面"的接触不参与岛连通（车与地面不是同一刚体系统），使岛基本等于"互相碰撞的车群"，岛数 ≈ 车群数。这样岛更小、缓存更友好，也让并行门控更准。
- **自适应迭代**：按岛的大小/最大穿透深度给迭代数（小岛 4 次、大堆 8~16 次），而不是全局 16 次。
- **真正的睡眠**：赛道静态体已经是 static；对"撞墙后静止"的车可以局部冻结（车辆本身有 Reset 逻辑，需谨慎）。
- 复杂度：不变，常数降；并行扩展性变好（岛更均匀）。

### 3.8 复杂度与收益汇总

| 项 | 时间：改前 → 改后 | 空间：改前 → 改后 | 120 车/480 Hz 预期 |
|---|---|---|---|
| 3.1 静态网格 | 射线 O(log X) → **O(1)** | 树一半（317 KB）+ 网格 120 KB | vehicle 1.45 → 0.5~0.7 ms |
| 3.2 节点压缩 | O(log X) 常数 ÷2~4 | 48 B/节点 → 16~24 B/节点 | 射线/宽相查询各 -50% |
| 3.3 单代理 | O(D·11) → **O(D)** | 不变 | world 1.15 → 0.85 ms |
| 3.4 持久 manifold | 窄相 O(SAT)/对 → **O(点)/对** | +52 B/对常驻，-每步重建 | 窄相 -60~70% |
| 3.5 TGS | S×(宽相+窄相) → **1×+S×求解** | 不变 | 每帧 20.8 → ~9 ms |
| 3.6 soft constraint | O(I·P) 同阶，常数 ÷2 | 608 B/对 → 热字段 ~24 B/点 | 稳定性 + 求解 -40% |
| 3.7 岛调度 | O(D + C) → **O(Δ)** | 不变 | 岛构建 <0.05 ms |

叠加 3.1+3.2+3.3+3.4 后估算每子步 `1.45+1.15 = 2.60 ms` → `0.6+0.75 ≈ 1.35 ms`（每帧 10.8 ms）；再叠 TGS 可到 **~7 ms/帧**。这些是**按单点实测常数外推**，落地时必须逐项 ABAB 复测（§8）。

### 3.9 已评估但不推荐的方向（避免重复踩坑）

| 方案 | 为什么看起来好 | 为什么不选 |
|---|---|---|
| **SAP 扫描排序**（sweep and prune） | 增量插入排序 O(n + k)，世界变化小时极快；Bullet 的默认宽相之一 | 需要选一个主轴并保持稳定分布。我们的世界是**宽而扁的 2D 赛道 + 少量动态车**，X/Z 轴都会频繁触发全部重排，收益不稳定；而且它解决的是"物体沿单轴稀疏"的场景，不是"少数车在局部密集"的场景 |
| **动态侧也用空间哈希** | O(1) 插入 | 只有 120 个动态体，哈希的计算与跨 cell 重复成本 ≥ 树查询；哈希更适合**上万**均匀分布的物体 |
| **GPU 宽相/窄相** | 吞吐高 | 见 §4.6：数据量小、延迟敏感、读回成本高；本项目的物理在固定步进的仿真线程上 |
| **直接 LCP 求解器 / block solver** | 堆叠质量最好（Box2D 的 2 点 block solver） | 我们的瓶颈是**车-车/车-墙的稀疏接触**，不是金字塔堆叠；收益小、代码复杂度高。若将来要做"翻车压车"的稳定堆叠再考虑 |
| **Featherstone 多体 / 完整悬架运动学** | 物理更真实 | 这是**建模**选择，不是性能优化；替换的是射线悬挂模型（`physics-upgrade.md` §0.3 已明确不做），而且自由度更多 = 更慢 |
| **CCD（连续碰撞检测）** | 消除高速穿模 | 需要扫掠体 + TOI，成本是普通的数倍；现行方案是"480 Hz 子步 + 薄盒加厚"（已记录为已知限制） |

---

## 4. 硬件层优化

### 4.1 容量算术：什么装得下、什么装不下

本机 **L2 = 256 KB/core（WMI 报 4 MB 是 16 核合计）**、L3 = 40 MB。

| 工作集 | 大小 | 与缓存的关系 |
|---|---:|---|
| 赛道 BVH 节点（3 300 叶 → 6 600 节点 × 48 B） | **317 KB** | **超 L2**，每次查询落到 L3 |
| `rpcBox`（3 300 × 112 B） | 370 KB | 超 L2 |
| `rpcBody`（2 120 × 296 B） | 627 KB | 超 L2 |
| 接触对常驻（9 146 × 624 B） | **5.7 MB** | L3 |
| 接触求解工作区（9 146 × 608 B） | **5.6 MB** | L3 |
| 岛池（峰值，本轮修复后） | 3.1 MB | L3 |

三条直接推论：

1. **射线查询是延迟问题，不是算力问题**：每条射线 27.4 个节点 × 48 B 随机跳转 ≈ 1.3 KB 随机行，绝大多数落 L3（~40-50 cycles）。这就是 2.32 µs/条的主因（85 ns/节点 ≈ 220 cycles/节点，含每次节点 4 条射线的标量 slab 测试）。→ 对策是"减少节点数（网格）+ 减少每节点字节（量化）"，不是换 SIMD。
2. **接触数据是带宽问题**：9146 个接触对 × (624+608) B ≈ 11 MB，`PreSolve`/`Solve` 每迭代都要重读一遍 608 B 的 AoS 状态。→ 对策是 **SoA 拆分 + 每迭代只碰热字段**（法线、点、累积冲量、有效质量），把每接触点的工作集从 56 B 降到 ~24 B。
3. **静态/动态混在一起**让 317 KB 的节点表被 1320 个动态叶子持续搅动（动态叶每步 update/移动），静态部分的局部性也被破坏。→ 对策见 §3.1 分离索引。

### 4.2 访问顺序与布局

- **空间排序**：按 Morton/Z-order 重排静态 box 与 body 的内存顺序（一条赛道一次性排序），使"同一个 8 m 网格里的盒子"在内存里相邻。这不改任何算法，只改索引顺序，但能让 §3.1 的候选列表近似顺序读。
- **接触按岛聚类**：当前岛池是"按岛追加"，已经是好的；再把每岛的接触状态按"点 → 字段"重排为 SoA（`normal[] / ra[] / rb[] / impulse[] / mass[]`），求解内层循环连续读。
- **拓扑排序 / 图着色**：同岛内先把"两两不共享 body"的接触分组（graph coloring），组内可以无冲突地 SIMD 化或并行化。这是 Box2D v3 的 `graph coloring` 思路，收益在"大堆"场景。
- **防伪共享**：`rpcWorkerPool` 的 `m_nextTask` / `m_tasksDone` / `m_seq` / `m_taskCount` 现在相邻，被所有 worker 高频写 → 同一缓存行乒乓。加 `alignas(std::hardware_destructive_interference_size)`（本机实测 64 B）分离。
- **软件预取**：`QueryBatch` 里兄弟节点可以先 `__builtin_prefetch` 把下一层拉进缓存（`node->left/right` 连续存放时收益最大）。注：neuralnet.cpp 全仓 `__builtin_prefetch` **零命中**，这块是纯增量。

### 4.3 ISA / SIMD 的真实收益边界

**实测 1（碰撞核）**：把整个 L1 用 `-march=native -funroll-loops`（Broadwell → AVX2+FMA）重编，`rpc-pileup-tests --cars 24 --spread 0` 三轮均值 **0.356 ms/步（native）vs 0.359 ms/步（基线 SSE2）——无可测差异**。

**实测 2（射线路径）**：`-march=x86-64-v3` 的整机构建（`APEX_NATIVE_ARCH=ON`）在同一台机上把悬挂射线从 **1.97 µs 抬到 2.16 µs（-10%）**，`world.Step` 不变。因此该项在 CMake 里**默认 OFF**（可选项），而不是默认开启。

结论要如实写进决策：**窄相/宽相/求解是分支与访存主导，编译器的自动向量化在这里几乎拿不到东西**。所以：

- `-march=x86-64-v3` 不该默认加：碰撞核无感、射线路径实测更慢（见上），只适合需要向量化数学的分支里局部启用。
- 真正的 SIMD 收益必须来自**手工数据并行**：4 条射线打包成一个批次（§3.2）、同岛内 4 个无冲突接触打包（§4.2 图着色之后）、4 个 AABB 一次求交（Box2D v3 的 `b2_simd` 做法）。
- `std::simd`（C++26）在本机**不可用**（实测：MSVC STL 无 `<simd>`、无 `<experimental/simd>`），所以现阶段用 `<immintrin.h>` 薄封装，接口按 `std::simd` 的形状写，将来切换只改一层。

### 4.4 多线程

- 现有 worker pool 已是"调用者参与 + 原子任务计数 + 自旋"的零分配设计，方向正确。补三件事：自适应阈值（岛太小别并行，neuralnet.cpp 实测 65 536 元素并行反而 0.32×、4 M 元素 7.8×，物理侧对应"岛数/接触数门控"）、缓存行隔离、任务按体量排序（大岛先领，减少尾部）。
- 接触窄相可以按"车"做无冲突并行（每车只写自己的 manifold），但 `AddContact` 会写 body 的边表 → 需要把"接触创建"从"窄相计算"里拆出来（先并行算 manifold，再串行建边）。
- 求解的并行边界是岛（已有），岛内并行要等图着色（§4.2）。

### 4.5 分配器

- `rpcHeap` 是 20 MB 固定块 + first-fit 线性扫描 + 空闲块数组；`rpcPagedAllocator` 是定长页。对物理这种"大小固定、生命周期整齐"的场景，**arena + 尺寸分级池**更合适：每帧的接触/岛工作区从 arena 一次切，帧末整体回收（零分配热路径）。
- 复用 neuralnet.cpp 的经验：池化 + 空闲**龄期**归还（只归还"上次 release 以来未被使用"的整块），避免稳态抖动；并暴露 `PoolStats`（块数/占用/峰值）给 bench。
- 所有热区容器 `alignas(64)`；`static_assert(sizeof(Node) <= 32)`、`static_assert(sizeof(rpcContactState) == 56)` 之类编译期断言挂上，防止后续字段顺手加胖。
- 池扩容（GB 级那种一次性 `resize`）用 **non-temporal store**（`_mm256_stream_*`）避免把 L3 冲干净——本轮修掉的"3.2 GB 预留"就是这种操作，虽然现在已经不需要那么大了。

### 4.6 硬件加速的边界（GPU / 专用指令）

要明确"什么不该上 GPU"：

- **不该**：单车的 4 条悬挂射线、单个岛的求解。它们只有几十到几百个元素，走 GPU 的传输 + 启动开销（~5-20 µs）比 CPU 直接算（2.3 µs/条）还贵，而且延迟会直接进帧时间。
- **可以**：**大规模互不相关的窄相**（碎片/多车同步接触）、**离线构建静态几何索引**（赛道加载时一次）、**批量扫掠/预计算**（例如整条赛道的路面网格 + 表面参数）。
- **专用指令**：`_mm_prefetch`（兄弟节点，§4.2）、`_mm256_*`（4 路打包，§3.2）、`movntdq`（大块初始化）、`popcnt`/`lzcnt`（位集索引，若做稀疏体素）。本机无 AVX-512，按 4-wide 设计即可；不要为 8-wide 牺牲可读性。
- 关于"用 GPU 跑物理"：本项目的物理在**仿真线程**上按 120-480 Hz 固定步进，帧内延迟敏感、数据量小，**不是 GPU 的甜点**。真要上，也是"车-车窄相批处理"这种可离线一帧的子任务，且必须与主循环解耦（异步读回），否则回退比收益大。

### 4.7 线程、亲和性与频率

- 仿真线程与渲染线程共享 L3（40 MB）：`CollisionBus` 的事件、快照三缓冲都是跨线程写读，尽量**只传指针/小结构**，不传大数组（现在已经是这个方向）。
- 物理线程与渲染线程最好绑不同物理核（`SetThreadAffinityMask`），避免超线程兄弟互相抢 L1/L2——本机 16C32T，若两个热线程落在同一物理核的两个 SMT 上，实测会掉 20-40%（可用 `--bench` 做绑核 A/B 验证）。
- 固定步进的成本对**频率缩放**敏感（480 Hz 意味着每 2.08 ms 必须算完一个子步）：笔记本省电档下主频腰斩会让 480 Hz 直接崩掉，因此**游戏侧应有"目标物理率 vs 实测预算"的自适应降级**（例如自动从 480 降到 240，或在堆积时临时降子步），这比在 L1 里抠 10% 更有效。

---

## 5. 现代 C++（C++23/26）落地清单

**本机实测可用性**（clang 23.1.5 + MSVC STL，`-std=c++23`）：

| 特性 | 头 | 本机 | 在物理里的用法 |
|---|---|---|---|
| `std::span` | `<span>` | ✅ | 岛池/查询批次/接触视图（已在用，继续扩展） |
| `std::expected` | `<expected>` | ✅ | 冷路径错误通道（已在用，热路径不检查） |
| `std::mdspan` | `<mdspan>` | ✅ 新增 | 接触状态按 `[contact][point][field]` 建 2D/3D 视图，取代裸指针 + 手算 stride |
| `std::flat_map` | `<flat_map>` | ✅ 新增 | `boxToBody` 反查表（当前 `unordered_map`，每命中一次哈希）→ 有序紧凑数组 + 二分，或直接改成"leaf id → body"数组 |
| `std::inplace_vector` | `<inplace_vector>` | ❌ C++26 | 岛内 ≤8 接触点、≤16 体的小容器（落地后可替换 `std::vector` 成员） |
| `std::simd` | `<simd>` | ❌ C++26 | 4 路射线/4 路 AABB；现阶段用 `immintrin.h` 薄封装 |
| `[[assume]]` | 语言 | ✅ | 热路径指针非空、`count > 0`、索引上界，帮助消分支 |
| `[[likely]]` / `[[unlikely]]` | 语言 | ✅ | 接触命中/未命中、盒对超预算 |
| `constexpr` / `consteval` | 语言 | ✅ | 盒索引表、SAT 轴表、单位轴、量化常数 |
| `static_assert` | 语言 | ✅ | 结构体尺寸/缓存行/对齐契约（见 §4.5） |
| `std::bit_cast` / `std::byteswap` | `<bit>` | ✅ | 量化 BVH 的打包/解包 |
| `deducing this` | 语言 | ✅ | query callback 用静态多态替代虚函数（消除每叶子一次虚调用） |
| `std::to_underlying` / `std::unreachable` | `<utility>` | ✅ | `enum class` 位标志、不可能的 SAT 分支 |
| `std::hardware_*_interference_size` | `<new>` | ✅ = 64 | 防伪共享 padding |
| `#embed` | 语言 | ✅（clang 扩展，C++26 标准；需关 `-Wc23-extensions`） | `CarModelData.h` 现在是 5.3 MB 的 C 数组源码 → `#embed` 直接嵌入二进制，编译时间与源码体积双降 |
| modules | 语言 | ✅ 已在用 | 把 L1 也做成模块，跨 TU 内联热路径（neuralnet 有 `import nn` vs `#include` 的实测记录） |

**写法约定**（与 neuralnet.cpp 的规范对齐）：

- 热路径**零分配**：所有缓冲区成员化/arena 化；`noexcept` + `[[nodiscard]]` 标满。
- 向量化内核**必须收进独立小函数**（neuralnet 实测：带 pragma 的循环放在大函数里会拖垮同函数其它循环，整函数从 374 掉到 95 GFLOPS）。
- 不做"为了现代而现代"：`std::ranges` 在逐元素热循环里不引入；`std::function` 不进热路径。
- 刻意**不用** `-ffast-math`（会破坏 NaN 语义与确定性），用 `-fno-math-errno -fno-trapping-math -funroll-loops`。

### 5.1 具体写法（可直接照抄的形状）

**① 接触状态 SoA + `std::mdspan`**：现在每对接触是一块 608 B 的 AoS（`rpcContactConstraintState`），求解内层每迭代重读整块。改成按"点"拆字段，用 `mdspan` 表达 `[point][field]` 视图，内层循环只碰热字段：

```cpp
// 每岛一片，切自 arena；point 轴连续，field 轴是热/冷两段
using FieldSpan = std::mdspan<float, std::dextents<std::size_t, 2>>;
FieldSpan hot{ arena.data(), points, 6 };   // n.x..n.z, ra, rb, impulse
// 内层：连续 6 个 float 一次缓存行，编译器可直接向量化
for (std::size_t i = 0; i < points; ++i)
    dv = dot(vB + cross(wB, hot[i, 4]) - vA - cross(wA, hot[i, 3]), normal);
```

**② `boxToBody` 从 `unordered_map` 换成稠密数组**（#8）：树节点里本来就存了 `void* userData`，直接改成"叶 id → `BodyHandle`"的 `std::vector<std::uint32_t>`，命中时一次数组读：

```cpp
// 建树期：leafToBody[leafId] = handle;  查询期：
const BodyHandle handle = leafToBody[static_cast<std::size_t>(id)];
```

**③ 静态多态 callback 去虚调用**（#9，C++23 `deducing this`）：

```cpp
struct ClosestHit {
    RayHit result; BodyHandle ignore; const std::uint32_t* leafToBody;
    bool operator()(rpcBox* box, std::int32_t id) const { /* 内联，无 vtable */ }
};
template <typename Cb> void rpcScene::RayCastT(Cb&& cb, rpcRaycastData& ray) const;
```

**④ `[[assume]]` + `[[likely]]` 消灭热分支**：

```cpp
[[assume(count > 0 && count <= 32)]];
if ([[likely]](bodyCount > 0 && contactCount > 0)) { /* 常规路径 */ }
```

**⑤ 编译期契约**（防止后续字段顺手加胖、防伪共享）：

```cpp
static_assert(sizeof(rpcContactState) == 56);
static_assert(sizeof(Node) <= 32, "节点必须落进半条缓存行");
struct alignas(std::hardware_destructive_interference_size) WorkerTaskCounter {
    std::atomic<std::int32_t> nextTask{-1};   // 与 tasksDone 分处不同缓存行
};
```

**⑥ 量化边界的打包/解包**（#1，`std::bit_cast` + `std::byteswap`）：

```cpp
// 相对父节点中心的量化：范围 [-R, R] 映射到 uint16
constexpr float kInvStep = 65535.f / (2.f * kQuantRange);
std::uint16_t q = static_cast<std::uint16_t>((v + kQuantRange) * kInvStep);
float back = static_cast<float>(q) / kInvStep - kQuantRange;   // 解包
```

**⑦ `#embed` 收掉模型数据源码**：`CarModelData.h` 现在是 5.3 MB 的 C 数组字面量（每次全量重编都要重新解析），改成 `constexpr auto kBody = std::define_static_array(#embed "body.bin")` 形状后，编译时间和源码体积双降。

**⑧ L1 模块化**：把 `rush-physics-core` 从"头文件 + 静态库"改成 `export module rush.physics.core;`，热路径（`Query`、`rpcBoxtoBox`、solver 内层）才能稳定跨 TU 内联（现在依赖 ThinLTO，跨模块边界的内联不稳定）。

---

## 6. 复用 neuralnet.cpp 的优化范式

同一套工程哲学可以直接映射，且大多带量化收益：

| neuralnet.cpp 的手法 | 物理侧的对应物 | 预期 |
|---|---|---|
| 寄存器分块微内核（4×8，`NI` 作模板参数；运行时传参慢 1.7×） | 4 条射线/4 个接触点的打包内核，宽度作模板参数 | 遍历/求解内核 1.5~2× |
| 自适应并行阈值（<512 K 等效元素串行，实测 65 K 并行 0.32×、4 M 7.8×） | 岛/接触数门控：小岛串行、大堆并行 | 去掉并行抖动 |
| 零分配 latch 线程池（N 块 1 锁 0 分配 + 调用者参与 + 短自旋） | `rpcWorkerPool` 补自适应门控 + 防伪共享 | 每步省 µs 级 |
| 缓存分块（`BLOCK_SIZE=64`，32 KB 装 L1）+ 栈上 `std::array` 不初始化 | 接触/岛工作区按 L1 分块；热字段 SoA | 带宽 -30~50% |
| 图 IR 融合（DCE/常量折叠/CSE/寄存器分配；Adam 每步 266→66.5 kernel） | **对"接触/约束图"做同样的化简**：合并共面盒、删掉被完全包含的盒、去掉零面积/零密度形状；岛内共享子表达式提取 | 静态几何可减 20~40% |
| 流式 fold 不物化（seq² 从 3.2 GB → 0.2 GB） | 不物化 per-island 拷贝：直接在池上原地求解（本轮已把"岛 × 全场峰值"的物化去掉） | 已经拿到 3.2 GB→3.1 MB |
| 池化分配 + 空闲龄期整块归还 + `PoolStats` | 接触/形状池 + `rpcScene::Stats`（已加） | 稳态零分配 |
| 零拷贝 `shared_ptr` 张量 | 快照/接触事件不做全量拷贝（`CollisionBus` 已经是引用传递方向） | 每帧省一次全量拷贝 |
| 向量化隔离（带 pragma 的函数必须独立） | SAT/裁剪内核独立成 `rpcCollideOnePair()` 等小函数 | 避免拖垮邻居循环 |
| 精度只降存储（f16 存 + f32 算，显存 -69%、耗时 +5%） | 形状参数/质量用 f16 存？——**不建议**：物理积分对精度敏感，只考虑 BVH 量化（16-bit 整数） | 见 §3.2 |
| 每线程局部累加避免原子竞争 | 求解统计/接触计数按岛局部累加，末尾合并 | 去掉热路径原子 |
| ABAB 同窗交错 + warmup/多轮取 best | 见 §8 度量协议 | 结论可信度 |

---

## 7. 路线图

### P0（≤1 天，低风险，先拿"免费"的）

1. ~~`-march=x86-64-v3`~~：实测无收益且射线路径变慢，改为可选（默认 OFF，§4.3）。
2. `boxToBody` 从 `unordered_map` 换成"leaf id → body handle"的稠密数组；`rpcBox::Raycast`/`ComputeAABB` 的 `local == identity` 特化。
3. `rpcWorkerPool` 计数器 64 B 隔离；`static_assert` 尺寸契约。
4. `[[assume]]` / `[[likely]]` 铺到 `Query` / `rpcBoxtoBox` / 求解内层。
5. 把 `rpcScene::Stats` 接到 `--bench` 输出（现在只有 pileup 用例读它）。

**验收**：`--drive-test` 三后端 0 failures；`--bench 120 480 rpc` 的 world ≤ 1.10 ms，vehicle ≤ 1.40 ms。

### P1（3~5 天，结构性，收益最大）

6. §3.1 静态几何 XZ 网格。
7. §3.2 BVH 量化节点（先不上 4 路，单步验收）。
8. §3.3 每车单代理宽相。
9. §3.4 持久 manifold + 接触对象池。

**验收**：每子步 ≤ 1.4 ms（120 车/480 Hz，即每帧 ≤ 11 ms）；`rpc-pileup` 的堆积最差步不劣于 76 ms；`--drive-test` 数值断言仅允许在手感容差内变化。

### P2（1~2 周，求解器与并行）

10. §3.5 TGS 子步 + 局部锚点。
11. §3.6 soft constraint + relaxation（保留 PGS 作 A/B 开关）。
12. §3.7 岛/接触图分层 + 自适应迭代 + 图着色。

**验收**：480 Hz/120 车每帧 ≤ 8 ms（60 fps 有余量）；三后端 drive-test 全绿；`physics-upgrade.md` §8 手感项全项重跑。

---

## 8. 验证与度量协议

- **ABAB 交错**：同一进程内交替跑 A/B 两版本各 3 轮（避免温漂），取 best；单点对比一律不认。本轮实测到的最强佐证就是 `rpc-ray-index` 在**同一进程**里背靠背比两种策略（tree 0.61 vs indexed 0.42 µs/ray）——跨进程的绝对毫秒数在本机受后台负载影响可达 ±20%，不能作为结论依据。
- **warmup + 多轮**：warmup 20 帧、测 3×50 帧取 best（沿用 neuralnet 的 `run_ops.ps1` 口径）。
- **报均值 + 1% low**：`--bench` 已输出帧/步的 1% low；只看均值会漏掉堆积尖峰。
- **四类场景固定**：空旷巡航（无接触）、车群混战（稀疏接触）、堆积（密集接触）、对撞（高速单次）。
- **硬件计数**：需要时用 WPR/ETW 采 IPC、L1/L2/LLC miss、branch miss；判断"是延迟还是带宽"用 miss per instruction。
- **回归门槛**：`--drive-test` 三后端 0 failures（确定性档 `JobSystem::Configure(0)`）；`rpc-pileup` 不退步；`--bench` 同场景不劣化 >5%。
- **确定性**：任何换算法（尤其 §3.4/§3.5）都要先用"逐位一致"验证路径正确，再允许数值差异；`drive-test` 的断言是可复现基线。

---

## 9. 风险与取舍

| 风险 | 影响 | 缓解 |
|---|---|---|
| 换求解器改变手感 | 弯中/撞墙行为变化 | 保留 `solver` 开关，双路径 drive-test + 手感对照 |
| 量化 BVH 误差 | 穿透/漏检 1~2 cm | 保守膨胀 + `rpc-pileup` 与 `drive-test` 逐位对照 |
| 静态网格与 BVH 双份唤醒逻辑 | 最近命中选错 | 两条索引都返回最近命中，取 min；单测覆盖边界 cell |
| 接触预算（12/对）丢细节 | 极端堆叠略软 | 上限可配；撞车场景以"不穿模"为验收 |
| TGS 与射线悬挂耦合 | 悬挂仍按子步积分，需与接触子步对齐 | 外步/内步只作用于接触求解，悬挂保持每外步一次，先小步验证 |
| 过度并行 | 小场景抖动 | 自适应阈值 + 门控（§4.4） |
| 现代 C++ 特性不可用（`std::simd`/`inplace_vector`） | 依赖工具链 | 实测表（§5）为准；薄封装 + `__has_include` 守卫 |

---

## 10. 附录

### 10.1 符号与常数（本机）

| 符号 | 值 | 来源 |
|---|---|---|
| 缓存行 | 64 B | `std::hardware_destructive_interference_size` |
| L2 / L3 | 256 KB per core / 40 MB | CPU（Broadwell-EP） |
| 树节点 | 48 B | 推导 |
| 窄相 | 0.19 µs/接触对 | pileup 实测 |
| 求解 | ≈15 ns/接触点/迭代 | pileup 实测 |
| 宽相 | ≈1.5 µs/车/子步（11 盒） | bench 实测 |
| 射线 | 2.32 µs/条、27.4 节点/条 | bench 探针实测 |
| 480 Hz 每帧子步 | 8 | `physics.hz / 60` |

### 10.2 参考文献

1. Erin Catto, *Solver2D*（2024）：PGS / PGS-NGS / TGS / soft constraint / relaxation 的对比与公式，含"子步不重算接触点、用局部坐标锚点重投影"的做法。
2. Miles Macklin et al., *Small Steps in Physics Simulation*（SCA 2019）：子步优于迭代的依据。
3. Erin Catto, *Iterative Dynamics with Temporal Coherence*（2005）：Sequential Impulses 原始表述。
4. Bepu Physics v2：把 soft constraint 参数化简为与质量无关的 `biasRate / massCoeff / impulseCoeff`。
5. Jolt Physics：静态/动态分层的宽相（分块四叉树 + broadphase layer），可作为"静态几何专用索引"的对照方案。
6. Box2D v3：动态树查询 SIMD 化、graph coloring、接触回收。
7. Embree（Intel）：4-wide / 8-wide BVH 与 16-bit 量化节点（本机无 AVX-512，按 4-wide 设计）。
8. Randy Gaul, qu3e（本内核的源头）：本文所有"现状"行为都以本仓库 fork 后的代码为准。
