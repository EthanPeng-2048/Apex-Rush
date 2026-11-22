# Rush Engine 性能对比报告

改造前（单线程同步主循环）vs 改造后（Rush Engine：事件总线 + JobSystem 并行 AI + 仿真线程/快照）。

## 测试方法

- 命令：`build\apex-rush.exe --bench 100`
- 负载：100 车、autopilot、固定辅助、10 秒比赛（600 帧 @60fps，4800 物理子步 @240Hz）
- 基线：提交 `718eeff`（性能基准落地时的同步主循环）
- 异步：提交 `04a185e`（P5 完成后；AI 并行 + 事件总线 + 可选仿真线程）
- 指标为**串行可比口径**的分段计时；异步 AI 开启后 `ai` 段为并行墙钟

## qu3e（默认物理后端）· 100 车

| 指标 | 基线（同步） | 异步（JobSystem AI 并行） | 提升 |
|---|---:|---:|---:|
| **update / 帧** | **31.8 ms** | **14.4 ms** | **≈2.2×** |
| AI / 帧 | 19.5 ms | 2.5 ms | **≈7.9×** |
| vehicle / 子步 | 0.779 ms | 0.755 ms | 持平（world 串行） |
| world.Step / 子步 | 0.705 ms | 0.684 ms | 持平（引擎内部） |
| race / 帧 | 0.214 ms | 0.202 ms | 持平 |
| **墙钟（600 帧）** | **19.1 s** | **8.6 s** | **≈2.2×** |
| **吞吐** | 251 steps/s | 556 steps/s | **≈2.2×** |
| 帧率（墙钟） | 31.4 fps | 69.5 fps | **≈2.2×** |

基线时 AI 占 update 的 **61%**——并行化后 AI 只剩 17%，瓶颈回落到 `world.Step`（qu3e 内部；RPC 深度定制仅限碰撞，不含车体）。

## nudge（备选后端）· 100 车

| 指标 | 基线 | 异步 | 说明 |
|---|---:|---:|---|
| update / 帧 | 681 ms | 629 ms | AI 并行有感，但 vehicle 占 85% |
| AI / 帧 | 19.7 ms | 2.5 ms | ≈7.9×（同 qu3e） |
| vehicle / 子步 | 69.9 ms | 67.2 ms | **仍串行**（IPhysicsWorld 非线程安全） |
| 墙钟 | 409 s | 377 s | 提升有限 |

nudge 的主成本在逐车 `vehicle.Update`（射线悬挂 + 轮胎）。并行车辆子步只能在 **L2 游戏/引擎编排**做（分片或与 Step 交错）；**不**下沉进物理库，也无「引擎内置车辆共仿真」规划。

## 架构层面的性能资产（本轮交付）

1. **`rush.engine.events`**：物理接触只 `Publish`，游戏在独立 stage `Drain`——生产者栈零回调，模块间无阻塞等待。
2. **`rush.engine.job`**：exstl 线程池封装；`Configure(0)` 串行降级保证 drive-test 确定性。
3. **`rush.engine.runtime`**：仿真线程 + `SnapshotTripleBuffer`（三缓冲 shared_ptr：Publish O(1) 指针交换、CopyFront 只付引用计数）；实机比赛阶段 sim 与主线程渲染/输入重叠，菜单仍单线程。
4. **C++20 named modules**：`import rush.engine.{core,physics,render,events,job,runtime}`。

## 复现

```text
# 基线数据
bench-baseline-100.txt   # 718eeff 时代

# 异步数据
bench-async-100.txt      # P4/P5 后

cmake --preset release && cmake --build --preset release
build\apex-rush.exe --bench 100
build\apex-rush.exe --drive-test   # 双后端 0 failures
```

## 结论

- **百车 qu3e 同步主循环 → 全异步引擎：update 31.8→14.4 ms，墙钟 19.1→8.6 s（≈2.2×），AI 段 7.9×。**
- 余下大头是 `world.Step`（qu3e/rpc 碰撞）与串行 `vehicle.Update`（L2 车体，永不进 L1）——碰撞并行只动 RPC/qu3e 内核；vehicle 并行只动 L2。
- 实机路径额外获得 **仿真线程与渲染重叠**（headless bench 不度量该项）。
