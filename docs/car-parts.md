# 车辆功能 = 配件 + 能力位（现行）

状态：**规范**。取代"每个功能一个 `VehicleParams` bool"的旧做法（`drsEnabled` / `activeAeroEnabled` / `overtakeEnabled` 已删除）。
关联：[architecture.md](architecture.md) · [car-refactor.md](car-refactor.md) · [ui.md](ui.md)

---

## 1. 为什么换

旧做法的三个问题，都是同一个病：

1. 开关散落在 `VehicleParams` 上，**每加一个功能就多一个 bool**，谁都要记得查；
2. 查的人不一致：`Vehicle::Update` 查了，HUD 没查（于是 classic 赛季照样画一条 ERS 电量条），AI 没查，联机规则又是另一套 `seasonBits`；
3. "没有这个功能"和"有这个功能但没触发"在代码里长得一样，所以回充这种**无输入也生效**的分支特别容易漏门（ERS 在 classic 赛季刹车照样充电就是这么来的）。

能力位把这三件事合成一件：**车上装没装 = 一个位**。装没装是车的构建属性，不是运行时状态。

## 2. 契约

```cpp
enum class VehicleCapability : std::uint8_t {
    None = 0,
    Drs = 1u << 0,          // 直道尾翼开启（classic 规则）
    ActiveAero = 1u << 1,   // 2026 X/Z 翼型
    Ers = 1u << 2,          // 混合能量存储：制动回充 + 部署
    Overtake = 1u << 3,     // 2026 超车部署档
};
constexpr bool HasCapability(std::uint8_t mask, VehicleCapability c);

struct VehicleParams {
    std::uint8_t capabilities = CapabilityBit(VehicleCapability::Drs);
};
```

**语义（三条，缺一不可）**

| 情况 | 必须的行为 |
|------|------------|
| 位为 0 | 该子系统**完全不存在**：不产力、不产生状态变化、不发布遥测（`ersCharge` 恒 0、`ersDeploying` 恒 false） |
| 位为 1 | 子系统可以按自己的规则运行，但仍需自身的触发条件（DRS 区、油门、电量…） |
| 读的人 | **所有**消费方读同一个位：物理、HUD/UI、AI、联机规则。禁止在别处再写一套赛季判断 |

**不在 L2 读赛季/规则全局**：L2 只认 `capabilities`。赛季 → 能力位的映射只有一处——L3 的 `SpawnRacersInto`（`src/game/Game.cpp`），它把 `[season]` 门变成掩码写进 `VehicleParams`。这样"这条规则谁说了算"永远只有一个答案。

## 3. 装配

一个功能要成为配件，必须满足组件规范（见 [car-refactor.md](car-refactor.md)）：**Params + State + Step**。

- `Params`：可调参数，和别的部件零耦合。
- `State`：本子步的输出（读给 HUD / 联机 / 回滚）。
- `Step`：只做力学，不读游戏规则。

已按此拆出的部件：Engine · Transmission · Differential · Suspension · TireModel · AeroKit · DriverAssistance · DriverControls · **ErsUnit**。

**ErsUnit 是这次重构的核心收益**：ERS 原来寄生在 `Engine::UpdateErc` 里（引擎既管内燃机又管电池），现在是独立部件，`Engine` 只从 `StepInput.ersPower` 收到一路功率。于是"没装 = 不充电"变成了结构上的必然，而不是一个 if。

## 4. 联机与回滚

- 能力位**不单独上网**：它由房规（赛季位）在两端各自推导，房规随 `Rules` 下发，客户端必须用**主机**的房规建车（用本地 config 建车会让两端跑不同物理）。
- 部件状态必须进 `VehicleSnapshotState`：回滚逐位相等（存档 → 若干 tick → 恢复 → 重跑）是联机预测的门禁，加部件就要同步加 `SaveState/RestoreState`。
- `VehicleParams` 布局变化会改变 `BuildSignature()`（`LanGame.cpp` 对 `sizeof(VehicleParams)` 做 FNV），两端必须是同一次构建——已有 "GAME BUILD MISMATCH" 拒绝兜底。

## 5. 加一个新功能（照抄即可）

1. `VehicleCapability` 加一个位。
2. 若能独立成部件：新建 `XxxUnit`（Params/State/Step），`Vehicle::Spawn/Reset/Update` 各加一处编排，`VehicleSnapshotState` 加成员。
3. 在 `SpawnRacersInto` 里把对应的赛季门映射成位。
4. HUD/UI/帮助行改读 `HasCapability(params.capabilities, …)`，不要读 `seasonXxx_`。
5. 门禁：`--drive-test`（装了的车行为不变）+ 一条"没装 = 无状态变化"的断言。

## 6. 明确不做

- **运行时挂载/卸载部件**：能力位是构建属性，比赛内不变；想换配置就重开一场。
- **字符串配表/外部数据驱动的部件目录**：编译期确定的装配就够，加数据驱动是过度设计。
- **AI 与玩家的部件差异**：AI 用同一套装配，只是装配掩码可能不同（例如低难度不给 AI ERS）。
