# 车辆组件化重构（现行）

状态：**已实施**（2026，native 全后端 drive-test 0 failures、bench 无退化；WASM 未重跑）。本文件是车辆层重构的现行真源，行为以代码与本文档为准。
关联：[architecture.md](architecture.md)（系统分层）· [rpc-refactor.md](rpc-refactor.md)（物理后端重构方法论参考）。

---

## 0. 动机与目标

现状 `Vehicle` 是一个**上帝类**（`rush-engine/src/physics/vehicle_impl.cpp`，880 行）：物理、动力总成、辅助、气动、损伤、渲染全塞一个类一个文件，`Update()` 是超长串行过程。多个领域耦合，无法独立替换、无法面向"改车"按部件调参。

本次重构目标（用户裁定）：

1. **把"车"拆成明确区分的零部件**：引擎、变速箱、悬架（前/后）、空力套件（前/后/侧/底盘）、驾驶辅助（TCS/ESC/ABS）。
2. **输入处理不属于驾驶辅助**：转向平滑（键盘 easing）、防 spin/打滑、自动反打等是"输入处理"层，独立于车部件。
3. 为将来接入**独立的变速箱系统、引擎系统**（可替换式）与**改车功能**（按部件调参/改装）铺路。
4. 验收**不要求物理结果逐位一致**——本次是设计变更，允许行为在小幅范围内演进（见 §6 验收）。

**铁律（不破坏）**：车永不直接碰 L1 碰撞内核，只经 `IPhysicsWorld` 下发 `ApplyForce/Raycast/GetTransform`。重构只动 L2 `Vehicle` 及 L3 对 `Vehicle` 的调用点，不动三套后端与整体分层。

---

## 1. 组件划分

车从"单类"变为**装配体（aggregate）**。每个组件遵循统一形状：**Params（可改/可调参）+ State（运行时输出）+ Step/Update（单子步推进，产出作用力或作动请求）**。

```
                        ┌─────────────────────────────────────────┐
                        │              Vehicle（编排器）            │
                        │  持有装配 + 底盘刚体 + 轮胎模型 + 渲染     │
                        │  Update(): 读底盘 → 各组件协作 → 汇总结力 │
                        └───────────┬─────────────────────────────┘
                    ┌───────────────┼───────────────────────────┐
                    │               │                           │
      ┌─────────────▼─────┐  ┌─────▼────────────┐  ┌──────────▼──────────┐
      │  DriverControls     │  │  DriverAssistance │  │   Powertrain        │
      │  （输入处理，非部件） │  │  TCS / ESC / ABS   │  │  Engine + Gearbox   │
      │  转向平滑/反打/防spin │  │  作动调制，非输入   │  │  扭矩/齿比/挡位/刹车 │
      └─────────────────────┘  └──────────────────┘  └─────────────────────┘
                    │                                   │
                    └───────────────┬───────────────────┘
                                    ▼
                    ┌────────────────────────────────────┐
                    │  Suspension（悬架，前/后）            │
                    │  4 轮弹簧阻尼 → 载荷/接触（需 raycast）│
                    └────────────────────────────────────┘
                                    │
                                    ▼
                    ┌────────────────────────────────────┐
                    │  AeroKit（空力套件）                 │
                    │  前翼/尾翼/侧箱/底盘 + 部件健康度      │
                    │  损伤→面积、DRS、X/Z、俯仰 → 气动力    │
                    └────────────────────────────────────┘
```

依赖方向（单向、无环）：

```
DriverControls → CarIntent
DriverAssistance → CarIntent + 底盘状态
Engine → Transmission → 驱动/刹车分配（后轴）
Suspension → IPhysicsWorld(Raycast) → 每轮载荷/接触
TireModel（每轮） ← Suspension 载荷 + Engine/Transmission 驱动
AeroKit → 底盘状态 + CarIntent(DRS/翼挡) → 气动力
Vehicle（编排器）汇总结力 → IPhysicsWorld(ApplyForce)
```

---

## 2. 组件接口

### 2.1 统一形状

每个组件一个 `Params`（装配时的可调参数，即"改车"的着力点）+ 一个 `State`（运行时输出）+ 一个 `Step()`。组件**不直接摸 IPhysicsWorld**，除非其本职就是几何交互（悬架做 raycast、AeroKit 读位姿）——一律通过 Vehicle 传入的只读上下文。

### 2.2 CarContext（每子步共享上下文）

Vehicle 每子步构造一个只读上下文，各组件从它读底盘状态，写自己的 State，最后 Vehicle 汇总力。

```cpp
struct CarContext {
    const IPhysicsWorld* world;      // 悬架 raycast / 车体查询用
    Transform chassis;               // 底盘位姿
    Vec3 velocity, angularVelocity;
    Vec3 forward, right, up;         // 车体轴（+Z 前 / +X 几何左 / +Y 上）
    float speed, forwardSpeed;
    float dt;
};

struct CarActuation {                // 各部件产出的"作动请求"，Vehicle 汇总成力
    Vec3 aeroForce;                  // 气动（作用点分前后轴，见 AeroKit）
    float drivePerWheel[4];          // 每轮驱动力 [N]
    float brakePerWheel[4];          // 每轮制动力 [N]
    Vec3 escYawTorque;               // ESC 横摆阻尼力矩
    Vec3 escLateralForce;            // ESC 侧向阻尼力
    // 悬架/轮胎力由 Suspension+Tire 直接 ApplyForce 或汇总进 here
};
```

> 设计取舍：把"力"从"作动请求"分离。Suspension 与 Tire 直接产出力；Powertrain 产出驱动/刹车请求；AeroKit 产出气动力。Vehicle 统一 ApplyForce。避免每个组件各自乱调 world。

### 2.3 DriverControls（输入处理——**非车部件**）

负责把**原始输入**加工成**驾驶员意图**。明确不含在驾驶辅助内。

```cpp
class DriverControls {
public:
    struct Params {                  // 平滑/反打强度等（可选改车项）
        float steerTauUnwind = 0.11f;
        float steerTauWind   = 0.10f;   // + 0.0032*speed 封顶 0.34
        float counterSteerMax = 0.85f;  // Full 挡反打上限
        ...
    };
    struct State { float steerAngle = 0.f; float throttleSmooth = 0.f; float brakeSmooth = 0.f; };
    struct Output {                  // 加工后的"驾驶员意图"（含 DRS/翼挡/ERS 请求）
        float throttle, brake, steer;
        bool handbrake, reverse;
        bool drsRequest, drsZone, wingToggle, overtake;
    };

    // 输入处理：键盘 easing + 反打/防 spin 辅助（根据当前 steerAngle 与过转门限）
    Output Process(const CarInput& raw, const CarState& cs, const VehicleParams& p, float dt);
    // 供 AI/测试直接注入已加工意图，或跳过此层
    void Reset();
};
```

**为什么独立**：用户明确要求"驾驶辅助不含输入处理"。反打/自动反打/防 spin 是针对驾驶员输入叠加修正，逻辑上位于"输入→意图"边界，未来可整体搬到 L3 或替换为手柄线性映射。

### 2.4 DriverAssistance（驾驶辅助：TCS / ESC / ABS）

操作对象是**车物理状态**与**作动请求**，不是输入。

```cpp
class DriverAssistance {
public:
    struct Params {
        AssistLevel tcLevel; escLevel; absLevel;
        // 阈值（TC 滑移 cut、ABS 释放、ESC 门限/增益）——可改车/可分级
    };
    struct State { float escStrength = 0.f; };

    // 读 CarState（上一子步车轮滑移、steerAngle、beta/yaw 过转门限）
    // 写 CarActuation：TC 切油门(进 throttle)、ESC 横摆力矩+侧向阻尼+尾刹保留、ABS 逐轮释放
    void Step(const CarState& cs, const CarActuation& actuation,
              CarActuation& out, float dt);
};
```

> TCS 作用于油门（切驱动），ABS 作用于逐轮制动，ESC 产出横摆阻尼/侧向阻尼/尾刹保留——三者都是对"作动请求"的调制，均非输入处理。原 `UpdateAssists` 中"转向小修 + 反打"属 DriverControls（输入处理）。

### 2.5 Engine（引擎）

```cpp
class Engine {
public:
    struct Params {                 // 扭矩曲线——改车/换引擎的着力点
        float maxTorque; idleRpm; peakRpm; redlineRpm;
        float shiftUpRpm; shiftDownRpm; shiftTime;
        float engineBrakeTorque;
        std::array<float,8> gearRatios; float finalDrive;
    };
    struct State { float rpm = 0.f; int gear = 1; float shiftTimer = 0.f; };

    // rpm 由车速×齿比决定（锁止到引擎）；扭矩曲线 → 扭矩因子
    float TorqueFactor() const;
    void SetRpmFromWheel(float wheelRps, const Params& p);   // 由 Transmission 驱动
    void UpdateShift(const State&, Params&);                 // 挡位切换/动力中断
};
```

### 2.6 Transmission（变速箱）

```cpp
class Transmission {
public:
    struct State { int gear = 1; float shiftTimer = 0.f; };  // 0=倒挡
    struct Output {
        float drivePerWheel[4];     // 后轴 50:50，LSD 预留
        float brakePerWheel[4];     // 60:40 前后，ESC 尾刹保留
        float wheelRps;             // 反馈给 Engine 算 rpm
    };

    // 输入：Engine 扭矩 → 齿比×主减速 → 驱动轮力；刹车请求 → 分配
    Output Compute(const Engine::State&, float throttle, float brake, ...);
};
```

> **为"独立变速箱/引擎"铺路**：Engine 只管扭矩曲线与 rpm 语义；Transmission 只管齿比/挡位/分配。二者通过极小的数据契约（rpm ↔ wheelRps、扭矩 → 轮上力）解耦，将来可整体替换成电驱/无级/序列式变速箱。

### 2.7 Suspension（悬架，前/后）

```cpp
class Suspension {
public:
    struct Params {
        // 前轴 / 后轴 独立
        float springKf, damperCf, springKr, damperCr;
        float restLength; maxTravel; wheelRadius;
        float camberStatic; camberGain;   // 简化的外倾
    };
    struct CornerState {  // 每轮
        bool grounded; float compression; float load; float contactY;
        Vec3 contactPoint, contactNormal;
        Vec3 contactVelocity;
    };
    struct State { CornerState corner[4]; };

    // 逐轮 raycast：压缩/压缩率 → 弹簧+阻尼力 → 每轮载荷；返回法向力（悬架施力）
    void Step(const CarContext& ctx, const Params& p,
              State& st, Vec3 normalForce[4], float dt);
};
```

> 前/后轴参数分离（springKf/springKr…）——满足"悬架（前/后）"的明确区分。外倾/防倾杆仍是简化常数（真实失真项），重构时保留现状、不扩物理深度。

### 2.8 AeroKit（空力套件：前翼 / 尾翼 / 侧箱 / 底盘）

```cpp
class AeroKit {
public:
    struct Params {                 // 每部件可改：基准面积/系数、前后分配、俯仰敏感
        float dragArea; downforceArea; frontDownforceShare;
        float aeroPitchBalance; centerOfPressureOffset; airDensity;
    };
    struct PartHealth { float frontWing=1, rearWing=1, sidepods=1, floor=1; };
    struct State {
        float frontDownforce, rearDownforce, dragForce;
        PartHealth health;
    };
    struct Output { Vec3 frontPoint, rearPoint; float frontDF, rearDF, drag; };

    // 吸收原游戏层 WriteVehicleAero 的规则：损伤→面积、DRS、X/Z 翼、俯仰平衡
    void ApplyDamage(CarPart part, float amount);     // 改车/碰撞写健康度
    Output Compute(const CarState& cs, const CarIntent& intent,
                   const Params& p, State& st);
};
```

> **把 `WriteVehicleAero`（Game.cpp:57）整体下沉进 AeroKit**：游戏层不再每子步算气动面积，只调 `ApplyDamage`。`partHealth` 从 `VehicleState` 移入 `AeroKit::State`（气动部件由空力套件拥有）。DRS/X-Z 的激活门控仍由游戏（赛季规则）经 intent 传入，AeroKit 只做面积→力。

### 2.9 Vehicle（编排器，瘦身后的本体）

```cpp
class Vehicle {
    // 装配
    Engine engine_; Transmission gearbox_; Suspension susp_;
    AeroKit aero_; DriverAssistance assists_;
    std::array<TireModel,4> tires_;
    BodyHandle chassis_;

    void Update(const VehicleInput& raw, float dt);   // 编排各组件
    void ProcessContacts(...);                        // 碰撞 → aero_.ApplyDamage
    void Reset(const Transform&);                     // 重置全部组件 State
    // 只读：State() / Params() / Chassis()（快照、AI、渲染、音频仍可用）
};
```

编排顺序（每子步）：

```
Update(raw, dt):
  1. 读底盘位姿/速度 → CarContext
  2. DriverControls.Process(raw, …) → CarIntent      // 输入处理（平滑/反打）
  3. Suspension.Step → 每轮载荷/接触
  4. Transmission.Compute + Engine rpm → 驱动/刹车分配
  5. DriverAssistance.Step → 调制驱动/刹车 + ESC 力矩    // TCS/ESC/ABS
  6. 每轮 TireModel.Compute → 轮胎力
  7. AeroKit.Compute → 气动力
  8. 汇总 悬架法向 + 轮胎 + 气动 + 阻力 + ESC 力矩 → ApplyForce
  9. 收集 VehicleState
```

---

## 3. 数据归属变化

| 数据 | 现状 | 重构后 |
|------|------|--------|
| `partHealth`（前/尾/侧/底盘） | `VehicleState` | **`AeroKit::State::health`** |
| 气动面积计算（损伤/DRS/XZ/俯仰） | 游戏层 `WriteVehicleAero` | **`AeroKit::Compute`** |
| 引擎扭矩/rpm/挡位 | `Vehicle::UpdatePowertrain` | `Engine` + `Transmission` |
| 悬架弹簧阻尼/载荷 | `Vehicle::UpdateWheels`（前半） | `Suspension` |
| TCS/ESC/ABS | `Vehicle::UpdateAssists` | `DriverAssistance` |
| 转向平滑/反打/防spin | `Vehicle::Update`+`UpdateAssists` | `DriverControls`（非部件） |
| 轮胎力 | `TireModel` | **保持不变** |
| 渲染 | `Vehicle::Render/RenderPose` | **保留在 Vehicle**（快照友好） |

`VehicleInput` 拆成两份：
- **`CarInput`**（驾驶员意图：throttle/brake/steer/handbrake/reverse/drs/翼/ERS）——保留在 L2 接口，AI/游戏写入。
- **气动面积**（`dragArea/frontDownforceArea/rearDownforceArea`）**从输入中删除**——改由 `AeroKit` 拥有。删掉 `Vehicle::Update` 里的 `gameAero` 分支与俯仰 fallback（单测专用逻辑），单测改为直接驱动 `AeroKit`。

---

## 4. 死代码与遗留

重构顺手清理（不改行为面）：
- `hardpointY`（`vehicle_impl.cpp:237`）计算未使用 → 删除。
- 引擎内 aero fallback 分支（`vehicle_impl.cpp:406-421`）→ 移除，面积归属 AeroKit。
- 重复的俯仰平衡计算（游戏层 + 引擎 fallback）→ 收敛到 AeroKit 一处。

---

## 5. 落地步骤（对应 Task）

| 步骤 | 内容 |
|------|------|
| T2 | 在 `physics.cppm` 定义组件接口（Params/State/Step 骨架） |
| T3 | 实现各组件（先 DriverControls / AeroKit / Engine / Transmission / Suspension / DriverAssistance） |
| T4 | `Vehicle::Update` 改编排器；AeroKit 接管 WriteVehicleAero；清理 fallback 与死代码 |
| T5 | 更新 L3：`SpawnRacers` 改喂组件 Params、`StepPhysics` 删 WriteVehicleAero、AI/快照/渲染/音频适配 |
| T6 | 构建 + drive-test 全后端 + WASM 冒烟回归 |

---

## 6. 验收

- 三物理后端 `--drive-test` 全部通过（零 failures）。
- `--bench` 百车分段计时可跑通（性能不退化量级内）。
- WASM 冒烟可启动。
- `Vehicle` 不再直接 include L1；组件经 `IPhysicsWorld`/CarContext 与车解耦。
- `WriteVehicleAero` 从 Game.cpp 删除；气动面积只由 AeroKit 产出。
- 验收**不要求**物理量逐位一致：行为允许在小幅范围内演进（如挡位时序、气动面积合并次序），但**手感方向不变**（仍可稳定驾驶、DRS/翼挡/损伤生效、AI 不上墙）。
