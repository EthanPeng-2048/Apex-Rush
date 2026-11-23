# 车辆物理升级设计（差速器 · 悬挂几何/防倾杆 · 气动侧向力 · 部件碰撞盒）

状态：**T1–T5 全部实施（2026-11-23，三后端 drive-test 0 failures）；§10 性能优化执行中**。
关联：[car-refactor.md](car-refactor.md)（车辆组件化，已实施）· [engine-integration.md](engine-integration.md)（引擎/传动，已实施）· [architecture.md](architecture.md)（系统分层）· [rpc-refactor.md](rpc-refactor.md)（RPC 后端）。

---

## 0. 背景与范围

### 0.1 来源

对现行车辆物理的一次审查（基于 `rush-engine/src/physics/vehicle_impl.cpp` 与 `modules/physics.cppm` 已提交代码 `16b151b`/`ccabf34`）列出 8 处简化/缺失。审查结论给出的实施优先级：

1. **差速器**（后轴 50:50 平分，无扭矩偏置）
2. **横向稳定杆 + 悬挂几何**（无主销后倾/拖距/束角/侧倾几何，外倾是常数）
3. **气动侧向力/横摆力矩**（只有阻力和下压力；下压力沿世界竖直而非车身轴；无 COP 高度）
4. **部件独立碰撞盒**（损伤是"接触点坐标分区"，不是真实部件碰撞体）

> 审查时以为差速器已完成；核对 git 历史/stash/分支与工作区后确认**未实现**（`Transmission::Compute` 仍 `driveForce * 0.5f`，`vehicle_impl.cpp:309`），经用户确认纳入本次设计。

### 0.2 设计原则（延续既有铁律）

- **分层不动**：L2 `Vehicle` 永不直接碰 L1 碰撞内核，只经 `IPhysicsWorld`；本次对 L1 的扩展（§4）走接口层，三后端适配。
- **组件化延续**：新物理全部按 `Params + State + Step` 形状落进现有组件（Differential 新组件、Suspension/AeroKit 扩展），改车/换部件只动 Params。
- **三后端兼容**：qu3e（当前默认）与 rpc 原生支持新能力；nudge（Unity 移植，单盒无标签）走**兼容回退路径**，`--drive-test` 三后端必须 0 failures。
- **确定性**：所有新随机/时序敏感逻辑保持无随机或种子化；`JobSystem::Configure(0)` 下 drive-test 可复现。
- **数值预算**：单子步只增加标量/少量矢量运算，bench 百车分段不劣化（量级内）。

### 0.3 明确不做（本期）

| 项 | 原因 |
|---|---|
| 纵向力 Magic Formula | 现有线性 kStiff 饱和 + 隐式阻尼稳定且够用；列为远期（§9） |
| 轮胎温度全模型 | 有意为之的二级修正（审查认可）；维持单参数 |
| 簧上/簧下质量、多刚体悬架 | 对 60FPS 竞速目标无益；列为远期 |
| 翼片断裂（多刚体+约束） | 工程量最大；本期用"复合刚体多盒"近似 |
| 完整悬架运动学（瞬时中心/双叉臂求解） | 用等效参数模型替代（§2），精度收益/成本不划算 |
| CCD 连续碰撞检测 | qu3e 无 CCD；靠 240–480Hz 子步 + 盒覆盖缓解，已知限制（§4.6） |

---

## 1. 差速器（Differential 组件）

### 1.1 现状与问题

`Transmission::Compute`（`vehicle_impl.cpp:309`）后轴 `drivePerRear = driveForce * 0.5f`，且驱动轮 slaved 到曲轴（`vehicle_impl.cpp:915-916`：左右轮同一命令转速 `crankOmega/totalRatioNow`）。这等价于**100% 锁止差速（spool）**：

- 弯中内/外轮路径长度不同但转速被强制相同 → 内侧轮胎被迫打滑/外侧抓地被浪费，弯中牵引与真实 LSD 不符；
- 无扭矩偏置：真实限滑差速（clutch-type LSD）会把更多驱动扭矩向**更快的外轮**转移，出弯加速时外侧轮抓地更多。

### 1.2 物理模型

**差速壳运动学**（刚性，沿用现有 slaved 语义）：

```
ω_case = ω_crank / i_total          // 差速壳与传动轴刚性连接（曲轴→挡位→差速壳）
ω_L + ω_R = 2·ω_case                // 行星齿轮耦合（平均转速锁定）
定义轮内自旋 Δ = (ω_L − ω_R)/2      // 唯一的差速自由度
⇒  ω_L = ω_case + Δ,  ω_R = ω_case − Δ
```

**扭矩分配**（clutch-pack 模型）：

```
T_axle = 轴驱动扭矩（曲轴净扭矩经挡位，含引擎制动/ERS/TC 削减）
T_L + T_R = T_axle                  // 守恒
T_L − T_R = T_b                     // 偏置扭矩
偏置律：T_b = clamp(preload + k_ramp·(ω_L − ω_R), −T_b,max, +T_b,max)
       T_b,max = lockFactor · |T_axle|
```

- `preload`（N·m）：静摩擦预载——即使左右无转速差也有基础偏置，弯中出弯更早建立偏置；
- `k_ramp`（N·m per rad/s）：转速差越大偏置越大，把扭矩推向快轮（外轮）；
- `lockFactor ∈ [0,1]`：偏置上限比例。**1 = 全锁（spool）= 现状行为**，**0 = 开式差速**——参数连续覆盖整条谱线，便于调参与回归；
- 方向性：真实 F1 LSD 加速锁止高、减速低（减速工况差速近乎开式，避免弯中刹车时车尾甩出）。用两套偏置参数区分加速/减速工况（§1.4）。

**轮端力**：`F_L = T_L / r, F_R = T_R / r`（现有 TireModel 接口的 driveForce 即轮上切向力 N）。

### 1.3 数值方案（每子步）

后轮**离开 slaved 模式**，回到 TireModel 自有 spin 积分（`slaved=false`），差速约束在编排器层实施：

```
1. Δ ← 上一子步的差速自由度（初始化 0）
2. 工况判定（加速/减速按轴扭矩符号）→ 取对应 preload/ramp/lockFactor
3. T_b ← clamp(preload + k_ramp·2Δ, ±lockFactor·|T_axle|)
   偏置变化速率限幅：|ΔT_b| ≤ biasRateLimit·dt（防预载突变振荡）
4. T_L = (T_axle + T_b)/2,  T_R = (T_axle − T_b)/2   →  F_L, F_R
5. 后轮各自 TireModel::Compute(非 slaved, driveForce=F_L/F_R)
   → 隐式接触阻尼解出 Fx_L, Fx_R，并各自积分 ω_L, ω_R
6. 平均转速投影：ω_case = ω_crank/i_total（曲轴刚性）
   ω_L −= ((ω_L+ω_R)/2 − ω_case) · wL/(wL+wR)？——按惯量加权均摊：
   直接等量均摊：ω_L −= e, ω_R −= e,  e = (ω_L+ω_R)/2 − ω_case
7. Δ ← (ω_L − ω_R)/2   （状态更新，供下一子步偏置律使用）
```

要点：

- **稳定性**：轮胎力仍走 TireModel 现有 Backward-Euler 隐式阻尼（`vehicle_impl.cpp:101`），差速只改变"驱动扭矩如何分配"，不进入轮胎隐式解；偏置律对 Δ 是显式前馈 + 速率限幅，240Hz 子步下安全。若调参发现预载振荡，备选方案是 Δ 走隐式（联立两轮），见 §6。
- **drivelineLoad_ 不变**：曲轴负载反射的是后轴合力 `rearFx + rearBrake`（`vehicle_impl.cpp:944`），差速只重分配不改变总和，现有闭环反馈（滑移负反馈）保持成立。
- **TC/ABS 交互**：TC 切油门作用于引擎（轴扭矩源头），差速自然继承；ABS 逐轮刹车在差速**之后**的轮级（§1.5 取舍），不受影响。
- **刹车走不走差速**：F1 后刹在差速壳两侧（inboard），刹车扭矩经差速分配。但 ABS 需要逐轮独立制动力，若刹车经差速分配会互相干扰。**本期取舍：驱动扭矩走差速，刹车力保持轮级直连**（ABS 语义不被破坏；减速工况差速偏置本就调低，行为接近真实）。文档记录此取舍，远期可在 ABS 关闭时让刹车也走差速。
- **clutchOpen / 倒挡**：离合器打开时后轮自由（现状）；倒挡驱动同样走差速（符号一致）。

### 1.4 参数（新增 `DifferentialParams`，嵌入 `TransmissionParams` 或独立组件参数）

| 参数 | 默认 | 说明 |
|---|---|---|
| `preloadAccel` | 40 N·m | 加速工况基础偏置 |
| `rampAccel` | 120 N·m/(rad/s) | 加速工况转速差锁止 |
| `lockFactorAccel` | 0.65 | 加速偏置上限比例（1=spool，0=开式） |
| `preloadDecel` | 8 N·m | 减速工况（含引擎制动） |
| `rampDecel` | 30 N·m/(rad/s) | 减速工况 |
| `lockFactorDecel` | 0.35 | 减速偏置上限 |
| `biasRateLimit` | 4000 N·m/s | 偏置变化速率限幅 |

初值对标 Monoposto/RR3 档手感，落地后在 drive-test 场景（起步/弯中牵引/点刹）与手感回归中调参，默认值只保证行为方向正确。

### 1.5 接口

```cpp
// Transmission 输出改为“轴级”驱动/刹车：
struct Transmission::Output {
    float rearAxleDrive = 0.f;          // 后轴总驱动 [N]（替代 drivePerWheel[2/3]）
    std::array<float,4> brakePerWheel;  // 刹车保持逐轮（ABS 语义）
};

// 新组件 Differential：
class Differential {
    struct Params  { /* §1.4 全部参数 */ };
    struct State   { float delta = 0.f; };          // 差速自由度 Δ [rad/s]
    struct Input   { float axleDrive; float crankOmega; float gearRatio; /* i_total */ 
                     float wheelRadius; float dt; };
    struct Output  { float driveL, driveR; };       // 轮端驱动 [N]
    // Step: 偏置律 → 分配；后轮自旋积分 + 平均投影由编排器/TireModel 协作
    Output Step(const Input& in);
    void Reset() { st_ = State{}; }
};
```

编排器 `Vehicle::Update` 的改动（步骤 4/6 之间）：

```
- Transmission::Compute 产出后轴总驱动 + 逐轮刹车
- Differential::Step(轴驱动, 曲轴转速, 总齿比) → driveL/driveR
- 后轮 TireModel::Compute 用 driveL/driveR、非 slaved；
  差速平均转速投影在轮力后执行（步骤 6）
- 前轮完全不变
```

### 1.6 行为回归要点

- `lockFactorAccel = 1` 时**严格复现现状**（spool）：左右同转速、总驱动 50:50 分配——作为差分回归基准（与当前 drive-test 数值对照）。
- 开式（lockFactor=0, preload=0）：弯中外轮自然多走（转速自由），驱动仍 50:50——验证运动学约束实现正确。
- AI 弯速预算（Game.cpp 0.90）可能因弯中牵引改善而偏保守，验收时观察，可微调。

---

## 2. 横向稳定杆 + 悬挂几何

### 2.1 现状

`Suspension::Step`（`vehicle_impl.cpp:328-400`）：

- 每轮独立弹簧/阻尼（前/后轴参数分离），**无左右耦合**；
- 外倾：`camberStatic ± camberGain-1)·roll`（`vehicle_impl.cpp:389-399`）——`camberGain` 是魔法常数，前/后共用；
- 无主销后倾（caster）、无拖距（trail）回正力矩、无静态束角（toe）、无侧倾转向。

真实 F1 的侧倾刚度与外倾变化来自悬架几何（轮心轨迹/瞬时中心）。本期不求解瞬时中心（§0.3），用**等效参数模型**覆盖其可感知行为。

### 2.2 防倾杆（anti-roll bar）

**模型**：杆把左右压缩差转化为等量反向的载荷修正。设前轴左右压缩差 `Δf = comp[FL] − comp[FR]`：

```
F_arb_L = −kArbF · Δf        // 压缩多的一侧被杆“抬住”，载荷减少
F_arb_R = +kArbF · Δf        // 另一侧被杆压下，载荷增加
normalForce_L += F_arb_L      // 在 Suspension 弹簧力之后修正
normalForce_R += F_arb_R
```

- 左右修正**等量反向**：总载荷不变、无净力——对整车动力学零副作用；
- 滚转力矩自动产生：悬架力经 `ApplyForce(chassis, up·load, contactPoint)` 施加在左右接触点（相距 `trackWidth`），载荷差即滚转力矩，无需额外 ApplyTorque；
- **效果**：调前后防倾刚度比 = 调侧倾刚度分配 = 调转向平衡（前强→转向不足，后强→转向过度），这是 F1 调车的基本旋钮；
- 与弹簧的交互：防倾杆对"总侧倾角"无贡献（总侧倾由弹簧+杆共同决定），它只重分配——正确物理。

**参数**：`kArbF / kArbR`（N/m 等效线性刚度，0=无杆）。默认前 9000、后 12000（相对弹簧 38000 的 ~25-30%，初值，落地调参）。

### 2.3 几何参数（等效模型）

| 参数 | 默认 | 模型 |
|---|---|---|
| `casterTrailF` | 0.035 m（~3.5cm） | 前轮回正力矩 `M_align = (Fy_FL + Fy_FR)·trail`，作用于车体 +Y（横摆）——`Fy` 符号沿几何左为正，方向实现时按惯例复核 |
| `pneumaticTrail` | 0.012 m | 气胎拖距（可选并入 casterTrail 或独立） |
| `toeF / toeR` | 0.0015 rad（~0.09° 内束）/ −0.0005 rad | 静态束角：前轮在 steer 基础上叠加、后轮直接叠加，绕车体 up 旋转 wheelForward/wheelRight |
| `camberStaticF / camberStaticR` | 0.030 / 0.040 rad | 拆开现有 `camberStatic`（前后独立负外倾） |
| `camberGainF / camberGainR` | 0.60 / 0.70 | 拆开现有 `camberGain`（侧倾回摆比例前后独立） |
| `rollSteerF` | 0.0 rad/rad | 侧倾转向：`steer += rollSteerF·φ`，φ = 前轴侧倾角 `(compL−compR)/trackWidth`（真实 F1 侧倾→束角变化，默认关，调参旋钮） |

实现位置：

- **回正力矩**进 `TireModel::Output`（新增 `alignMoment` 字段，`vehicle_impl.cpp:402-408` 的 Output 结构扩展）——轮胎模型最清楚自己的 Fy 方向，Vehicle 编排器汇总 `ApplyTorque(chassis, up·Σalign, 车体质心)`；
- **toe/侧倾转向**进 `Suspension::Step` 的 wheelForward 旋转段（`vehicle_impl.cpp:337-341`）；
- **camber 拆分**：现有公式 `vehicle_impl.cpp:389-399` 参数化即可。

### 2.4 与气动侧倾力矩联动

§3.4 引入的 COP 高度使气动侧向力在高速弯产生**气动侧倾力矩**——该力矩由悬架（弹簧+防倾杆）的侧倾刚度平衡，表现为车体侧倾角与外倾变化。这是"悬挂几何缺失"（审查第 3 点）与"气动侧倾"（审查第 4 点）的自然交汇：**无需额外代码，防倾杆 + camber 独立参数即构成闭环**。验证用例：250km/h 高速弯中侧倾角 vs 低速弯，应明显更大。

---

## 3. 气动：车身轴下压力 + 侧向力/横摆力矩 + COP 高度

### 3.1 现状问题（`vehicle_impl.cpp:857-868`）

- 下压力沿**世界竖直** `-worldUp`（`:860`）——车身俯仰/侧倾时不会"把车压向路面"，在斜坡/路肩/翻转时方向错误；
- 只有阻力与前后下压力：无气动侧向力、无气动横摆力矩（车身侧偏时气动中心偏移的稳定作用完全缺失）；
- 无 COP 高度参数：气动俯仰靠 `aeroPitchBalance` 近似（前后 DF 分配随 pitch 变），气动侧倾力矩不存在。

### 3.2 车身轴下压力

```
ApplyForce(chassis, −frame.up · frontDF, frontPoint)   // 替换 −worldUp
ApplyForce(chassis, −frame.up · rearDF,  rearPoint)
```

一行改动（`frame.up` 已有，`vehicle_impl.cpp:773`）。物理正确性：车体轴向上的压力分量在倾斜路面 = 真正把车压向路面；路肩上单侧轮抬高时下压力随车身姿态正确转向。

### 3.3 侧向力与横摆力矩（气动导数模型）

真实 F1：车身侧偏角 β 时，气动侧向力与横摆力矩是航向稳定的主要来源（>80km/h 后超过轮胎）。用线性气动导数：

```
β = atan2(v_lat_body, v_long_body)            // 车体系侧偏角（现有 DriverControls 同款计算可共享）
q = 0.5·ρ·v²                                  // 动压

Fy_aero = −q · sideArea · Cl_beta · β          // 沿车体 right（负导数：抵抗侧滑）
N_aero  = −q · sideArea · L_ref · Cn_beta · β  // 沿车体 up（横摆稳定力矩）
         − q · sideArea · L_ref · Cn_r · ψ̇     // 可选横摆阻尼导数（大侧偏时稳定）
```

| 参数 | 默认 | 说明 |
|---|---|---|
| `sideArea` | 1.15 m² | 气动侧向参考面积 |
| `aeroClBeta` | 2.6 /rad | 侧向力导数（负号在实现内，抵抗 β） |
| `aeroCnBeta` | 0.9 /rad | 横摆力矩导数（航向稳定） |
| `aeroCnR` | 0.10 /(rad/s) | 横摆阻尼导数（可选，默认开小值） |
| `aeroRefLength` | 3.6 m | 力矩参考长度（= wheelbase） |

作用点：侧向力作用于 COP（含 §3.4 高度）；横摆力矩直接 `ApplyTorque` 车体 up。

**数值安全**：β 在低速/静止时噪声大（atan2 分母小）——按现有惯例用 `max(|v_long|, 3)` 保护；且 `q·v²` 使低速时该项自动趋零，静止无影响。

### 3.4 COP 高度与俯仰/侧倾力矩

新增 `copHeight`（m，COP 相对 CG 的垂直高度）。气动合力（下压力+侧向力）作用点 = 前后轴 DF 作用点 + 侧向力作用点，统一抬升 `copHeight`：

- 俯仰：前后 DF 差已有俯仰力矩（现有行为）；合力作用点抬升后，**阻力/下压力经 `ApplyForce at point` 自动产生纵倾力矩**——不再需要 `aeroPitchBalance` 手工近似（该参数保留为"平衡分配"旋钮，但其物理来源变为 COP 高度，可两者并存：`aeroPitchBalance` 管稳态分配、`copHeight` 管力矩生成）；
- 侧倾：气动侧向力作用于 CG 上方 → 滚转力矩 → 由 §2 悬架侧倾刚度平衡（2.4 联动）。

`copHeight` 默认 0.22 m（F1 气动中心明显高于 CG）。

### 3.5 接口

```cpp
struct AeroKitParams {
    // 现有：airDensity / dragArea / downforceArea / frontDownforceShare
    //      / centerOfPressureOffset / aeroPitchBalance
    float sideArea = 1.15f;      // 新增
    float aeroClBeta = 2.6f;     // 新增
    float aeroCnBeta = 0.9f;     // 新增
    float aeroCnR = 0.10f;       // 新增
    float aeroRefLength = 3.6f;  // 新增
    float copHeight = 0.22f;     // 新增
};
struct AeroKit::Output {
    Vec3 cop;                    // 现有（加高度）
    float frontDF, rearDF, drag; // 现有
    Vec3 sideForce;              // 新增：车体系侧向力（世界）
    Vec3 yawMoment;              // 新增：横摆力矩（世界）
};
```

`Vehicle::Update` 步骤 5 应用：`-frame.up` 下压力 + `aero.sideForce` at cop + `ApplyTorque(aero.yawMoment)`。

---

## 4. 部件独立碰撞盒

### 4.1 现状

`Vehicle::Spawn`（`vehicle_impl.cpp:688-698`）只建一个底盘盒（`chassisHalfX/Y/Z`）；损伤判定是**接触点坐标分区**（`vehicle_impl.cpp:1005-1009`：`z>1.1 → 前翼`、`z<−1.15 → 尾翼`、`|x|>0.52 → 侧箱`、否则底板）。碰撞几何与真实部件不符：前翼盒不存在（车头撞墙记到底板/前翼按坐标猜），高速掠射时翼片区域穿模（设计文档已承认 qu3e 无 CCD）。

### 4.2 L1 接口扩展（多盒 + 部件标签）

**可行性已核实**：qu3e `q3Body::AddBox` 与 rpc（第一方 fork）`rpcBody::AddBox` 均支持一个 body 挂多个局部空间 box，质量/惯量自动按密度重算（`rpcBody.h:56-60`、`qu3e_impl.cpp:199`）。nudge 仅单盒（`nudge_impl.cpp:126-127`）。

```cpp
struct ShapeDesc {                       // 新增
    Transform localTransform;            // 相对 body 的局部位姿
    Vec3 halfExtents;
    void* userData = nullptr;            // 部件标签（CarPart 值经 uintptr 透传）
    float friction = 0.f;                // 0 = 继承 body 摩擦
};
struct BodyDesc {
    // 现有字段不变…
    std::vector<ShapeDesc> shapes;       // 新增：空 = 现有单盒路径（halfExtents）
};
struct ContactEvent {
    // 现有字段不变…
    int partA = -1, partB = -1;          // 新增：shape 标签；-1 = 未知（兼容回退）
};
```

- **qu3e**：`CreateBody` 循环 `body->AddBox` + `q3Box::SetUserdata`（`qu3e/collision/q3Box.h:59-63`）；`BeginContact` 回调的 `q3ContactConstraint` 直接暴露 `A/B` 两个 `q3Box*`（`qu3e/dynamics/q3Contact.h:127-128`），取 `GetUserdata()` → `partA/partB`；
- **rpc**：同上；`rpcBox` 若缺 userdata 字段，第一方 fork 直接加；
- **nudge**：不支持 → `partA/partB = -1`，Vehicle 回退现有坐标分区（行为与现状一致）；
- 摩擦/碰撞开关：部件盒共用 body 摩擦（0 继承），不做每盒摩擦差异。

### 4.3 盒布局（Vehicle::Spawn 组装）

```
前翼 FrontWing:  local {x:0, y:+0.06, z:+1.68}  half {0.72, 0.05, 0.18}
尾翼 RearWing:   local {x:0, y:+0.62, z:-1.66}  half {0.70, 0.12, 0.15}
侧箱 L/R:        local {x:±0.70, y:+0.20, z:0}   half {0.14, 0.16, 1.05}
底板/鼻锥 Floor: local {x:0, y:-0.02, z:+0.5}    half {0.55, 0.06, 1.0}
底盘 Main:       local {x:0, y:+0.15, z:0}       half {0.58, 0.18, 1.35}
```

- 盒位/尺寸进 `VehicleParams`（新增 `partBoxes[CarPart::Count]` + `chassisBox`），游戏可改（改车/不同车种）；
- **质量配平**：多盒时密度 = `mass / Σ(8·hx·hy·hz)`（所有盒体积），总质量精确 = 目标 750kg；盒布局对称 → 质心保持原点；**附带收益**：惯量张量由后端按真实分布计算（宽扁车体 + 悬挑翼片），比当前单盒更真实（横摆惯量 ~m·1.6 的魔法系数可逐步让位于真实张量——本期保留魔法系数用于 ESC/AI 标定，验收后评估迁移）；
- 悬架 raycast 不受影响（`Raycast` 忽略自身 body，多盒自动）。

### 4.4 损伤归属

```
Vehicle::UpdateDamageFromContact：
  若 event.partA/B ≥ 0 → 用部件标签（qu3e/rpc）
  否则 → 现有坐标分区回退（nudge）
```

部件盒挂 userdata = `CarPart` 值 → `AeroKit::ApplyDamage` 直接命中正确部件，**删掉坐标魔法常数**（`z>1.1` 等）；侧箱→底板的连带损伤规则保留。

### 4.5 对游戏层的影响

- `Game::SpawnRacersInto`：`VehicleParams` 新增字段有默认值，游戏层无需改（除非要按车种定制）；
- `ProcessCollisionEvents`（Game.cpp:906-926）：`ContactEvent` 新增字段向后兼容，音频碰撞逻辑不变。

### 4.6 CCD 取舍（已知限制）

qu3e 无 CCD；部件盒不消除高速掠射穿模（相对速度下薄盒可能整帧穿过），缓解手段：

1. 240–480Hz 子步（现状 120Hz 配置，建议本项落地后回归验证 240Hz 默认）；
2. 薄盒（翼片）做小幅膨胀（half 加 2-4cm 容差），把"掠射穿模"降为"掠射擦碰"；
3. 远期：L1 加 raycast-sweep 预检或换 Jolt（接口已留好）。

---

## 5. 数据结构与接口变更汇总

| 位置 | 变更 |
|---|---|
| `physics.cppm` `BodyDesc` | +`shapes`（多盒，空=兼容） |
| `physics.cppm` `ContactEvent` | +`partA/partB`（-1=未知） |
| `physics.cppm` `TireModel::Output` | +`alignMoment`（回正力矩） |
| `physics.cppm` `TransmissionParams` | +`DifferentialParams`（或独立组件参数） |
| `physics.cppm` `SuspensionParams` | +`kArbF/kArbR`、`casterTrailF`、`toeF/toeR`、`camberStaticF/R`、`camberGainF/R`、`rollSteerF` |
| `physics.cppm` `AeroKitParams` | +`sideArea/aeroClBeta/aeroCnBeta/aeroCnR/aeroRefLength/copHeight` |
| `physics.cppm` `VehicleParams` | +`partBoxes[4]` + `chassisBox` |
| `physics.cppm` 新增 | `Differential` 组件（Params/State/Step） |
| `physics.cppm` `Transmission::Output` | 驱动改轴级（`rearAxleDrive`），刹车保持逐轮 |
| `physics.cppm` `VehicleState` | +`diffBias`（调试/HUD 可显示偏置扭矩，可选） |
| `vehicle_impl.cpp` `Suspension::Step` | 防倾杆修正 + toe/roll-steer + camber 拆分 |
| `vehicle_impl.cpp` `AeroKit::Compute` | β 导数、sideForce/yawMoment、cop 高度 |
| `vehicle_impl.cpp` `Vehicle::Update` | 差速接线、-frame.up 下压力、侧向力/力矩应用、alignMoment 汇总 |
| `vehicle_impl.cpp` `Vehicle::Spawn` | 多盒组装 + 密度配平 |
| `vehicle_impl.cpp` `UpdateDamageFromContact` | 标签优先 + 坐标回退 |
| `qu3e_impl.cpp` / `rpc_impl.cpp` | 多盒 + 接触标签（shape userdata） |
| `nudge_impl.cpp` | 无改动（回退路径） |
| `src/game/Game.cpp` | 无必须改动（新参有默认值）；AI 弯速预算观察 |

---

## 6. 数值稳定性与确定性

- **差速**：偏置律对 Δ 显式 + 速率限幅（§1.3）；若振荡，备选 Δ 隐式（两轮联立 2×2 线性解，仍每子步标量规模）。`lockFactor=1` 差分基准回归（§1.6）。
- **防倾杆**：载荷修正与压缩差同帧线性（240Hz 下无相位滞后问题）；注意 `compression` 钳制范围与防倾杆修正叠加后仍应 `max(0, load)`（现有 clamp 保留）。
- **气动 β**：低速保护（§3.3）；横摆阻尼导数 `Cn_r` 与 ESC 横摆阻尼并存——ESC 门限高于气动项（ESC 是过转后才介入），不冲突，验证时确认无叠加振荡。
- **多盒**：后端自动重算惯量——**确定性**由后端既有路径保证（不新增随机）；nudge 回退路径结果与现状一致。
- **确定性**：新增逻辑无随机、无时序依赖；`Configure(0)` 下 drive-test 必须逐位可复现（与现状同准则）。

---

## 7. 落地步骤

| 步骤 | 内容 | 验证 | 状态 |
|---|---|---|---|
| T1 | **差速器**：`Differential` 组件 + Transmission 轴级输出 + 编排器接线（slaved 命令转速 + Δ 动力学）+ 参数默认值 | 单测：spool 50:50 基准 / 弯中偏置方向 / 开式 / 减速工况 / slew 限幅；drive-test 全后端 | ✅ 已实施（提交 5bdfdc4 / 9f8106c） |
| T2 | **防倾杆 + 悬挂几何**：Suspension 扩展 + TireModel `alignMoment` + Vehicle 汇总 | 单测：回正力矩方向/线性/零 trail；drive-test 全后端 | ✅ 已实施（提交 92802e6 / 199dc51） |
| T3 | **气动**：车身轴下压力 + β 导数 + COP 高度 + Vehicle 应用 | 单测：侧滑恢复力/横摆方向、COP 高度、静止为零；drive-test 全后端 | ✅ 已实施（提交 e300b41 / da9426c） |
| T4 | **部件碰撞盒**：L1 多盒/标签（qu3e+rpc）+ nudge 回退 + Spawn 盒布局 + 损伤归属 | 单测：前翼盒标签、主盒 -1、nudge 回退、标签优先损伤、坐标回退、侧箱连带；drive-test 全后端 | ✅ 已实施（提交 acb8668 / d460c5b） |
| T5 | **集成回归**：三后端 drive-test 0 failures + bench 记录 + WASM 冒烟（未配置）+ 手感调参 + 文档状态更新 | 验收全项（§8） | ✅ 已实施（本文档） |

每步沿用既有纪律：先核对本设计对应节 → 编码 → bench + drive-test → 童子军清理（不留死代码/魔法常数）。

### 7.1 T5 回归记录（2026-11-23）

- **drive-test**：qu3e / nudge / rpc 三后端 + threaded-smoke **0 failures**（含新增：diff×10、align×4、aero×6、parts×7 用例）；
- **bench 16 车**（600 帧 × 2400 physSteps）：qu3e 2.180s、rpc 2.072s（T1 前基线 1.855s / 1.818s，**+14–17%**）、nudge 40.998s（≈基线 40.71s，多盒回退路径无额外成本）。退化来源 = 部件多盒贴地 4–6 盒的 narrowphase/solver 接触增加，属功能正确代价，列为 §10 优化靶点；
- **确定性**：`JobSystem::Configure(0)` 下逐位可复现（drive-test 全过）；
- **手感方向**：起步/极速/换挡/高速点刹/倒车/刹车/AI 走廊全过、AI 不上墙、DRS/翼挡/损伤生效（drive-test 断言全绿）；AI 弯速预算 0.90 无需调整；
- **参数终值 = 文档默认**（T5 调参结论：初值行为方向正确，无需改动）：
  - 差速：preload 40/8 Nm、ramp 120/30、lockFactor 0.65/0.35、biasRateLimit 4000、deltaDamping 20、deltaMax 1.0 rad/s；
  - 防倾杆：kArbF 9000 / kArbR 12000；casterTrailF 0.035 m；toeF 0.0015 / toeR −0.0005 rad；camber 0.030/0.040 + gain 0.60/0.70；rollSteerF 0；
  - 气动：sideArea 1.15、Clβ 2.6、Cnβ 0.9、CnR 0.10、Lref 3.6、copHeight 0.22 m；
  - 部件盒布局：VehicleParams.partBoxes/chassisBox 默认值（§4.3）；
- **WASM 冒烟**：未跑（本机未配置 EMSDK，与现状一致）。

---

## 8. 验收标准

- 三物理后端 `--drive-test` 0 failures（含新增用例：差速偏置/全锁基准、防倾杆载荷、气动 β、部件盒损伤归属）；
- `--bench` 百车分段计时与基线同量级（无退化）；
- 确定性：`JobSystem::Configure(0)` 下结果可复现；
- 手感方向不变：起步/极速/换挡/高速点刹/倒车/刹车/AI 走廊用例全过、AI 不上墙、DRS/翼挡/损伤生效；
- 新参数默认值交付并在 T5 完成调参（给出手感回归记录：弯中牵引、高速弯侧倾、出弯加速对比基线）；
- WASM 冒烟：若本机 EMSDK 可用则重跑（与现状同标准，未配置则标注未跑）。

---

## 9. 远期（明确不做，记录备查）

- **纵向 Magic Formula**：现有线性 kStiff 饱和可升级为与侧向同款 B/E/C/D 曲线（Shape 函数已存在，`vehicle_impl.cpp:18`），峰值-回落形状更真实；需重标 TC/ABS 阈值；
- **簧上/簧下拆分**：独立轮质量 + 底盘柔性，悬架响应更真实（乘感提升），工程量中等；
- **翼片断裂**：部件盒→独立刚体 + 铰链/断裂阈值（真实 F1 前翼断裂减下压力），需 L1 约束支持；
- **全运动学悬架**：瞬时中心/双叉臂求解（等效参数模型的下一步）；
- **CCD**：换 Jolt 或 L1 加 sweep；
- **刹车经差速**：ABS 关闭路径下的 inboard 刹车分配（§1.5 取舍的延伸）。

---

## 10. 性能优化规划（功能落地后执行）

**范围裁定（用户 2026-11-23 明确）**：nudge 不是优化目标，一切优化**只针对自有管线 RPC**——即 L1 `rush-physics-core`（第一方 fork）、L2 `vehicle_impl.cpp`（跨后端共享，优化同样惠及 qu3e，但度量只认 RPC）、游戏层 AI/编排/整合路径。qu3e（stock）、nudge（第三方移植）不修改。

### 10.1 度量基准（`--bench 16`，600 帧 × 2400 physSteps，本机基线）

| 段 | RPC per-frame ms | 占比 |
|---|---|---|
| step（L1 solver/broadphase） | 1.948（=0.4869ms/step×4） | 64.3% |
| vehicle（L2） | 0.492（=0.1230ms/step×4） | 16.2% |
| ai（游戏层 AI 赛道投影/转向） | 0.503 | 16.6% |
| race / contact | 0.042 | 1.4% |
| **RPC wall（16 车 10s 仿真）** | **1.818 s** | 100% |

### 10.2 目标

- **硬目标**：RPC `--bench 16` wall ≤ 1.35s（较基线 **≥25%** 降低）；
- **冲刺目标**：wall ≤ 0.91s（**≥50%** 降低）——需 step + vehicle + ai 三段同步压（见下）。
- 约束：确定性档 `JobSystem::Configure(0)` 下 drive-test 仍 0 failures 且逐位可复现；手感方向不变（验收 §8 全项重跑）。

### 10.3 诊断角度与初定靶点（每项落地前先 profiler/分段计时确认）

1. **热路径判断与分支**：L1 solver 内层迭代中的 `if`/虚调用/能力枚举检查；L2 `Vehicle::Update` 每子步重复的 `GetTransform/GetLinearVelocity/GetAngularVelocity`（每子步 3 次 world 往返，可合并为一次后端快照查询）。
2. **重复计算**：Suspension 每轮重复 `Quat::FromAxisAngle`+`Rotate`（转向四元数每子步 4 轮重复构造 2 次，前后轮共 8 次）；AeroKit `pitch=asin` 每子步重复；TireModel 内 `atan2/tan/atan` 链可合并；`SteerEnvelopeDeg` 每子步在 controls/AI 两侧重复调用。
3. **缓存利用与带宽**：L1 body/constraint 内存布局（AoS→SoA 或按热路径重排）、`std::map`/`unordered_map` 句柄查表（L2 每子步多次 hash/红黑树查找——可换稠密数组索引）；ContactEvent 全量拷贝经 `CollisionBus` 每帧一次。
4. **算法**：L1 solver 迭代次数/热启动路径、broadphase 更新；L2 轮胎隐式解中 `pow/atan2` 的近似替换（精度受 drive-test 逐位确定性约束——近似必须位级稳定）。
5. **架构**：跨模块边界调用次数（IPhysicsWorld 每子步 ~15 次虚调用/车，16 车 = ~240 次/子步 → 960/帧）；评估"每子步单次批查询"接口（如 `FetchChassisState`）是否值得进 L1 接口（注意三后端兼容、nudge 回退实现）。

### 10.4 交付与现状（2026-11-23 晚）

**已实施**（rpc + 共享 L2/游戏层，qu3e 只保留功能必需改动）：

1. **AI 段 -42%**：`Track::ProjectDistance(pos, hint)` 邻域搜索（连续运动 O(窗口) 替代 O(样本数)，hint 失效自动回退全扫描）；16 车 ai 0.528→0.305 ms/frame。
2. **L2 批查询**：`IPhysicsWorld::FetchChassis`（transform+双速度一次后端往返，替代 3 次句柄查找；三后端实现）。
3. **rpc 适配层热路径**：句柄表 `unordered_map`→稠密 vector（handle 递增 → 数组索引）；`ApplyForce` 提前解析 body 指针（Step 免二次查找）；热路径去 Result 包装（Debug assert，错误契约保留在低频路径）；**冲量聚合**（每 body 一次线性+角冲量应用，替代 ~11 次逐点惯性计算，数学等价）。
4. **L1 岛批处理**（rpcScene）：span 池 + `std::vector` 复用（无裸指针分配、无每步分配）；静态 body 不撑大池（`m_dynamicCount` 缓存）；接触池按峰值单调增长（消除接触抖动引发的逐帧 realloc）；同步段只清动态 body；SolveIslands 单一路径（串行/并行共用，数值逐位一致）。
5. **并行岛（D9 落地）**：进程级**共享单例线程池**（游戏启动预热、CPU 核数-1、永不动态扩容/销毁、空闲背压 spin→yield）；门控 `islandCount ≥ 4×workers`（16 车网格保持串行，120 车并行）；**并行与串行数值逐位一致**。
6. **bench 1% low**：逐帧/逐步样本记录，报告最差 1% 平均（帧时间与物理步），覆盖抖动而非只看均值。

**当前数据（--bench 16，600 帧 × 2400 steps）**：

| 项 | 功能基线 | 现状 |
|---|---|---|
| rpc wall | 2.072s | **2.061s**（-0.5%；ai -42% 抵消了多盒/新物理增量） |
| rpc ai | 0.528 ms/f | 0.305 ms/f（-42%） |
| rpc step | 0.568 ms/step | 0.572 ms/step（持平） |
| rpc 1% low frame | —（未测） | 4.33 ms（231 fps），优于 qu3e 5.07 ms |
| qu3e wall | 2.180s | 2.119s（无退化） |
| 120 车 rpc wall（并行） | —（初版并行 10.84s） | 7.71s（-29%） |

**未达 -25% 目标的差距与根因**：
- 16 车 rpc 平均仅持平——剩余瓶颈是 **vehicle 段**（rpc 0.204 vs qu3e 0.163 ms/step）：悬架 **4×Raycast/子步/车**（120 车 = 480 次树查询）是主体，rpc 树查询慢于 qu3e（同源 fork 的分化点，下一步：树查询热点对比 + 悬架 raycast 聚合接口）；
- 多盒碰撞（T4）带来的 solver/窄相增量是功能正确代价（-14%），已通过上述优化抵消；
- **1% low step**（0.85 vs 平均 0.57，+49%）主要来自车车碰撞/高接触峰值步——可在并行门控下进一步平滑。

**下一步（若继续）**：悬架 4 轮 raycast 批量接口（1 次树遍历返回 4 结果）；rpc 树 vs qu3e 树查询分差定位；16 车 vehicle 段 EX_TRY 以外的残余开销。

**确定性**：drive-test 三后端 + threaded-smoke 全 0 failures（含全部新用例），`Configure(0)` 下逐位可复现；并行路径与串行路径数值逐位一致（岛间无共享写）。
