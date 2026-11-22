# 统一输入系统

状态：**已实施（P0–P4 全部落地）**  
目标读者：维护输入模块与接入新动作的开发者  
规范依据：`~/.config/mimocode/AGENTS.md`（通用 AI 编程架构与工程规范）  
关联：[architecture.md](./architecture.md) · [rpc-refactor.md](./rpc-refactor.md) · `rush-engine/modules/{input,events,render}.cppm` · `src/game/Game.cpp`（`RegisterGameInput`）

---

## 0. 优先级与冲突裁决

按 AGENTS.md §零：

| 优先级 | 来源 | 本方案中的体现 |
|--------|------|----------------|
| 1 | 用户即时指示 | **独立 input 线程**；**模块自注册按键+事件名**；**纯事件队列、禁止回调** |
| 2 | 项目特定要求 | L2 不含玩法语义；与 `CollisionBus` 同构的 `Publish`/`Drain`；WASM 可退化为同线程泵 |
| 3 | 通用规范 | 分层 / SRP / 接口先行 / 可插拔 / Result / 可测试 / 无硬编码键位 if-else |

**硬约束（不可被通用规范推翻）：**

1. **输入不绑渲染线程**：采样与手势识别在 **input 线程**；渲染只在自己的 stage `Drain` 或读快照。  
2. **无回调系统**：生产者只 `Publish` 输入事件；消费者只在己方 stage `Drain`——与 `rush::EventBus` / `CollisionBus` 同一模式。  
3. **注册表驱动**：需要输入的模块注册「事件名 + 手势类型 + 多个按键」；引擎内不得出现 `KEY_W → 油门` 之类玩法硬编码。  
4. **不引入异常**控制业务流；绑定注册/加载失败用 `rush::Result`（L2）。

---

## 1. 背景与目标

### 1.1 现状一句话

`IInput` / `InputState` 寄生在 `rush.engine.render`；`RaylibBackend::Poll()` 每帧硬编码键位并直接吐出玩法语义字段；`Game` 经 `PlayerIntent` 邮箱把该结构拷到 sim 线程。

### 1.2 为什么重构

| 驱动 | 证据 |
|------|------|
| **线程耦合** | 输入采样与渲染帧循环绑死；无法独立节拍、独立退避 |
| **硬编码映射** | `raylib_impl.cpp` `Poll()` 一整段 `IsKeyDown(KEY_W) \|\| …` |
| **语义泄漏** | `drsToggle` / `menuUp` 等 L3 概念出现在 L2 render 模块 |
| **边沿手写** | `IsKeyPressed` + `PlayerIntent` set-only，分散且难测 |
| **不可插拔** | 换输入源必须动渲染类；双继承 `IRenderer+IInput` |

### 1.3 目标

1. 新建 L2 模块 **`rush.engine.input`**：独立 **input 线程** + **注册表** + **纯事件总线**。  
2. 模块（L3 或 L2 组件）**自注册**所需事件名、手势类型、多个触发键。  
3. 手势类型覆盖：**按住（Hold）**、**按下/抬起（PressRelease）**、**单击/双击/长按（Click / DoubleClick / LongPress）**。  
4. **零回调**：只 `Publish` / `Drain`；与 `CollisionBus` 同构。  
5. 默认绑定与现网键位等价，保证 `--drive-test` 与菜单/比赛行为可迁移。

### 1.4 非目标（本期）

- 键位重绑定设置 UI（注册与 `LoadBindings` 接口预留）  
- 本地分屏多玩家输入路由  
- 触屏/手势专用通道  
- 在引擎内解释「DRS / 菜单」语义  

---

## 2. 分层与所有权

```
┌─────────────────────────────────────────────────────────────┐
│ L3  apex-rush（src/）                                        │
│     GameInputRegistrar：注册 throttle/brake/drs/menu…        │
│     各 stage：Drain(InputBus) → 写 PlayerIntent / 改菜单游标   │
│     import rush.engine.{core,input,render,physics,…}         │
└────────────────────────────┬────────────────────────────────┘
                             │ 只依赖 rush::engine 公共模块面
┌────────────────────────────▼────────────────────────────────┐
│ L2  rush.engine.input（新模块）                               │
│     InputThread · InputRegistry · InputSystem · InputBus     │
│     IInputSource（Raylib / Null）· 手势状态机                 │
│     （不 import render；不依赖玩法枚举）                       │
└───────────────┬─────────────────────────┬───────────────────┘
                │                         │
        IInputSource 实现            rush::EventBus<InputEvent>
        RaylibInputSource            （复用 events.cppm 模式）
        NullInputSource
```

| 路径 | 层 | 所有权 |
|------|----|--------|
| `rush-engine/modules/input.cppm` | L2 接口 | 类型、`IInputSource`、`InputRegistry`、`InputSystem`、`InputBus` |
| `rush-engine/src/input/input_system.cpp` | L2 实现 | 线程循环、手势状态机、Publish |
| `rush-engine/src/input/raylib_source.cpp` | L2 实现 | 设备采样（依赖窗口已 Init） |
| `rush-engine/src/input/null_source.cpp` | L2 实现 | scripted / 无头确定性 |
| `rush-engine/modules/render.cppm` | L2 | **删除** `IInput` / `InputState`；后端不再实现输入 |
| `src/game/input_actions.*` | L3 | 注册表填充、事件名常量、事件 → `VehicleInput`/菜单 |

**依赖方向硬约束**

- `rush.engine.input` **不得** `import rush.engine.render`。  
- `rush.engine.render` **不得** `import rush.engine.input`。  
- `main` 组装顺序：渲染 `Init` 开窗 → 创建 `RaylibInputSource` → 启动 `InputSystem` 线程 → 构造 `Game`。

---

## 3. 核心模型

### 3.1 设备层（无玩法语义）

```cpp
// rush.engine.input
enum class KeyCode : uint16_t { Unknown, A, B, /* … */, W, Up, Escape, /* … */ };

enum class DeviceKind : uint8_t { Keyboard, Mouse, Gamepad };

struct DeviceKey {
    DeviceKind kind;
    uint16_t code;        // KeyCode / mouse button / gamepad button
    uint8_t deviceIndex;  // gamepad slot；键盘/鼠标为 0
};

// 一帧原始采样（input 线程写入，仅内部消费）
struct RawFrame {
    // 稀疏按下集合 + 边沿；实现可用固定上限数组，避免每帧堆分配
    // mouse 相对位移、滚轮一并携带（本期不映射玩法，留扩展）
};
```

### 3.2 注册表：模块自注册按键与事件名

需要输入的模块在 **启动阶段（InputSystem 启动前或持锁 Register）** 注册，而不是在 `Poll` 里写死。

```cpp
enum class Gesture : uint8_t {
    Hold,           // 按住：Begin + End（+ 可选 Level 脉冲），适合油门/刹车/转向按住
    PressRelease,   // 按下/抬起：同一注册名派生两个事件名
    Click,          // 单击完成
    DoubleClick,    // 双击完成
    LongPress,      // 长按完成
};

struct BindingSpec {
    DeviceKey key;           // 单个触发键
    float axisScale = 1.f;   // Hold 轴向：+1 / -1；数字键作轴用
    bool invert = false;
};

struct Registration {
    std::string name;              // 事件基名，如 "throttle" / "drs" / "menu.select"
    Gesture gesture;
    std::vector<BindingSpec> keys; // ≥1；W 与 ↑ 都可绑到同一 name
    // 手势参数
    float holdTickHz = 0.f;        // >0：Hold 期间按频率发 Level（油门连续量）
    float doubleClickWindowMs = 300.f;
    float longPressMs = 500.f;
    std::string context;           // 可选："race" | "menu" | ""（全局）
};

class InputRegistry {
public:
    // 重复 name + 同 gesture：覆盖（重绑）；冲突策略见 §6
    rush::Result Register(Registration reg);
    rush::Result Unregister(std::string_view name, Gesture g);
    // 配置覆盖：config.rushconf [input] 段 → 替换 keys，不改 gesture/语义
    rush::Result ApplyOverrides(const BindingOverrideTable& table);
};
```

**事件名派生规则（纯数据，无回调）**

| Gesture | 入队事件名 | 说明 |
|---------|------------|------|
| `Hold` | `<name>.begin` / `<name>.end` / `<name>.level` | `level` 仅当 `holdTickHz > 0`，带 `value ∈ [0,1]` |
| `PressRelease` | `<name>.down` / `<name>.up` | 一次注册、两个事件名 |
| `Click` | `<name>.click` | 抬起且未双击/未超长按时入队 |
| `DoubleClick` | `<name>.dblclick` | 第二次按下达成窗口时入队 |
| `LongPress` | `<name>.longpress` | 按住超过 `longPressMs` 时入队（一次） |

同一 `name` 可注册多种 Gesture（如 `drs` 的 `Click` + `PressRelease`）；键位可重叠，状态机按注册并行判定。

**示例（L3 注册，替代现 `Poll()` 硬编码）**

```cpp
// src/game/input_actions.cpp — 启动时一次注册
reg.Register({.name = "throttle", .gesture = Gesture::Hold,
              .keys = {{.key = {DeviceKind::Keyboard, KeyCode::W}},
                       {.key = {DeviceKind::Keyboard, KeyCode::Up}}},
              .holdTickHz = 60.f, .context = "race"});

reg.Register({.name = "brake", .gesture = Gesture::Hold,
              .keys = {{KeyCode::S}, {KeyCode::Down}},
              .holdTickHz = 60.f, .context = "race"});

reg.Register({.name = "steer.left",  .gesture = Gesture::Hold,
              .keys = {{KeyCode::A}, {KeyCode::Left}}, .axisScale = -1.f,
              .holdTickHz = 60.f, .context = "race"});
reg.Register({.name = "steer.right", .gesture = Gesture::Hold,
              .keys = {{KeyCode::D}, {KeyCode::Right}}, .axisScale = +1.f,
              .holdTickHz = 60.f, .context = "race"});

// DRS / 尾翼 / 菜单：单击
reg.Register({.name = "drs",   .gesture = Gesture::Click, .keys = {{KeyCode::F}}, .context = "race"});
reg.Register({.name = "wing",  .gesture = Gesture::Click, .keys = {{KeyCode::V}}, .context = "race"});
reg.Register({.name = "menu.select", .gesture = Gesture::Click,
              .keys = {{KeyCode::Enter}, {KeyCode::Space}}});  // 全局或 "menu"

// 需要按下/抬起两侧的：一次注册 → 两个事件
reg.Register({.name = "handbrake", .gesture = Gesture::PressRelease,
              .keys = {{KeyCode::Space}}, .context = "race"});
// → "handbrake.down" / "handbrake.up"
```

### 3.3 纯事件：InputBus（无回调）

复用 `rush.engine.events` 的 `EventBus<T>` 语义（或薄封装同一实现），**禁止** `OnKey` / `Bind(fn)` 类 API。

```cpp
struct InputEvent {
    uint32_t nameId;       // 注册时 intern 的稳定 ID（比较用）
    // name 字符串仅调试/日志；热路径走 nameId
    Gesture gesture;
    float value;           // Hold.level：0..1；Hold.begin/end 可为 1/0
    double tSec;           // InputSystem 时钟（steady）
    DeviceKey source;      // 触发源键（调试、重绑冲突诊断）
};

// 全局或由 InputSystem 持有：与 CollisionBus() 同级访问方式
rush::EventBus<InputEvent>& InputBus();  // 或 InputSystem::Bus()
```

**生产者**：仅 input 线程在手势状态机内 `Publish`。  
**消费者**：菜单 stage、比赛意图 stage 各自在 `Update*` 开头 `Drain`——**从不**在注册回调里执行玩法。

### 3.4 InputSystem 与独立线程

```cpp
class IInputSource {
public:
    virtual ~IInputSource() = default;
    // 由 InputSystem 线程调用；不得假设在渲染线程
    virtual void Poll(RawFrame& out) = 0;
};

class InputSystem {
public:
    InputSystem(IInputSource& source, InputRegistry& registry);
    ~InputSystem();  // 停止并 join 线程

    // 注册表变更：启动前任意线程；运行中 Register/ApplyOverrides 需
    // 暂停采样或持同一把 registry 锁（实现二选一，文档见 §6）
    void Start();   // 拉起 input 线程
    void Stop();

    // 仅测试 / WASM 单线程模式：手动跑一拍（不启线程）
    void PumpOnce();

    rush::EventBus<InputEvent>& Bus();
};
```

**线程模型**

```
[input 线程]                         [渲染/主线程]              [sim 线程]
  loop @ ~120–240 Hz 或与源节拍对齐
    source.Poll(RawFrame)
    手势状态机（按 Registry）
    Bus.Publish(InputEvent…)
         │                                  │                         │
         │                                  │  UpdateMenu:           │  ApplyPlayerIntent:
         │                                  │    Drain → 游标/确认     │    Drain → 累加 hold
         │                                  │                         │    → VehicleInput
         │                                  │  （渲染只画，不采键）     │
```

| 规则 | 说明 |
|------|------|
| 谁 Poll | **仅 input 线程**（`PumpOnce` 例外：测试/WASM） |
| 谁 Drain | 各玩法/菜单 **自己的 stage**；渲染路径不读原始键 |
| 谁碰 Registry | 启动期单线程；运行期变更走锁或双缓冲注册表快照 |
| 背压 | 与 `EventBus` 一致：队列上限，溢出丢最旧，**永不阻塞** input 线程 |
| 与 sim 邮箱 | Hold 连续量仍可在主线程 Drain 后写入 `PlayerIntent`（轴覆盖）；离散边沿 set-only——**保留现网跨线程语义**，仅把「边沿产生点」换到 InputBus |

**WASM / drive-test**

- `__EMSCRIPTEN__` 或 `--drive-test`：不启线程，`PumpOnce()` 插在帧循环（与现单线程菜单一致）。  
- `NullInputSource`：脚本写入 `RawFrame` 或直接 `Bus.Publish`（测试夹具），断言事件序列确定性。

---

## 4. 手势状态机（input 线程内部）

对每个已注册 `(name, gesture, key)` 维护独立状态；多键任一命中即触发（OR）。

```
                    ┌──────────── Hold ────────────┐
     down           │  value = axisScale           │  up
  ──────────► [Held] ──holdTickHz──► Publish .level ──► Publish .end
       Publish .begin         │
                              └── 超时/失焦可选：仍发 .end

  PressRelease: down → Publish .down；up → Publish .up
  Click:        短按下抬起 → Publish .click（若在双击窗口内改判 dblclick）
  DoubleClick:  两次 click 节奏 → Publish .dblclick（并抑制多余 .click，策略固定）
  LongPress:    持续 down 超 longPressMs → Publish .longpress（仅一次；
                是否同时抑制 click：长按后抬起不再发 click）
```

**上下文门控**：`Registration.context` 非空且与 `InputSystem::ActiveContext()` 不一致时，该注册不产生事件（状态机冻结，避免抬起时误发 `.up`——失活时强制 `End`/`抑制` 策略实现时写死并单测）。  
菜单/比赛切换由 L3 调 `SetActiveContext("menu"|"race")`（主线程，与 Drain 同 stage）。

---

## 5. 与现网数据流的迁移映射

| 现网 | 迁移后 |
|------|--------|
| `RaylibBackend::Poll()` → `InputState` | `InputThread` + `Registry` 默认表 → `InputBus` 事件 |
| `Game::UpdatePlayerInput` 读 `raw.*` | `Update*` 中 `Drain(InputBus)` |
| `raw.throttle` 连续量 | `throttle.level` 事件累加/覆盖到 intent 轴 |
| `raw.pause`（Pressed） | `pause.click` 或 `pause` 的 `Click` |
| `raw.drsToggle` | `drs.click` |
| `raw.menuUp` 等 | `menu.up.click`（注册名自定，L3 统一常量） |
| `PlayerIntent` 跨线程 | **保留**：主线程 Drain 后写入；sim 只消费邮箱 |
| `NullBackend.scripted` | `NullInputSource` 脚本帧或脚本事件注入 |
| `IInput` 构造注入 | `Game` 改为注入 `InputSystem&`（或 `InputBus` 访问器 + registry 属主） |

**默认绑定验收**：与现 `Poll()` 键位一致（W/↑ 油门，S/↓ 刹车，A/D/←/→ 转向，Space 手刹，R 倒车，F DRS，V 翼，Shift 超车，C 相机，Esc/P 暂停，Enter 重开/确认，B 回看，方向键+WASD 菜单）。

---

## 6. 注册、配置与 Result

### 6.1 运行期变更

| 场景 | 策略 |
|------|------|
| 启动 | 单线程完成全部 `Register` → `Start()` |
| 设置页改键（后期） | 主线程 `ApplyOverrides`；实现持 `registryMutex_`，input 线程每拍拷贝快照或短暂暂停采样 |
| 冲突 | 同一 `DeviceKey` + 同 `context` 绑到不同 `name+gesture`：`Register` 返回 `Err(DuplicateBinding)`，由调用方决定覆盖策略（默认拒绝，测试可强制覆盖） |

### 6.2 配置格式（KVRec / `config.rushconf`）

```
[input]
throttle.keys = keyboard.w, keyboard.up
drs.keys = keyboard.f
# 只覆盖 keys；gesture / name 以代码注册表为准
```

`LoadBindings` / `ApplyOverrides` 返回 `rush::Result`；非法键名 → 错误串，不抛异常、不半改。

### 6.3 AGENTS.md 对照

| 条款 | 落地 |
|------|------|
| 严格分层 | 新模块；render 去输入；事件名由 L3 注册 |
| 单一职责 | Source=采样；System=线程+手势；Bus=队列；Game=消费 |
| 接口先行 | `IInputSource` / `InputRegistry` / `InputSystem` 先于实现 |
| 可插拔 | Raylib + Null（+ mock 源） |
| 拒绝硬编码 if | 键位全在注册表数据 |
| Result | Register / ApplyOverrides |
| 可测试 | `PumpOnce` + Null 源断言事件序列 |
| 无回调 | 仅 Publish/Drain，与 CollisionBus 同构 |
| 童子军 | 删除 render 中 `IInput`/`InputState` 与双继承 |

---

## 7. 实施状态（P0–P4 已落地）

| 阶段 | 内容 | 状态 |
|------|------|------|
| **P0 骨架** | `modules/input.cppm`：`KeyCode`/`RawFrame`/`InputState`/`Gesture`/`Registration`/`InputRegistry`/`InputEvent`/`IInputSource`/`NullInputSource`/`InputSystem`；`PumpOnce` 手势机 | 已实施（`rush-engine/src/input/input_system.cpp`） |
| **P1 线程** | `InputSystem::Start/Stop` 独立线程；WASM/drive-test 走 `PumpOnce`；`RaylibInputSource` | 已实施（`rush-engine/src/input/raylib_source.cpp`；交互主循环 `inputSystem.Start()`） |
| **P2 等价迁移** | `Game::Init` 调 `RegisterGameInput`；`UpdatePlayerInput` 改为 `Consume(rawInput_)`；render 删除 `IInput`/`InputState`/`Poll`；测试 `ScriptedInput` 辅助 | 已实施；`--drive-test` 全过（三物理后端 + threaded smoke） |
| **P3 配置** | `InputRegistry::ApplyOverrides(config_, "input")`，`[input] <name>.keys = "w,up"` 换键，保留手势/scale；非法键名 `Result` 错误 | 已实施；单测覆盖合法/非法 rebind |
| **P4 可选** | 手柄 `DeviceKind::Gamepad`；设置页键位 UI | 未实施（扩展点已留：`BindingSpec` 支持 `axisIndex`/游戏柄键） |

**实现约定（与代码逐条对齐）：**

- Click / DoubleClick / LongPress 在 **按下边沿** 触发（匹配旧 `IsKeyPressed` 语义，测试的单帧 press 立即可见）；DoubleClick 第二次按下在窗口内发 `.dblclick` 并抑制当次 `.click`。
- `InputSystem::Consume(InputState&)` 每帧把 Hold 动作值（`ActionValue`）+ 事件边沿折叠成游戏语义快照；边沿事件按「`base` / `base.click` / `base.down` / `base.dblclick` / `base.longpress`」匹配。
- 独立线程模式下 `Game::UpdatePlayerInput` 只 `Consume`（线程负责 `PumpOnce`）；内联模式（drive-test / WASM / 无头）由 Game 在 `Consume` 前调用 `PumpOnce`。
- 上下文切换：`menuPhase → SetActiveContext("menu")`，比赛阶段 `"race"`；非活动上下文冻结识别状态，避免跨阶段误发事件。

## 8. 风险与决策点

1. **Hold 连续量 vs 纯事件**：油门/刹车/转向用 `Hold` + `.level` 事件（`holdTickHz=0` 时每拍发）；游戏每帧 `Consume` 折叠成 `InputState`。脚本测试经 `RawFrame.axes` 注入模拟量（键盘本身给 0/1）。  
2. **raylib 全局窗口**：`RaylibInputSource` 要求 `RaylibBackend::Init` 已调用；`main` 固定顺序即可，Source 不持有 `IRenderer*`。  
3. **双击与单击互斥**：固定策略（成 dblclick 则抑制当次 click）并写入单测，避免各消费方各解一套。  
4. **事件 nameId intern**：启动期 `unordered_map<string,uint32_t>`；Drain 热路径只比整数。  
5. **与 ENet 客户端发输入**：客户端本地仍跑 InputSystem，把 **离散事件 + 周期 hold 快照** 打成包——协议层是 L3，不在本模块内。

---

## 9. 决策摘录

| 决策 | 结论 |
|------|------|
| 输入线程 | **独立 input 线程**；不绑渲染帧循环（WASM/测试 `PumpOnce` 例外） |
| 交互模型 | **纯事件** `Publish`/`Drain`；**禁止**输入回调 API |
| 注册 | 模块注册 **事件名 + Gesture + 多键**；引擎无玩法键位硬编码 |
| Gesture 集合 | `Hold` · `PressRelease` · `Click` · `DoubleClick` · `LongPress` |
| 模块位置 | `rush.engine.input`（与 render 拆分） |
| 跨线程玩法 | 保留 `PlayerIntent` 邮箱；事件在主线程 Drain 后写入 |
| 与 CollisionBus | 同一 `EventBus` 模式，不另造回调总线 |
