# 界面控件层（rush.engine.ui，现行）

状态：**规范**。控件渲染与控件操控归 L2（`rush-engine/modules/ui.cppm`），游戏只声明界面和处理事件。
关联：[architecture.md](architecture.md) · [input.md](input.md) · [car-parts.md](car-parts.md)

---

## 1. 分层与职责

| 层 | 负责 | 不负责 |
|----|------|--------|
| 引擎 `rush.engine.ui` | 焦点移动、行高亮、数字盘/文本编辑、控件绘制、配色 | 知道"这是设置页还是大厅"、业务状态、跳转 |
| 游戏 L3 | 声明 `UiScreen`（有哪些行、每行 id/文本/值）、按 `UiAction.id` 分派、维护自己的子状态机 | 手写游标移动、手写高亮、手写文本编辑 |

判据：**游戏里不应该再出现"`cursor = (cursor+1) % n` + 按行号 switch + 画 `"> "` 前缀"这种代码**。出现即违规。

## 2. 数据

```cpp
enum class UiKind { Label, Header, Button, Value, Digits, TextField };

struct UiWidget {
    UiKind kind; std::string id;    // id 只在需要回报动作的行上填
    std::string label, value;
    int size = 22; int digits = 4;  // Digits: 位数；TextField: 字节上限
    bool enabled = true;
    bool focused = false; int subCursor = 0;   // 引擎写
    bool customColor = false; Color color;
};

struct UiScreen {
    std::string title, hint;
    std::vector<UiWidget> widgets;
    int cursor = 0;                 // 引擎写
    int titleSize; float x, y, rowStartY, rowHeight, valueColumnX;
    bool dimBackground = true;
};

enum class UiActionKind { None, Activate, AdjustLeft, AdjustRight, Back, TextChanged };
struct UiAction { UiActionKind kind; std::string id; int delta; };

class UiController {
    static void ResetFocus(UiScreen&);
    UiAction Update(UiScreen&, const InputState&);
    void Draw(const UiScreen&, IRenderer&) const;
};
```

行的语义：

| kind | 可聚焦 | 左/右 | 上/下 | Enter |
|------|--------|-------|-------|-------|
| Label / Header | 否 | — | — | — |
| Button | 是 | — | 移动焦点 | `Activate(id)` |
| Value | 是 | `AdjustLeft/Right(id, ±1)` | 移动焦点 | — |
| Digits | 是 | 移动第几位 | 改这一位的数字（0–9 回绕）→ `Adjust*(id, 新数字)` | `Activate(id)` |
| TextField | 是 | — | — | `Activate(id)` |

焦点默认**循环**（与旧的手写菜单 `(cursor+1)%rows` 一致）；需要到端点停住的屏（短确认框、表单）在构建时写 `screen.wrapFocus = false`。这是数据开关，不是让游戏层自己再实现一遍移动。

## 3. 三条硬规则

1. **Screen 是持有式数据，不要每帧重建。** `cursor`/`focused`/`subCursor`/文本缓冲都在 screen 里。每帧 new 一个 `UiScreen` 会让焦点回到第一行（玩家按一次"上"就跳回顶部）。做法：把 screen 作为成员持有，只在**屏幕打开时** `ResetFocus`，其余帧就地更新动态内容（设置值文本、房间列表条目）。
2. **分派只认 `UiAction.id`，不认行号。** 行数会变（房间列表），行号不是稳定标识。
3. **TextField 独占一屏。** 聚焦文本框时它吞掉所有导航键（方向键/WASD 都算字符输入），只认 Enter（= `Activate`）和 Esc（= `Back`）。所以不要在同一屏里混"文本框 + 可导航行"。

## 4. 输入来源

控件层消费的是已经归一化的 `InputState`（见 [input.md](input.md)）：

- `menuUp/menuDown/menuLeft/menuRight/menuSelect/menuBack` —— 由 `menu.*` 事件折叠而来，与具体按键解绑，改键只改输入注册表。
- `text[96]/textLen/textBackspace` —— 文本通道：`RawFrame.text`（raylib `GetCharPressed` 的 Unicode 码点）→ UTF-8 字节。**Backspace 不经过手势注册表**（它是原始设备键，只在文本通道里作为一次性边沿上报），因此不会和 `menu.back`（Esc）冲突。

控件层只接受可打印 ASCII；非 ASCII 键盘输入会被丢弃（引擎没有对应字形，静默丢字形比拒绝按键更糟糕）。需要中文名字时另开字形方案，不要在控件层塞 UTF-8 渲染。

## 5. 加一个新屏

1. 声明 screen：设置 `title/hint`，按顺序填 widgets，给需要回报动作的行写 `id`。
2. 打开屏幕时 `UiController::ResetFocus(screen)`。
3. 每帧：`const UiAction a = ui_.Update(screen, rawInput_);` 然后 `ui_.Draw(screen, renderer_);`
4. 按 `a.id` 分派（`switch (IdOf(a.id))`，不要字符串 if 链），`a.kind` 决定动作类别。
5. 门禁：`--ui-selftest` 覆盖焦点跳过静态行、Activate、Value/Digits 调节、文本输入、Back、无窗口绘制。新控件种类要同时在 selftest 里加断言。

## 6. 明确不做

- **鼠标点击/悬停**：当前全部界面都是键盘导航；加鼠标要先定义 hit-test 与焦点抢占规则，属于独立设计。
- **滚动列表**：房间列表按当前规模（LAN 1–8 个房间）一屏够用；需要分页时按页切片重建 widgets，不做滚动视口。
- **布局引擎/主题系统**：行高与列宽是 screen 上的几个标量，够用；主题只保留一套配色常量。
- **即时模式 API**：控件层是持有式数据，不是 IMGUI。理由见 §3 规则 1。
