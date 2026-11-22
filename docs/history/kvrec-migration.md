> **已归档**（2026-11-22）：迁移已落地，本文件只作过程与决策记录。  
> 现行数据布局见 [../architecture.md](../architecture.md) §6。

# 地图与数据存储迁移到 KVRec — 改动方案

状态：**已实现**（`.rushmap` / `.rushconf`；业务代码已切换）  
目标读者：引擎分析 / 实现阶段的执行者  
关联仓库：[neuralnet-cpp-dev/kvrec](https://github.com/neuralnet-cpp-dev/kvrec)

---

## 1. 背景与目标

当前数据落盘方式：

| 数据 | 现状 | 问题 |
|------|------|------|
| 地图 | `assets/maps/<name>/{track,terrain,decor,zones}.csv` 多文件 CSV | 无 schema、无版本、解析分散在游戏层 |
| 配置 | `config.ini` 分区 INI | 仅字符串、类型弱、与 KVRec 生态不一致 |
| 地图生成 | `tools/make_test_track.py` 写出 CSV | 运行时仍读 CSV，工具链与运行时格式脱节 |

目标：

1. **运行时**地图与配置全部改为 **KVRec** 二进制记录；**落盘扩展名**分别为 `.rushmap` / `.rushconf`（库仍叫 kvrec，只改文件后缀）。
2. **分层**严格：L3 游戏 → L2 引擎 → L1（exSTL、两个物理库、KVRec）。
3. **游戏不直接**依赖 KVRec / exSTL；只经引擎封装 API。
4. **扁平化 L1**：`exstl`、`kvrec`、`qu3e`、`nudge` 同级放在同一 `third_party` 目录，**不**再嵌套第二份 exSTL。
5. **地图转换工具**：把 Python 生成的（或手工维护的）CSV 地图编成 `.rushmap`（内容仍是 KVRec 字节）。
6. **CMake 预设**：可分别构建「仅游戏」与「游戏 + 工具」；**Release 不构建工具**。

**命名约定（已定）：**

| 用途 | 扩展名 | 内容格式 |
|------|--------|----------|
| 地图 | `.rushmap` | KVRec v2 |
| 配置 | `.rushconf`（如 `config.rushconf`） | KVRec v2 |
| 库 / CMake 目标 | 仍称 `kvrec` / `kvrec::kvrec` | 不改库名 |

---

## 2. 分层与依赖（必须遵守）

```
L3  apex-rush (游戏)
      │  仅 import rush.engine.* / 链接 rush::engine
      ▼
L2  rush-engine
      │  对外：地图读、配置读/写、路径、数学、物理、渲染…
      │  对内：唯一允许 #include <kvrec/...>、exstl 的实现层
      ▼
L1  扁平兄弟（同一文件夹，无相互嵌套）
      exstl::exstl
      kvrec::kvrec          ← 只消费已有 exstl::exstl，不再 add 自己的 exstl
      qu3e
      nudge
```

规则：

- **禁止**游戏代码 `#include <kvrec/...>` 或 `#include <exstl/...>`。
- **禁止**在 `third_party/kvrec` 内再 vendor / `add_subdirectory` 一份 exSTL。
- L1 四个库由 L2 的 `rush-engine/CMakeLists.txt` **按目标名**链接；目录平铺，**不**表达「先有 kvrec 再有 exstl」的目录层级依赖。CMake 里仍需在 link 之前保证 `exstl::exstl` 目标已存在（先 `add_subdirectory(exstl)` 再 `add_subdirectory(kvrec)` 仅是目标创建顺序，不是模块嵌套）。

---

## 3. 目录布局（目标态）

```
Apex Rush Online/
├── CMakeLists.txt                 # 游戏 + 可选 tools
├── CMakePresets.json              # release / debug / tools 相关预设
├── config.ini → 迁移后废弃，改 config.rushconf（见 §5.2）
├── assets/maps/
│   └── test_circuit.rushmap       # 单文件地图（由 mapconv 生成）
│       # 或保留源 CSV 目录作为「作者源」，见 §6
├── tools/
│   ├── make_test_track.py         # 仍可生成 CSV 作者源（不直接写 KVRec）
│   └── mapconv/                   # 新：CSV 目录 → .rushmap
│       ├── CMakeLists.txt
│       └── main.cpp
└── rush-engine/
    ├── CMakeLists.txt             # + add_subdirectory(third_party/kvrec)
    ├── modules/core.cppm          # MapData / Config 对外 API（不暴露 kvrec 类型）
    ├── src/core/
    │   ├── config_impl.cpp        # Config 落盘改为 KVRec
    │   ├── map_io.cpp             # MapData ↔ KVRec 序列化
    │   └── map_csv.cpp            # 仅工具/迁移用 CSV 读入（可选，见 §6）
    └── third_party/               # L1 扁平
        ├── exstl/
        ├── kvrec/                 # 已拷入（见 §9 前置）
        ├── qu3e/
        └── nudge/
```

**已发生的非业务改动（预研阶段）**：`.deps/kvrec` 已克隆，并拷贝到 `rush-engine/third_party/kvrec`（含 `include/`、`CMakeLists.txt`、`tests/`）。业务源码、游戏逻辑、现有 CSV/INI **均未修改**。

---

## 4. 对外 API（引擎封装面）

游戏只接触下列类型/函数（仍在 `rush.engine.core` 或后续 `rush.engine.storage` 模块；建议先留在 `core`，避免模块碎片化）。

### 4.1 地图

```cpp
namespace apex {

struct MapData {
  struct WayPoint { Vec3 position; float width, grassLeft, gravelLeft, grassRight, gravelRight; };
  struct Decor    { int type; Vec3 position; float scale; };
  struct DrsZone  { float start, end; };

  std::string name;
  std::vector<WayPoint> centerline;
  std::vector<Decor> decor;
  std::vector<DrsZone> drsZones;
  std::vector<float> gridX, gridZ, gridY;
  bool ok = false;

  float TerrainHeight(float x, float z) const;
  float MinTerrainY() const;
};

// 读取 mapsRoot 下 name 对应的 KVRec 地图文件
MapData LoadMapData(const std::string& mapsRoot, const std::string& name);

// 列出可用地图（*.rushmap 基名）
std::vector<std::string> ListMaps(const std::string& mapsRoot);

// 写入（供 mapconv / 编辑器；游戏运行时通常只读）
bool SaveMapData(const std::string& mapsRoot, const std::string& name, const MapData& map);

}  // namespace apex
```

文件约定（二选一，推荐 A）：

- **A（推荐）**：`assets/maps/<name>.rushmap` 单文件；`ListMaps` 扫描 `*.rushmap`。
- B：`assets/maps/<name>/map.rushmap`，目录仍可放缩略图等旁路资源。

`Game.cpp` 中 `LoadMapData(DataPath("assets/maps"), mapName_)` 调用点 **签名不变**，仅语义从 CSV 目录变为 KVRec。

### 4.2 配置

```cpp
class Config {
public:
  bool Load(const std::string& path);   // path = DataPath("config.rushconf")
  bool Save(const std::string& path) const;

  std::string GetString(section, key, fallback) const;
  float GetFloat(...) const;
  int   GetInt(...) const;
  bool  GetBool(...) const;
  void  SetString(section, key, value);

private:
  // 内存模型可仍为 section→key→string（与现 INI 语义一致）
  // 落盘时编码为 KVRec 嵌套 Record
};
```

游戏侧 API **不变**；仅路径常量从 `config.ini` 改为 `config.rushconf`（见 §5.2）。

### 4.3 明确不导出的内容

| 不导出 | 原因 |
|--------|------|
| `kvrec::KeyValueRecord` | 游戏不得依赖 KVRec |
| `exstl::Result` 等 | 仅引擎内部 / 已有 `rush::Result` 别名处使用 |
| CSV 解析细节 | 迁移与工具内部 |

---

## 5. KVRec 数据格式

KVRec 能力边界（来自头文件）：

- 类型：`UInt`（u64）、`Str`、`UIntArray`（`vector<u64>`）、嵌套 `Record`（**仅 v2**）。
- 运行时读写地图/配置一律用 **`serialize_v2()` + `parse()`**（支持嵌套、绝对偏移自描述）。
- **`kvrec_version` 为保留键**，业务字段不得占用。
- float **不能**原生存储 → 统一 `bit_cast` 到 u32，再零扩展进 `UInt` / `UIntArray` 的 u64。

### 5.1 地图 `*.rushmap`（内容为 KVRec v2）

```
root: Record
├── format_version : UInt = 1
├── name           : Str
├── centerline     : Record
│   ├── count            : UInt
│   ├── x, y, z          : UIntArray   // float bits, 长度 = count
│   ├── width            : UIntArray
│   ├── grass_left       : UIntArray
│   ├── gravel_left      : UIntArray
│   ├── grass_right      : UIntArray
│   └── gravel_right     : UIntArray
├── terrain        : Record
│   ├── grid_x, grid_z   : UIntArray
│   └── grid_y           : UIntArray   // row-major, 与现 gridY 一致
├── decor          : Record
│   ├── type             : UIntArray
│   ├── x, y, z          : UIntArray
│   └── scale            : UIntArray
└── zones          : Record            // 可空
    ├── drs_start        : UIntArray   // lap fraction 0..1
    └── drs_end          : UIntArray
```

约定：

- 坐标轴与 CSV 相同：作者文件为 `(x,z,y)`，进 `MapData` 后为 `Vec3{x,y,z}`（与现 `MapData.cpp` 轴交换一致）。
- `terrain` / `decor` / `zones` 可缺省或长度为 0；`centerline.count >= 3` 才 `ok = true`。
- 可选顶层 `schema` : Str = `"apex.map"`，便于工具校验（非必须）。

### 5.2 配置 `config.rushconf`（内容为 KVRec v2）

```
root: Record
├── format_version : UInt = 1
├── assists : Record { esc: Str, steer: Str, abs: Str, tc: Str, ... }
├── video   : Record { width: Str, height: Str, ... }
├── physics : Record { hz: Str, engine: Str, ... }
├── audio   : Record { ... }
├── race    : Record { map: Str, laps: Str, ... }
└── season  : Record { ... }
```

- 值仍存 **Str**（与现有 `SetString` / INI 一致），避免游戏层类型分叉；数值转换继续走 `GetFloat/GetInt`。
- 迁移：首次若存在旧 `config.ini` 而无 `config.rushconf`，由一次性逻辑读 INI → 写 KVRec（可放在 `Config::Load` 分支，或单独 `tools/config_migrate`）。分析阶段可先 **只读 KVRec、不做自动迁移**，用预设生成默认配置。

### 5.3 浮点编码（实现约定）

```cpp
inline std::uint64_t FloatBits(float f) {
  std::uint32_t b;
  std::memcpy(&b, &f, 4);
  return b;
}
inline float BitsFloat(std::uint64_t v) {
  std::uint32_t b = static_cast<std::uint32_t>(v);
  float f;
  std::memcpy(&f, &b, 4);
  return f;
}
```

仅存在于 `map_io.cpp` / 工具内部，不进公开头文件语义。

---

## 6. 地图转换工具 `mapconv`

### 6.1 职责

```
Python / 手工 CSV 作者源  ──mapconv──►  assets/maps/<name>.rushmap  ──运行时──►  LoadMapData
```

- **输入**：现有四件套目录  
  `track.csv`（必须）、`terrain.csv`、`decor.csv`、`zones.csv`（可选）。
- **输出**：单个 `<name>.rushmap`（内容为 KVRec 字节）。
- **不做**：运行时热重载、网络同步（可后续加）。

### 6.2 调用形态

```text
mapconv --in assets/maps/test_circuit --out assets/maps/test_circuit.rushmap
# 或
mapconv <in_dir> <out.rushmap>
```

### 6.3 实现归属

| 方案 | 说明 | 建议 |
|------|------|------|
| **工具链引擎库** | `mapconv` 链接 `rush::engine`，复用 `map_csv` + `SaveMapData` | **推荐**：格式只维护一份 |
| 工具独立解析 | 工具内再写一套 CSV→KVRec | 易与引擎 schema 漂移，不推荐 |

`map_csv.cpp` 可标成引擎内 **tool-only** 源文件，或放在 `tools/mapconv` 但 **序列化必须调用** 引擎导出的 `SaveMapData`。

### 6.4 与 Python 的分工

- `make_test_track.py`：**继续生成 CSV 作者源**（便于 diff / 手改）。
- 生成 CSV 后执行一次 `mapconv`（手动或 CMake `add_custom_command`，仅 tools 开启时挂钩）。
- 分析阶段建议：**仓库同时保留 CSV 源 + 已提交的 `.rushmap`**，保证 Release（不建 tools）也能直接跑游戏。

---

## 7. CMake 与预设

### 7.1 选项

```cmake
# 根 CMakeLists.txt
option(APEX_BUILD_TOOLS "Build developer tools (mapconv)" OFF)
# Debug 预设里显式 ON；Release 预设 OFF 或不设置（默认 OFF）
```

- `APEX_BUILD_TOOLS=ON` → `add_subdirectory(tools/mapconv)`。
- Release **不**定义/不构建 `mapconv`（满足「release 不构建工具」）。

### 7.2 预设（CMakePresets.json 草案）

| 预设 | 用途 | `APEX_BUILD_TOOLS` |
|------|------|-------------------|
| `release` | 发布 / CI 主路径，只游戏 + 测试 | **OFF** |
| `debug` | 日常调试，只游戏 | OFF（或沿用现状） |
| `debug-tools`（或 `dev`） | **游戏 + mapconv** | **ON** |
| `web` | Emscripten（不建工具） | OFF |

示例 cache 片段：

```json
{
  "name": "debug-tools",
  "displayName": "Clang Debug + Tools",
  "inherits": "debug",
  "cacheVariables": { "APEX_BUILD_TOOLS": "ON" }
}
```

buildPresets 对应增加 `debug-tools`；`release` 的 build/test 预设保持只构建默认目标。

### 7.3 引擎 CMake 增量

```cmake
# rush-engine/CMakeLists.txt
add_subdirectory(third_party/exstl)   # 先创建 exstl::exstl
add_subdirectory(third_party/kvrec)   # 扁平兄弟；内部 if(NOT TARGET exstl::exstl) 不会再拉一份
add_subdirectory(third_party/qu3e)
# nudge 原有 ...

target_link_libraries(rush-engine PUBLIC exstl::exstl qu3e nudge kvrec::kvrec)
```

`kvrec/CMakeLists.txt` 已具备：主仓已存在 `exstl::exstl` 时 **不会** 再 `add_subdirectory(../exstl)`。无需改 kvrec 源码即可满足「不重复包含 exSTL」。

### 7.4 游戏 CMake

- `apex-rush` **只** `target_link_libraries(... rush::engine)`，不直接链 `kvrec::kvrec`。
- 删除或不再编译 `src/game/MapData.cpp`（逻辑迁入引擎后）。

---

## 8. 代码改动清单（实现阶段按序执行）

> 本节是执行 checklist；**当前分析阶段不执行**。

| # | 层级 | 改动 | 涉及文件（预期） |
|---|------|------|------------------|
| 1 | L1 | 确认 kvrec 扁平 vendoring，CMake 接入且不双份 exSTL | `rush-engine/CMakeLists.txt`, `third_party/kvrec/**` |
| 2 | L2 | `MapData` 定义迁到引擎模块（结构体与现游戏侧一致） | `modules/core.cppm`（或新 `storage.cppm`） |
| 3 | L2 | 实现 KVRec ↔ MapData | `src/core/map_io.cpp` |
| 4 | L2 | `Config::Load/Save` 改 KVRec；内存 API 不变 | `src/core/config_impl.cpp` |
| 5 | L2 | `DataRoot` 注释与默认配置路径改为 `config.rushconf` | `core.cppm`, `paths_impl.cpp` |
| 6 | L2 | `ListMaps` / `LoadMapData` 改为 `*.rushmap` | `map_io.cpp` |
| 7 | L3 | 删除游戏内 CSV 解析；改路径与 include | `src/game/MapData.*`（删）、`Game.cpp`、`main.cpp`、`Track.h` |
| 8 | Tools | 新增 `mapconv` | `tools/mapconv/**` |
| 9 | Build | `APEX_BUILD_TOOLS` + 预设 | 根 `CMakeLists.txt`, `CMakePresets.json` |
| 10 | Assets | 生成并提交 `test_circuit.rushmap`；INI→rushconf 配置样例 | `assets/maps/`, `.gitignore` |
| 11 | Docs | 本方案落地后标为 Implemented | 本文档 |

**游戏层允许的最终依赖形态：**

```cpp
// Game / main / Track
import rush.engine.core;   // MapData, LoadMapData, Config, DataPath
// 禁止：#include <kvrec/kvrecord.hpp>
// 禁止：#include <exstl/...>
```

---

## 9. 迁移与兼容策略

1. **地图**  
   - 分析期：CSV 仍是作者源。  
   - 实现期：`mapconv` 产出 `.rushmap` 并纳入版本库。  
   - 运行时：只读 `.rushmap`；缺文件则 `ok=false`（与现缺 `track.csv` 行为一致）。

2. **配置**  
   - 新文件名：`config.rushconf`。  
   - `.gitignore`：增加 `config.rushconf`（与现 `config.ini` 对齐），避免提交本机设置。  
   - 可选：Load 时检测残留 `config.ini` 并迁移一次（实现阶段决定）。

3. **Web / Emscripten**  
   - `--preload-file .../assets/maps` 会带上 `.rushmap`，无需改 preload 路径结构（若采用方案 A 单文件）。

4. **测试**  
   - `drive-test` / 设置 QA 依赖 `Config` 与地图加载：API 不变则测试入口可不动。  
   - 增加一条「mapconv 输出可被 LoadMapData 读回」的工具自检（仅 tools 预设）。

---

## 10. 风险与决策点（分析阶段需拍板）

| # | 决策点 | 选项 | 方案倾向 |
|---|--------|------|----------|
| D1 | 地图文件布局 | A `name.rushmap` / B `name/map.rushmap` | **A**（简单、ListMaps 直接） |
| D2 | `MapData` 放哪个模块 | `rush.engine.core` / 新 `rush.engine.storage` | **先 core**，避免一次拆太多 |
| D3 | 配置旧 INI 自动迁移 | 要 / 不要 | 分析期 **不要**；实现期可选 |
| D4 | CSV 是否继续进 CI | 仅 rushmap / 双格式 | **双格式提交**，Release 不依赖 tools |
| D5 | mapconv 链接方式 | 全量 `rush::engine` / 抽 `rush::storage` 小库 | 分析期 **全量 engine** 足够 |
| D6 | 预设命名 | `debug-tools` / `dev` | **`debug-tools`**，语义直白 |

---

## 11. 验收标准（实现完成后）

- [x] 游戏目标未链接、未 include KVRec/exSTL 头（仅引擎 TU 可见；`kvrec::kvrec` 为 PRIVATE）。
- [x] `rush-engine/third_party` 下仅一份 exSTL；kvrec 不二次拉取。
- [x] `cmake --preset release` 成功且 **无** `mapconv` 目标（`APEX_BUILD_TOOLS=OFF`，独立 `build/`）。
- [x] `cmake --preset debug-tools` 可构建 `mapconv`（`build-tools/`，与 release 分离）。
- [x] `mapconv` 转换 `test_circuit` 后，游戏能加载并跑通 `drive-test`（qu3e 0 failures，含 DRS zone）。
- [x] 设置界面 Save/Load 走 `config.rushconf`（测试中已写出）。
- [x] `ListMaps` 能列出 `*.rushmap` 地图。

---

## 12. 实现摘要（2026-11-22）

| 项 | 结果 |
|----|------|
| L1 扁平 | `third_party/{exstl,kvrec,qu3e,nudge}` |
| 引擎 API | `MapData` / `LoadMapData` / `ListMaps` / `SaveMap*` / `Config` 在 `rush.engine.core` |
| 落盘 | 地图 `*.rushmap`，配置 `config.rushconf`（皆 KVRec v2） |
| 工具 | `tools/mapconv`：`mapconv <csv_dir> <out.rushmap>` |
| 预设 | `release` 不建 tools；`debug-tools` → `build-tools/` 含 mapconv |
| 验证 | `drive-test[qu3e]: OK (0 failures)` |
