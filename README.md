# Apex Rush Online

Windows / Linux 局域网多人 F1 风格竞速游戏。纯本地、零外部运行时依赖、双击即玩；主机权威联机，客户端经 UDP 自动发现并用房间码加入。

**技术栈**：C++23 · Raylib · ENet · 自写 4 轮射线悬挂 Simcade 车辆动力学 · 可插拔碰撞后端（qu3e / nudge / RPC）· Rush Engine（异步运行时）· KVRec 落盘。

---

## 构建

前置：CMake ≥ 3.28 + Ninja + 支持 C++23 named modules 的 Clang（Web 另需 emsdk）。

```powershell
# 游戏（Release，不构建工具）
cmake --preset release
cmake --build --preset release

# 发布/外发用（CRT 静态链接：目标机不需要 VC++ 运行库）
cmake --preset release-static
cmake --build --preset release-static

# 开发工具（mapconv：CSV → .rushmap）
cmake --preset debug-tools
cmake --build --preset debug-tools

# 门禁 / 基准
build\apex-rush.exe --drive-test    # 三物理后端 0 failures
build\apex-rush.exe --bench 100     # 百车分段计时

# Web / WASM（先 source emsdk_env）
cmake --preset web
cmake --build --preset web

# 单文件离线 HTML（SINGLE_FILE + 内嵌 data/FileSaver → dist/apex-rush-single.html）
cmake --preset web-single
cmake --build --preset web-single
```

### Linux

桌面版在 Linux 上同样可构建，无 PowerShell 依赖（raylib 的补丁由 CMake 脚本
`cmake/patch_raylib.cmake` 完成，Windows / Linux 共用）。

```bash
sudo apt install clang lld ninja-build cmake git pkg-config \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libgl1-mesa-dev                 # Vulkan 后端另加 libvulkan-dev

CC=clang CXX=clang++ cmake --preset release-linux
cmake --build --preset release-linux     # → build-linux/apex-rush
ctest --preset release-linux             # rpc-* 物理测试，无需显示器
./build-linux/apex-rush --net-selftest   # 无头网络自检
```

- `linux` presets 不写死编译器，取 `CC` / `CXX`（缺省即系统 `cc` / `c++`）；
  装了多版本 clang 时用 `CC=clang-24 CXX=clang++-24`。
- ThinLTO 需要 lld。配置期自动探测（含 Debian/Ubuntu 只有带版本号
  `ld.lld-20`、没有 `lld` 软链的情况）；找不到就只用 `-O3`，不报错。
  `-DAPEX_THIN_LTO=OFF` 可强制关闭。
- raylib 内嵌的 GLFW 默认只编 X11 后端（Wayland 会话走 XWayland）；需要原生
  Wayland 时加 `-DAPEX_GLFW_WAYLAND=ON`（另需 wayland/xkbcommon 开发包）。
- `--drive-test` 会开窗口，Linux 上需要 X11/Wayland 显示，无头机器用
  `xvfb-run ./build-linux/apex-rush --drive-test`。`rpc-*` 测试与
  `--net-selftest` 不需要。
- `web-single` 的最后一步打包仍用 PowerShell（要把多 MB 的 `.data` 做 base64），
  该 preset 需要 `pwsh`；普通桌面构建不需要。

### CPU 指令集（x86-64 / AVX2）

默认目标是 **x86-64 基线（SSE2）**：任何 x86-64 CPU 都能跑，包括支持 AVX2 的机器。

```bash
cmake --preset release-avx2         # Windows
cmake --preset release-linux-avx2   # Linux
```

`-DAPEX_NATIVE_ARCH=ON`（即上面两个 preset）把基线抬到 **AVX2 + FMA**：
clang/gcc 用 `-march=x86-64-v3`，MSVC 用 `/arch:AVX2`。它**不会**用
`-march=native`，所以产物仍然是可分发的——只要 CPU 支持 AVX2（Intel Haswell
2013+ / AMD Excavator 2015+）就能跑，不需要和构建机同款 CPU。代码里没有任何
AVX-512 路径，v3 就是硬上限。

注意该选项会让二进制在 **不支持 AVX2 的老 CPU 上直接崩**（非法指令），对外分发
用默认的 `release` / `release-linux`。非 x86-64 架构上该选项会被忽略并给出警告。

实测（`objdump`，x86-64 Linux，clang 24）：基线产物 `ymm` 引用 0 条；avx2 preset
产物 10k+ 条 `ymm` / 1.7k 条 `vfmadd`、**0** 条 `zmm`（即确实没有越过 AVX2）。

产物：`build/apex-rush.exe`（`release-static` 出到 `build-static/apex-rush.exe`，只依赖 Windows 自带 DLL；Linux 出到 `build-linux/apex-rush`）。配置与本机设置写在 EXE 同级 `config.rushconf`（KVRec；`config.ini` 已废弃）。Web 多文件在 `build-web/`；单文件离线包在 `dist/apex-rush-single.html`（构建后自动调用 `tools/pack-single-html.ps1`）。

---

## 目录结构

```
Apex Rush Online/
├── CMakeLists.txt / CMakePresets.json
├── config.rushconf          # 本机配置（gitignore）
├── assets/
│   ├── maps/                # 作者源 CSV + *.rushmap（mapconv 产物）
│   └── models/              # 低模车辆等资源
├── src/                     # L3 游戏层
│   ├── main.cpp             # CLI 入口（--bench / --drive-test / 正常菜单）
│   ├── game/                # Game、Track、比赛循环
│   ├── audio/               # miniaudio 封装
│   └── ...
├── rush-engine/             # L2 引擎子模块（C++20 named modules）
│   ├── modules/             # rush.engine.{core,physics,render,input,events,job,runtime}
│   ├── src/                 # 实现（含 IPhysicsWorld 三后端适配、input 线程）
│   └── third_party/         # L1 扁平：exstl · kvrec · qu3e · nudge · rush-physics-core
├── third_party/enet/        # 局域网可靠 UDP
├── tools/                   # mapconv、赛道/模型生成脚本
└── docs/                    # 活动文档（见下）
```

分层硬约束：

| 层 | 职责 | 禁止 |
|----|------|------|
| L3 游戏 | 玩法、UI、网络会话 | 不 include KVRec/exSTL/rpc.h；只 `import rush.engine.*` |
| L2 引擎 | 地图/配置、物理编排、渲染、输入事件、Job、仿真线程 | 车体只在此层；错误用 `rush::Result` |
| L1 物理/存储 | 碰撞内核、KVRec、exSTL | 无车体下沉；无异常控制业务流；符号 `rpc*` |

详见 [docs/architecture.md](docs/architecture.md)。

---

## 运行参数（摘要）

| 参数 | 说明 |
|------|------|
| （无） | 正常启动菜单 |
| `--drive-test` | 无头回归：三后端 flat 稳定性 + 赛道，失败则非 0 退出 |
| `--bench [cars]` | 分段计时（默认 100 车），对照异步引擎性能 |
| `--jitter-probe` | 已摘除（历史调试）；当前走快照速度外推渲染 |

物理后端由配置 `[physics] engine = qu3e \| nudge \| rpc` 切换（默认 qu3e）。

---

## 文档索引

| 文档 | 状态 | 内容 |
|------|------|------|
| [docs/architecture.md](docs/architecture.md) | 现行 | 系统分层、模块、异步运行时、三物理后端 |
| [docs/input.md](docs/input.md) | 已实施 | 统一输入：input 线程、注册表、纯事件总线（无回调） |
| [docs/rpc-refactor.md](docs/rpc-refactor.md) | P0+P1 已实施；P2 设计稿 | Rush Physics Core 重构与性能 |
| [docs/rpc-optimization.md](docs/rpc-optimization.md) | 现行方案 | RPC 内核深度优化：算法 / 硬件 / 现代 C++ 三层清单与路线图 |
| [docs/perf-async-engine.md](docs/perf-async-engine.md) | 现行报告 | 百车同步 → 全异步引擎对比 |
| [docs/history/](docs/history/) | 归档 | 原始设计、KVRec 迁移方案（已完成） |

历史文档不再作为实现依据；行为以代码与上表现行文档为准。

---

## 许可与资源

- 车辆模型等资源许可见 `assets/models/racecar/License.txt`。
- 第三方库各自保留许可证（见各 `third_party` / `rush-engine/third_party` 下 LICENSE）。
