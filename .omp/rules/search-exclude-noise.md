---
name: search-exclude-noise
description: 仓库内搜索先收窄 path。.vscode/.gitnexus/build/install/log/.codegraph 已被 gitignore（内置工具自动跳过），但 src/*/lib 受版本跟踪不会被跳过
condition: '(?i)\b(?:grep|rg|fd|fdfind)\b'
scope: [tool]
interruptMode: never
repeatMode: once
---
**搜索卫生**：先问「这个 pattern 可能出现在哪个包」，再决定 `path`。

**内置工具自动跳过（已被 .gitignore 覆盖）**：

| 目录 | 体积 | 内容 |
|---|---|---|
| `.vscode/` | 2.9G | VSCode 索引库 |
| `build/` `install/` | 232M / 182M | colcon 构建与安装产物 |
| `.gitnexus/` | 40M | 代码图谱索引 |
| `.codegraph/` | 7.0M | 图库 |
| `log/` | 3.8M | 构建日志 |

**不受 gitignore 保护（受版本跟踪，全仓 grep 照样会搜到并命中）**：

| 目录 | 体积 | 内容 |
|---|---|---|
| `src/tl_teleop/lib/` | 98M | `libPXREARobotSDK.so`（arm / x86 各一份） |
| `src/tl_driver/lib/` | 9.7M | `_tl_host.so`、`arm/tl_interface.py`（52K）、`x86/tl_interface.py`（228K）等 ctypes 绑定文本、`__pycache__/*.pyc` |
| `src/tl_driver/lib/include/`、`src/tl_teleop/lib/include/` | — | SDK 头文件（纯文本，值得直接读） |

- **搜文本** → 内置 `grep`（默认 `gitignore=true`，上表第一批自动跳过）：`path` 传具体目录（`src/tl_driver/src`、`src/tl_teleop_f710/include`、`src/tl_ros2_interface/msg`、`src/tl_driver/doc`），**不要传 `.`、`/` 或通配**；命中过多用 `skip` 分页。
- **找文件** → 内置 `glob`（`src/**/CMakeLists.txt`、`src/**/*.launch.py`、`**/package.xml`）。
- **查 SDK 接口** → 直接 `read src/tl_driver/lib/include/cpp/interface/tl_interface.h`（1170 行，按区间读），不要在 `.so` 上搜。
- **shell 侧**：`grep -r`、`find` 不读 `.gitignore`（全局 `bashInterceptor` 已拦截这些形态并指向内置工具）。确需 shell 时用 `git grep`（仅搜索跟踪文件），或显式 `--exclude-dir={.vscode,.codegraph,.gitnexus,build,install,log,__pycache__}`；`cmd | grep ...` 这类**仅筛管道输出**的用法不受限（无路径参数时不要带 `-r`）。

**代价量级**：一次未限定范围的全仓递归 grep 命中二进制或索引文件，会把大段无意义输出写进上下文；工具输出在后续每次请求都会重发，直到压缩。
