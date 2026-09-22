---
name: changelog-user-visible
description: CHANGELOG.md 只记录相对最后发布版本的用户可见净变更，不写构建系统/工具与 agent 配置/内部文档/目录重命名/代码风格
condition: 'CHANGELOG'
globs: ['**/CHANGELOG.md']
scope: [tool:edit(CHANGELOG.md), tool:write(CHANGELOG.md), tool:edit(**/CHANGELOG.md), tool:write(**/CHANGELOG.md)]
interruptMode: tool-only
repeatMode: once
---
**`CHANGELOG.md` 是写给使用者看的账本，不是内部开发日志。**每条目必须能回答「使用者感知到什么变化」。

**必须写（用户可见）：**

| 类别 | 示例 |
|---|---|
| 新增功能 | 新接口、新臂型支持、新功能包、新示例 |
| Bug 修复 | 哪个服务/话题/启动方式原来不工作，现在修好了 |
| 接口变更 | msg/srv/话题/服务的参数、返回值、名称变化；废弃与删除 |
| 启动与配置变更 | launch 文件增删改名、YAML 参数默认值变化 |
| 依赖变更 | 升级 SDK、替换第三方库、ROS2 发行版要求变化 |
| 性能/体验改进 | 控制循环实时性、上电时序、启动耗时 |

**不要写（纯内部）：**

- 构建系统内部逻辑（`CMakeLists.txt` 的写法调整、目标/依赖重构）
- 工具与配置（`.gitignore`、`.clang-format`、`pyproject.toml`、pre-commit hook、**CI 工作流 `.github/workflows/**` 的接入/迁移/守卫调整**）
- agent/harness 配置与规则（`.omp/**`、`.agents/**`、`AGENTS.md` 自身）
- 目录重命名、文件移动、代码搬家
- 内部文档与稿件（架构评审稿、说明文档勘误中的纯笔误、草稿）
- 测试内部（lint 配置调整、手工脚本改动、测试代码重构）
- 代码风格（变量重命名、include 补齐、注释润色、格式化）
- 发布流程内部动作（合并提交、回退某次提交——这类只写进文末「备注」；`package.xml` 的版本号/维护者等元数据整理也不写，**许可证变更除外**，它影响用户能否再分发）

**基线：相对最后一次发布版本（tag）的净变更，不是相对上一个提交。**

- 写条目前先对照发布 tag 验证：`git grep <符号> <发布tag> -- src/`。标签中已存在的接口不得写「新增」。
- 开发过程中「改了又改回去」「先移除后重新提供」「迁移引入又修复的回归」，相对发布基线净变化为零，**不写**。
- 旧名从未出现在任何发布版本中的「更名/迁移」不成立，直接按最终形态描述。
- 同一接口在多个小节重复出现或前后矛盾时，核对 tag 后合并为一条。
- 发版时把 `[Unreleased]` 冻结进版本号章节，下个周期从空开始；不得把已归档版本的内容当本周期变更重写。

**本仓库格式**（与 `AGENTS.md` 的 CHANGELOG 规范一致，勿自行改格式）：

- 未发布变更写在 `## [Unreleased]` 下，按日期（`### YYYY-MM-DD`，新 → 旧）分组；已发布章节只按类型分组。
- 分类枚举固定为 新增 / 修复 / 变更 / 移除 / 文档 / 工程。
- 行尾短哈希必须真实存在于本分支历史（`git log --oneline | grep <hash>`）；无提交就留空，禁止编造。
- 「工程」类只写**用户可感知**的工程变更（依赖升级、构建链变化、SDK 版本要求）；`.gitignore`、格式化配置、agent 规则这类内部调整不写。
