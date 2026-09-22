#!/usr/bin/env bash
#
# 变更日志章节提取脚本
# 从 CHANGELOG.md 抽取指定版本章节，作为 GitHub Release 正文
# （供 .github/workflows/release.yml 调用，也可本地预览）
#
# 用法:
#   ./scripts/release-notes.sh V2.0.1                    # 抽取 ## [2.0.1] 章节
#   ./scripts/release-notes.sh v2.0.1-rc1                # 先找 2.0.1-rc1，无则回退 2.0.1
#   ./scripts/release-notes.sh V2.0.1 path/to/CHANGELOG.md
#
# 输出（stdout）: 章节正文
#   - 去掉 "## [x.y.z] - 日期" 标题行（标题由 Release 标签体现）与首尾空行
#   - 到下一个 "## [" 标题、或备注区（以 **备注** 开头的行）为止；章末 "---" 分隔线一并去掉
#   - 章节内围栏代码块（``` / ~~~）原样保留，块内的 "## ["、"---"、"**备注**" 不当作边界
# 退出码: 0 成功；1 未找到该版本章节或章节为空
#         （发版前须先把 [Unreleased] 的内容合并进版本号章节，见 AGENTS.md）
#

set -euo pipefail

PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || dirname "$(dirname "$0")")

TAG="${1:-}"
CHANGELOG="${2:-${PROJECT_ROOT}/CHANGELOG.md}"

if [ -z "$TAG" ]; then
    echo "用法: $0 <标签> [变更日志路径]" >&2
    exit 1
fi

if [ ! -f "$CHANGELOG" ]; then
    echo "错误: 找不到变更日志文件 ${CHANGELOG}" >&2
    exit 1
fi

# 标签 → 章节版本号: 去掉 V/v 前缀（V2.0.1 → 2.0.1）
VERSION="${TAG#[vV]}"

# ── 抽取指定版本章节正文 ──────────────────────
# 版本号用字面量前缀比较（index 而非正则），避免 . 被当作通配符；
# 围栏代码块（``` / ~~~）内不识别章节边界与备注区，防止正文被误截断
extract_section() {
    local ver="$1"
    awk -v needle="## [${ver}]" '
        collecting && /^[[:space:]]*(```|~~~)/ { fence = !fence }
        !collecting && index($0, needle) == 1 { collecting = 1; next }
        # 章节边界: 下一个 "## [" 标题；备注区: 以 **备注** 开头的行（均在围栏外识别）
        collecting && !fence && index($0, "## [") == 1 { exit }
        collecting && !fence && $0 ~ /^[[:space:]]*\*\*备注\*\*/ { exit }
        collecting { n++; line[n] = $0 }
        END {
            # 去掉尾部空行与章末分隔线
            while (n > 0 && (line[n] ~ /^[[:space:]]*$/ || line[n] ~ /^-+[[:space:]]*$/)) n--
            # 去掉首部空行与占位符（“（暂无）”视为无内容）
            for (i = 1; i <= n; i++) {
                if (!printed && (line[i] ~ /^[[:space:]]*$/ || index(line[i], "（暂无）") > 0)) continue
                print line[i]
                printed = 1
            }
        }
    ' "$CHANGELOG"
}

NOTES=$(extract_section "${VERSION}")

# 预发布标签（2.0.1-rc1）在变更日志里通常只记基础版本章节 → 回退
if [ -z "$NOTES" ] && [[ "$VERSION" == *-* ]]; then
    VERSION="${VERSION%%-*}"
    NOTES=$(extract_section "${VERSION}")
fi

if [ -z "$NOTES" ]; then
    echo "错误: ${CHANGELOG} 中找不到版本 ${TAG}（章节 ## [${VERSION}]）的正文，或该章节为空" >&2
    echo "      发版前请先把 [Unreleased] 的内容合并进 \"## [${VERSION}] - YYYY-MM-DD\" 章节" >&2
    exit 1
fi

printf '%s\n' "$NOTES"
