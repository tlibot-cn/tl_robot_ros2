#!/usr/bin/env bash
#
# C++ 代码格式化包装脚本
# 自动跳过三方 SDK 头文件，用法与 clang-format 一致
#
# 用法:
#   ./scripts/format-cpp.sh                     # 格式化 src/ 下所有 C++ 文件
#   ./scripts/format-cpp.sh src/tl_driver/src/  # 格式化指定目录
#   ./scripts/format-cpp.sh file1.cpp file2.h   # 格式化指定文件
#

set -euo pipefail

PROJECT_ROOT=$(git rev-parse --show-toplevel 2>/dev/null || dirname "$(dirname "$0")")
CLANG_FORMAT_FILE="${PROJECT_ROOT}/.clang-format"

# ── 忽略路径（与 .githooks/pre-commit 保持一致） ──
IGNORE_PATTERNS=(
    '^src/tl_driver/lib/include/'
    '^src/tl_teleop/lib/include/'
)

# ── 判断文件是否应被忽略 ──────────────────────
is_ignored() {
    local file="$1"
    for pat in "${IGNORE_PATTERNS[@]}"; do
        if [[ "$file" =~ $pat ]]; then
            return 0  # 忽略
        fi
    done
    return 1  # 不忽略
}

# ── 收集要格式化的文件 ────────────────────────
collect_files() {
    if [ $# -gt 0 ]; then
        # 用户指定了文件/目录 → 展开目录，过滤
        for arg in "$@"; do
            if [ -d "$arg" ]; then
                find "$arg" -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.cc' -o -name '*.cxx' \)
            elif [ -f "$arg" ]; then
                echo "$arg"
            fi
        done
    else
        # 未指定 → 默认扫描 src/
        find "${PROJECT_ROOT}/src" -type f \( -name '*.cpp' -o -name '*.h' -o -name '*.hpp' -o -name '*.cc' -o -name '*.cxx' \)
    fi | while IFS= read -r f; do
        # 转为相对路径（用于匹配忽略规则）
        rel="${f#${PROJECT_ROOT}/}"
        is_ignored "$rel" || echo "$f"
    done
}

# ── 主流程 ────────────────────────────────────
if [ ! -f "$CLANG_FORMAT_FILE" ]; then
    echo "Error: .clang-format not found in project root" >&2
    exit 1
fi

FILES=$(collect_files "$@")

if [ -z "$FILES" ]; then
    echo "No C++ files to format (or all are ignored)"
    exit 0
fi

echo "$FILES" | tr '\n' '\0' | xargs -0 clang-format -i

COUNT=$(echo "$FILES" | wc -l)
echo "Formatted ${COUNT} C++ file(s)"
echo "Ignored paths: ${IGNORE_PATTERNS[*]}"
