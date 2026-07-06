#!/bin/bash
# ============================================================
# Claude Code Stop Hook — 每次回复后自动显示会话摘要
# ============================================================

GIT_DIR="d:/我的文档/26电赛/电赛STM32小车核心代码/stm32-master/car_example"

# ---- 1. 当前模型 ----
MODEL="${CLAUDE_MODEL:-unknown}"

# ---- 2. Git 未提交文件数 ----
STAGED=$(cd "$GIT_DIR" 2>/dev/null && git diff --cached --name-only 2>/dev/null | wc -l)
MODIFIED=$(cd "$GIT_DIR" 2>/dev/null && git diff --name-only 2>/dev/null | wc -l)
UNTRACKED=$(cd "$GIT_DIR" 2>/dev/null && git ls-files --others --exclude-standard 2>/dev/null | wc -l)
TOTAL_UNCOMMITTED=$((STAGED + MODIFIED + UNTRACKED))

# ---- 3. 未推送的 commit 数 ----
UNPUSHED_GITHUB=$(cd "$GIT_DIR" 2>/dev/null && git rev-list --count github/main..HEAD 2>/dev/null)
UNPUSHED_GITEE=$(cd "$GIT_DIR" 2>/dev/null && git rev-list --count gitee/main..HEAD 2>/dev/null)

# ---- 输出 ----
echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║  📊 会话状态                                      ║"
echo "╠══════════════════════════════════════════════════╣"
printf "║  Model : %-40s ║\n" "$MODEL"
printf "║  Git   : %-3s 个文件有未提交的修改              ║\n" "$TOTAL_UNCOMMITTED"
if [ "$UNPUSHED_GITHUB" != "" ] && [ "$UNPUSHED_GITHUB" -gt 0 ] 2>/dev/null; then
  printf "║  GitHub: %-3s 个 commit 未推送                  ║\n" "$UNPUSHED_GITHUB"
fi
if [ "$UNPUSHED_GITEE" != "" ] && [ "$UNPUSHED_GITEE" -gt 0 ] 2>/dev/null; then
  printf "║  Gitee : %-3s 个 commit 未推送                  ║\n" "$UNPUSHED_GITEE"
fi
echo "╚══════════════════════════════════════════════════╝"
