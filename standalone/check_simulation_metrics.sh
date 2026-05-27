#!/usr/bin/env bash
# 回归检查：standalone 仿真指标须与 README 一致（允许浮点末位误差）
set -euo pipefail

BIN="${1:-}"
if [[ -z "$BIN" ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  BIN="${SCRIPT_DIR}/../build_standalone/vins_multi_simulation"
fi

if [[ ! -x "$BIN" ]]; then
  echo "error: executable not found: $BIN" >&2
  exit 1
fi

OUT="$("$BIN" 2>&1)"
echo "$OUT" | tail -5

extract() {
  local key="$1"
  echo "$OUT" | sed -n "s/.*\\[metrics\\]\\[${key}\\].*mae=\\([0-9.eE+-]*\\).*/\\1/p" | head -1
}

RAW_MAE="$(extract raw)"
ALIGNED_MAE="$(extract aligned)"
VEL_MAE="$(extract vel)"

expect="0.0753451"
tol="5e-5"

awk -v raw="$RAW_MAE" -v aligned="$ALIGNED_MAE" -v vel="$VEL_MAE" \
    -v expect="$expect" -v tol="$tol" 'BEGIN {
  if (raw == "" || aligned == "" || vel == "") {
    print "FAIL: could not parse metrics lines"; exit 1;
  }
  ok = 1;
  if (raw + 0 < expect - tol || raw + 0 > expect + tol) {
    printf "FAIL: raw mae=%s expected ~%s (tol=%s)\n", raw, expect, tol; ok = 0;
  }
  if (aligned + 0 < 0.0752903 - tol || aligned + 0 > 0.0752903 + tol) {
    printf "FAIL: aligned mae=%s expected ~0.0752903\n", aligned; ok = 0;
  }
  if (vel + 0 < 0.00444835 - tol || vel + 0 > 0.00444835 + tol) {
    printf "FAIL: vel mae=%s expected ~0.00444835\n", vel; ok = 0;
  }
  if (ok) {
    printf "PASS: raw=%s aligned=%s vel=%s\n", raw, aligned, vel;
    exit 0;
  }
  exit 1;
}'
