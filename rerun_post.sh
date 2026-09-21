#!/usr/bin/env bash
set -euo pipefail

cd /home/moga/桌面/五类别预标链路
PY=/home/moga/miniconda3/envs/openpcdet/bin/python
DATA=/home/moga/桌面/SUSTechPOINTS/data
STAGE=/home/moga/SUSTechPOINTS/clips_stage
LOG=/home/moga/桌面/SUSTechPOINTS/logs
mkdir -p "$LOG"

weights=(epoch12 epoch15 epoch17 epoch20)
clips=(
  scene_crossroad_my_record_20260827_163412_clip0
  scene_crossroad_my_record_20260827_163412_clip3
  scene_nonmotor_lane_my_record_20260827_162436_clip13
  scene_nonmotor_lane_my_record_20260827_162436_clip15
  scene_nonmotor_lane_my_record_20260827_162436_clip5
)

for w in "${weights[@]}"; do
  for c in "${clips[@]}"; do
    if [ "$w" = "epoch12" ] && [ "$c" = "scene_crossroad_my_record_20260827_163412_clip0" ]; then
      echo "[skip] $w $c (already rerun)"
      continue
    fi
    raw="$DATA/${c}_${w}_raw.json"
    [ -f "$raw" ] || { echo "[MISSING raw] $w $c"; exit 1; }
    tmp=$(mktemp)
    cp "$raw" "$tmp"
    echo "[start] $w $c"
    if ! "$PY" run_end_to_end.py --clip "$STAGE/$c" --raw-json "$tmp" \
        --inference-python "$PY" --post-python "$PY" \
        --final-root "$DATA" --sust-root "$DATA" \
        --final-suffix "_${w}_pre" \
        --preserve-input --export-sust --overwrite \
        > "$LOG/${c}_${w}.post.log" 2>&1; then
      echo "[FAIL] $w $c"
      rm -f "$tmp"
      exit 1
    fi
    rm -f "$tmp"
    echo "[done] $w $c labels=$(ls "$DATA/${c}_${w}_pre/label" 2>/dev/null | wc -l)"
  done
done
echo "[batch] ALL POST RERUN DONE"
