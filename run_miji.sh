#!/usr/bin/env bash
set -euo pipefail

cd /home/moga/桌面/五类别预标链路
PY=/home/moga/miniconda3/envs/openpcdet/bin/python
DATA=/home/moga/桌面/SUSTechPOINTS/data
INPUT=/media/moga/police/密集
LOG=/home/moga/桌面/SUSTechPOINTS/logs
mkdir -p "$LOG"

clips=(
  scene_crossroad_my_record_20260827_164838_clip2
  scene_crossroad_my_record_20260827_164838_clip6
  scene_crossroad_my_record_20260827_164838_clip10
)

for w in epoch12 epoch15; do
  RAW_TMP=$(mktemp -d)
  echo "[batch] RAW $w start"
  if ! "$PY" scripts/run_five_class.py "$INPUT" "$DATA" \
      --mode inference --raw-output "$RAW_TMP" \
      --weight "$w" --skip-install --overwrite \
      > "$LOG/miji_${w}.raw.log" 2>&1; then
    echo "[FAIL] raw $w"; rm -rf "$RAW_TMP"; exit 1
  fi
  for r in "$RAW_TMP"/*_raw.json; do
    c=$(basename "$r"); c="${c%_raw.json}"
    mv "$r" "$DATA/${c}_${w}_raw.json"
    echo "[raw] ${c}_${w}_raw.json"
  done
  rm -rf "$RAW_TMP"

  echo "[batch] RAW $w done; post start"
  for c in "${clips[@]}"; do
    raw="$DATA/${c}_${w}_raw.json"
    [ -f "$raw" ] || { echo "[MISSING raw] $w $c"; exit 1; }
    tmp=$(mktemp); cp "$raw" "$tmp"
    echo "[start] post $w $c"
    if ! "$PY" run_end_to_end.py --clip "$INPUT/$c" --raw-json "$tmp" \
        --inference-python "$PY" --post-python "$PY" \
        --final-root "$DATA" --sust-root "$DATA" \
        --final-suffix "_${w}_pre" \
        --preserve-input --export-sust --overwrite \
        > "$LOG/miji_${c}_${w}.post.log" 2>&1; then
      echo "[FAIL] post $w $c"; rm -f "$tmp"; exit 1
    fi
    rm -f "$tmp"
    echo "[done] post $w $c labels=$(ls "$DATA/${c}_${w}_pre/label" 2>/dev/null | wc -l)"
  done
done
echo "[batch] 密集 DONE"
