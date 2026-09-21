#!/usr/bin/env bash
set -euo pipefail

cd /home/moga/桌面/五类别预标链路
PY=/home/moga/miniconda3/envs/openpcdet/bin/python
DATA=/home/moga/桌面/SUSTechPOINTS/data
INPUT=/media/moga/police/test
CFG=models/voxelnext_fiveclass_nuscenes_infer.yaml
LOG=/home/moga/桌面/SUSTechPOINTS/logs
mkdir -p "$LOG"

weights=(expA_e16 expB2_e8 expC_e8 expD_e8)
clips=(
  scene_crossroad_my_record_20260827_163412_clip0
  scene_crossroad_my_record_20260827_163412_clip3
  scene_crossroad_my_record_20260827_164838_clip2
  scene_crossroad_my_record_20260827_164838_clip6
  scene_crossroad_my_record_20260827_164838_clip10
)

for w in "${weights[@]}"; do
  for c in "${clips[@]}"; do
    echo "[start] $w $c"
    if ! "$PY" run_end_to_end.py --clip "$INPUT/$c" \
        --inference-python "$PY" --post-python "$PY" \
        --cfg "$CFG" --ckpt "models/$w.pth" \
        --final-root "$DATA" --sust-root "$DATA" \
        --final-suffix "_${w}_pre" \
        --preserve-input --export-sust --overwrite \
        > "$LOG/${c}_${w}.log" 2>&1; then
      echo "[FAIL] $w $c"
      exit 1
    fi
    echo "[done] $w $c labels=$(ls "$DATA/${c}_${w}_pre/label" 2>/dev/null | wc -l)"
  done
done
echo "[batch] ALL EXP WEIGHTS DONE"
