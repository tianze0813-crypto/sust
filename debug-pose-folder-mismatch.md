# [OPEN] pose-folder-mismatch

## Symptom
- 用户在正式场景 `data/my_record_20260414_115041_sub_dazhuo_anno` 中测试“按 ID 拟合动态障碍物朝向”仍无效果。
- 用户怀疑正式数据使用 `pose` 目录而不是 `ego_pose`，可能因为变量名或目录名不一致导致算法拿不到位姿。

## Hypotheses
- H1: 正式场景只有 `pose/`，而当前代码只读取 `ego_pose/`，导致位姿全部读取失败。
- H2: `pose/` 存在，但文件格式与 `scene_reader.read_ego_pose()` 预期的逐帧 JSON 不一致。
- H3: 正式场景的 pose 数据是集中式文件（例如 `pose/lidar_pose.json`），而新功能当前只支持逐帧 pose 文件。
- H4: pose 没问题，但目标 ID 的跨帧轨迹依然不足以拟合朝向。

## Plan
- 检查正式场景目录结构和 pose 文件组织方式。
- 对照 `scene_reader.read_ego_pose()` 与新功能的读取路径。
- 确认是否存在“目录名/文件组织不兼容”的证据。
