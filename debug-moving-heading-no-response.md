# [OPEN] moving-heading-no-response

## Symptom
- 用户新增“按 ID 拟合动态障碍物朝向”功能后，前端点击仍然没有反应，怀疑后端未重启导致新接口未生效。

## Hypotheses
- H1: 当前运行的后端进程仍是旧版本，尚未加载新接口 `/fit_moving_direction_by_id`。
- H2: 后端已重启，但启动目录或解释器不对，实际跑的不是当前 `SUSTechPOINTS` 工程。
- H3: 后端接口已存在，但前端仍加载旧缓存脚本，没有发起新的请求。
- H4: 前端请求已发出，但后端返回错误，前端只表现为“无反应”。
- H5: 当前页面选中的对象没有有效 `obj_track_id` 或相关轨迹不足，导致接口执行后没有更新。

## Plan
- 确认当前项目启动方式与工作目录。
- 重启后端服务，确保加载最新代码。
- 直接验证新接口是否可访问。
- 如有必要，再抓前端请求与控制台证据。

## Evidence
- 端口 `8081` 在重启前已可访问，首页返回 `200`。
- 重启前直接请求 `/fit_moving_direction_by_id?scene=example&obj_id=35`，接口已存在并返回：
  - `{"updated_frames":[],"fitted_frames":[],"skipped_frames":[],"message":"not enough valid track points to fit moving direction"}`
- 监听 `8081` 的旧进程是：
  - `D:\miniconda3\envs\sustechpoints\python.exe`
- 已显式杀掉旧进程并用同一解释器、同一工程目录重新启动：
  - `D:\miniconda3\envs\sustechpoints\python.exe main.py`
- 重启后 CherryPy 再次在 `http://0.0.0.0:8081` 启动，首页返回 `200`。
- 重启后再次请求 `/fit_moving_direction_by_id?scene=example&obj_id=35`，返回结果与重启前一致。

## Interim Conclusion
- H1 被否定：不是“旧后端没加载新接口”。
- H2 被否定：启动解释器与工程目录正确。
- 当前更可能是 H4/H5：
  - 前端没有把后端返回结果提示出来；
  - 或当前对象 ID 在该场景下确实无法形成足够轨迹，因此算法返回“没有可拟合结果”。
