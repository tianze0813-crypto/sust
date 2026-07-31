# [OPEN] backend-auto-pause

## Symptom
- 用户希望后端常驻运行，但服务会自动暂停/退出。
- 现象表现为：前端断连，守护进程检测到 `main.py` 退出后自动重启。

## Hypotheses
- H1: `TensorFlow` 或其底层原生依赖触发访问冲突，导致 Python 进程异常退出。
- H2: `CherryPy` 的 autoreloader 与当前环境/运行方式冲突，触发非正常退出。
- H3: `temp/views/assets` 等缺失目录只是启动警告，不是根因。
- H4: Trae sandbox 与本地原生库交互导致退出码 `3221225477`。
- H5: 不是缺包，而是已安装依赖版本之间存在 ABI 或运行时兼容性问题。

## Plan
- 停掉当前守护进程，避免干扰日志采集。
- 收集最近一次退出码、启动日志、Python/依赖版本信息。
- 对比“导入依赖”与“启动服务”两个阶段，判断崩溃点。

## Evidence
- 已停止守护进程，避免自动重启掩盖退出现象。
- `python 3.10.20`，环境位于 `D:\miniconda3\envs\sustechpoints\python.exe`。
- 已确认安装关键依赖：`tensorflow 2.10.1`、`CherryPy 18.10.0`、`numpy 1.23.5`。
- 单独导入 `tensorflow/cherrypy/numpy` 能正常退出，不存在“缺包即崩”的情况。
- 启动 `main.py` 后，服务能成功监听 `http://0.0.0.0:8081`，并能返回 `GET / -> 200`。
- 之后进程异常退出，退出码为 `3221225477`（十六进制 `0xC0000005`，访问冲突）。
- Windows 事件查看器记录的故障模块为 `nvdxgdmal64.dll_unloaded`，对应 NVIDIA DirectX 图形驱动模块。
- `server.conf` 中 `temp/views/assets` 缺失与 `auth.require` 未知命名空间只产生 CherryPy Checker 警告，服务在这些警告后仍成功启动，因此不是根因。

## Hypothesis Status
- H1: 基本成立。当前最强证据指向原生图形/驱动层崩溃，而非 Python 业务异常。
- H2: 暂不成立。`Autoreloader` 启动后服务可正常对外提供 `200`，未见其直接触发退出的证据。
- H3: 已排除。缺失目录与配置警告不影响服务成功启动。
- H4: 高度可疑。崩溃日志中同时出现 `TRAE Sandbox Error: process crashed`，且当前复现路径均发生在 sandbox 内。
- H5: 基本排除“缺依赖”。更像是已安装依赖与驱动/运行环境的原生兼容性问题。

## Current Conclusion
- 当前“自动暂停/掉线”的直接原因不是缺 Python 包。
- 直接原因是 `python.exe` 在运行期发生原生层访问冲突，故障模块落在 `nvdxgdmal64.dll`。
- 最可疑的组合是：`TensorFlow`/图形相关原生库 + 当前 Trae sandbox 运行方式 + 本机 NVIDIA 图形驱动。
