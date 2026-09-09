# Docker 部署

项目会在容器启动时加载并预热 TensorFlow 模型，首次就绪可能需要几十秒。镜像内已包含
`algos/models/deep_annotation_inference.h5`，不需要在构建时从网络下载模型。

## 使用 Docker Compose（推荐）

在项目根目录执行：

```bash
docker compose up -d --build
docker compose ps
```

访问：

- 页面：<http://localhost:8081/>
- 健康检查：<http://localhost:8081/health>
- 场景元数据 API：<http://localhost:8081/datameta>

宿主机的 `./data` 会挂载到容器 `/app/data`，标注结果也会写回该目录。停止服务：

```bash
docker compose down
```

## 直接使用 Docker

```bash
docker build -t sustechpoints:local .
docker run -d \
  --name sustechpoints \
  --restart unless-stopped \
  -p 8081:8081 \
  -v "$(pwd)/data:/app/data" \
  sustechpoints:local
```

Windows PowerShell 可使用：

```powershell
docker run -d --name sustechpoints --restart unless-stopped `
  -p 8081:8081 `
  -v "${PWD}/data:/app/data" `
  sustechpoints:local
```

查看启动日志和健康状态：

```bash
docker logs -f sustechpoints
docker inspect --format '{{.State.Health.Status}}' sustechpoints
```

## 给其他平台调用

同一 Docker 网络中的服务可通过 `http://sustechpoints:8081` 调用；Kubernetes 或云平台应将
容器端口配置为 `8081`，健康检查路径配置为 `/health`。主要现有接口包括：

- `GET /health`：存活检查
- `GET /datameta`：所有场景元数据
- `GET /scenemeta?scene=<场景名>`：单场景元数据
- `GET /load_annotation?scene=<场景名>&frame=<帧号>`：读取标注
- `POST /saveworldlist`：写入标注（JSON 请求体）

当前项目没有认证、租户隔离或 API 限流，并且部分接口能修改挂载的数据。不要把 `8081`
直接开放到公网；生产环境应放在带 TLS、身份认证和访问控制的 API 网关或反向代理后面。
