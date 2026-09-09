# syntax=docker/dockerfile:1
FROM python:3.11-slim-bookworm

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    CUDA_VISIBLE_DEVICES=-1 \
    TF_USE_LEGACY_KERAS=0 \
    TF_CPP_MIN_LOG_LEVEL=2

WORKDIR /app

# TensorFlow requires libgomp at runtime. libgl/libglib cover OpenCV utilities.
RUN apt-get update \
    && apt-get install --no-install-recommends -y libgomp1 libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

COPY requirement.txt constraints.txt ./
RUN python -m pip install --upgrade pip \
    && python -m pip install -r requirement.txt -c constraints.txt

COPY . .
RUN mkdir -p /app/data /app/temp

EXPOSE 8081
VOLUME ["/app/data"]

HEALTHCHECK --interval=30s --timeout=5s --start-period=120s --retries=3 \
  CMD ["python", "-c", "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8081/health', timeout=3).read()"]

CMD ["python", "main.py"]
