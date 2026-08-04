# Screened Poisson mesher for scan3d (Scene 5) — Open3D is MIT-licensed.
# Built automatically by reconstruct_cpu.sh when MESHER=poisson:
#   docker build -t scan3d/poisson -f poisson.Dockerfile .
FROM python:3.12-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
        libx11-6 libgl1 libgomp1 libegl1 libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir "numpy<3" open3d
COPY poisson_mesh.py /usr/local/bin/poisson_mesh.py
ENTRYPOINT ["python3", "/usr/local/bin/poisson_mesh.py"]
