# Build from the repository root: docker build -f umi_slam/Dockerfile -t umi-slam:0.1 .
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake libeigen3-dev libopencv-dev libboost-serialization-dev \
    libssl-dev libgl1-mesa-dev libegl1-mesa-dev libepoxy-dev libglew-dev \
    libjpeg-dev libpng-dev python3 python3-numpy python3-opencv ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY stereo_imu_trajectory/ORB_SLAM3 /src/stereo_imu_trajectory/ORB_SLAM3
COPY stereo_imu_trajectory/Pangolin /src/stereo_imu_trajectory/Pangolin
COPY umi_slam /src/umi_slam
ARG BUILD_JOBS=2
RUN python3 /src/umi_slam/build_backend.py --jobs ${BUILD_JOBS}
ENV PYTHONUNBUFFERED=1
ENTRYPOINT ["python3", "/src/umi_slam/pipeline.py"]
CMD ["--help"]
