# Build from the repository root: docker build -f software/umi_slam/Dockerfile -t umi-slam:0.1 .
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build libeigen3-dev libopencv-dev libboost-serialization-dev \
    libssl-dev libgl1-mesa-dev libegl1-mesa-dev libepoxy-dev libglew-dev \
    libjpeg-dev libpng-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY CMakeLists.txt /src/CMakeLists.txt
COPY cmake /src/cmake
COPY include /src/include
COPY src /src/src
COPY source/stereo_imu_trajectory/ORB_SLAM3 /src/source/stereo_imu_trajectory/ORB_SLAM3
COPY source/stereo_imu_trajectory/Pangolin /src/source/stereo_imu_trajectory/Pangolin
COPY software/umi_slam/backend /src/software/umi_slam/backend
ARG BUILD_JOBS=2
RUN cmake -S /src -B /opt/umi_slam/build -DCMAKE_BUILD_TYPE=Release \
        -DEDS_BUILD_STATION=OFF -DBUILD_TESTING=OFF \
    && cmake --build /opt/umi_slam/build -j ${BUILD_JOBS} \
    && /opt/umi_slam/build/umi_slam_pipeline --build-backend --build-dir /opt/umi_slam/backend --jobs ${BUILD_JOBS}
ENTRYPOINT ["/opt/umi_slam/build/umi_slam_pipeline", "--backend", "/opt/umi_slam/backend/backend.json"]
CMD ["--help"]
