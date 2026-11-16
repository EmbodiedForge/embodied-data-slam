# Local Kalibr image

Build the Ubuntu 20.04 / ROS Noetic Kalibr image from the project root:

```bash
docker build -t kalibr:ros1-20.04 -f docker/kalibr/Dockerfile docker/kalibr
```

Run it with the repository mounted at `/data`:

```bash
docker run --rm -it --net=host \
  -v "$PWD:/data" \
  kalibr:ros1-20.04 bash
```

The image is intentionally isolated from the Ubuntu 22.04 host. The build
downloads ROS/Kalibr dependencies and may take 20-60 minutes.
