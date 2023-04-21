# LCASTOR Docker

Docker configuration for the LCASTOR project.

To use the Dockerfile, first run

```bash
./build_docker.sh
```

to build the lcastor_base image.

To then run it with all GPU capabilities necessary and mounting the current LCASTOR working directoy (from the host), run


```bash
./run_docker.sh
```

This will drop you into a shell with access to your host ROS-workspace, located at /ros_ws. Continue development from there.