# ORB SLAM3 ROS Docker

![ORB SLAM3 ROS Docker](https://img.shields.io/badge/ORB_SLAM3_ROS-Docker-blue)

Container environment in which orb_slam3_ros_wrapper runs.

## References

---

- [ORB_SLAM3 - GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [orb_slam3_ros_wrapper - GitHub](https://github.com/thien94/orb_slam3_ros_wrapper)


## Environments

---

- ROS : noetic


## Building

---

```bash
docker-compose up -d
```


## Running

---

GUI display of the ROS tool in the host environment. <br>

```bash
xhost +
```

> **Warning** <br>
> After use, access should be restricted with `xhost -`.


Enter the container: <br>

```bash
docker-compose exec orb-slam3-ros-workspace bash
```

Connect a monocular camera. <br>
Activate the camera node. <br>
And, run `euroc_mono.launch` from the orb-slam3-ros-wapper package. <br>

terminal1 : <br>

```bash
source /opt/ros/noetic/setup.bash
rosrun usb_cam usb_cam_node
```

terminal2 : <br>

```bash
source /opt/ros/noetic/setup.bash
. ./devel/setup.bash
roslaunch orb-slam3-ros-wrapper euroc_mono.launch
```
