# ORB SLAM3 ROS Docker

![ORB SLAM3 ROS Docker](https://img.shields.io/badge/ORB_SLAM3_ROS-Docker-blue)

[en](./README.md) / ja

orb_slam3_ros_wrapperの実行環境を提供するコンテナ

## 参照

---

- [ORB_SLAM3 - GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [orb_slam3_ros_wrapper - GitHub](https://github.com/thien94/orb_slam3_ros_wrapper)


## 実行環境

---

- ROS : noetic


## 環境構築

---

```bash
docker-compose up -d
```


## 実行

---

GUIで提供されるROSツールをホスト環境で表示できるようにする <br>

```bash
xhost +
```

> **Warning** <br>
> 使用後は `xhost -` でアクセス制限を設けること <br>


コンテナに入る: <br>

```bash
docker-compose exec orb-slam3-ros-workspace bash
```

1. 単眼カメラを接続する
2. カメラノードを起動する(libuvc-camera, noetic-usb-camなど)
3. orb-slam3-ros-wrapperパッケージの `euroc_mono.launch` を起動する

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
