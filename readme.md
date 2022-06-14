# 六爻飞梦无人机仿真和实物实验
## 仿真环境安装 **ubuntu 18.04** 

### pixhawk安装

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
```
ROS 安装
```bash
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
bash ubuntu_sim_ros_melodic.sh
```
gazebo model 下载
```bash
https://github.com/osrf/gazebo_models
cp -r gazebo_models ~/.gazebo/models
```
QGC 下载
```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
```
下载qgc https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
```bash
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```
mavros 安装
```bash
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```
px4 gazebo编译
```bash
cd PX4-Autopilot
make px4_sitl gazebo
```
安装zbar
```bash
sudo apt-get install ros-melodic-zbar-ros
```

## 使用
**编译工作空间**
```bash
cd ~/catkin_ws/src
git clone https://github.com/gcx158632979/lyao_tutorial
catkin_make -DCATKIN_WHITELIST_PACKAGES="catkin_simple"
catkin_make -DCATKIN_WHITELIST_PACKAGES="eigen_catkin"
catkin_make -DCATKIN_WHITELIST_PACKAGES="glog_catkin"
catkin_make -DCATKIN_WHITELIST_PACKAGES="eigen_checks"
catkin_make -DCATKIN_WHITELIST_PACKAGES="nlopt"
catkin_make
```
**启动px4仿真**
```bash
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash   
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 posix_sitl.launch
```
**$(pwd) means your path**
**启动qgc**
```bash
./QGroundControl.AppImage
```

## 测试
**启动takeoff节点**
```bash
rosrun control_test takeoff
```
**启动高度控制节点**
```bash
rosrun control_test heightcontrol
```
**启动轨迹跟踪节点**
```bash
rosrun control_test followtraj
```
**启动摄像头节点**
```bash
roslaunch zbar_ros example_usb.launch
```