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


