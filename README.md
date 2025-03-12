# FRA532 Mobile Robot : Exam1
This `Exam 1` repository is used in the `FRA532 Mobile Robot` class at FIBO. It covers kinematics, odometry, slam and navigation aims to do ekf filter with combine with imu+gps by use diff drive `mir robot` **executes in Gazebo simulation environment**.

## Table of Contents
- [Demo Video](#demo-video)
- [System Overview](#system-overview)
- [Installation](#installation)
- [Filter odom with slam and nav2](#filter-odom-with-slam-and-nav2)


## Demo Video






https://github.com/user-attachments/assets/da60ee79-d010-4332-b1ee-d6eff75d68f2



## System Overview



## Installation

> [!WARNING]  
> Make sure you have installed **ROS 2 Humble**.  
> If not, please visit the official ROS 2 installation guide:  
> [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)


### Step 1: Clone the Repository
```bash
git clone https://github.com/peeradonmoke2002/FRA532_Exam1_6703.git
```
Forder stucture 
```markdown
└── src
    ├── aws-robomaker-small-warehouse-world
    ├── robot_bringup
    ├── robot_control
    ├── robot_localization
    ├── robot_nav
    └── robot_slam
```
> [!WARNING]
> plase be sure your work space are following forder stucture


### Step 2: Build Workspace
```bash
cd ~/FRA532_Exam1_6703

sudo apt update

sudo apt install -y python3-rosdep

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source ~/FRA532_Exam1_6703/install/setup.bash
```
> [!TIP]  
>[Optional] To automatically source the workspace in every terminal, add this line to your `~/.bashrc`:
>```bash
>echo "source ~/FRA532_Exam1_6703/install/setup.bash" >> ~/.bashrc
>```

## Filter odom with slam and nav2

The robot estimates its position using odometry based on a yaw rate that fuses wheel velocity data with yaw information from the IMU. However, this method can still suffer from errors or slippage. To improve accuracy, we employ an Extended Kalman Filter (EKF) via the robot_localization package. The EKF refines the pose estimation by reducing noise and compensating for data gaps, ultimately publishing the improved estimate on the `/odometry/filtered` topic.

### Step for do slam


step 1: open gazbo and import model robot
```bash
ros2 launch robot_bringup sim_slam.launch.py
```
step 2: open rviz and slam toolbox
```bash
ros2 launch robot_slam mapping.launch.py
```
step3: Control robot to collect map data
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
step4: save map
```
ros2 launch robot_slam save_map.launch.py
```

### Step for do nav

step 1: open gazbo and import model robot
```bash
ros2 launch robot_bringup sim_slam.launch.py
```
step 2: open rviz and nav2
```bash
ros2 launch robot_nav nav.launch.py
```
