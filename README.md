# FRA532 Mobile Robot : Exam1
This `Exam 1` repository is used in the `FRA532 Mobile Robot` class at FIBO. It covers kinematics, odometry, slam and navigation aims to do ekf filter with combine with imu+gps by use diff drive `mir robot` **executes in Gazebo simulation environment**.

## Table of Contents
- [Demo Video](#demo-video)
- [System Overview](#system-overview)
- [Installation](#installation)
- [Filter odom with slam and nav2](#filter-odom-with-slam-and-nav2)


## Demo Video



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



