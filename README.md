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
step4: save map for use in navigation process
```
ros2 launch robot_slam save_map.launch.py
```
> [!IMPORTANT]
>due to this repo foucs on slam the nav2 will not process 

### Resutls base from demo video

#### Screenshot while tesing with odom from diff plugin and odom yawrate
![image](https://github.com/user-attachments/assets/2d2af7a5-3efa-486f-a495-f0a263a60204)

- base from resutls the yaw rate have error a litte suffer from errors or slippage. howerver is close to odom that pub from diff drive plugin 

#### Screenshot while tesing with odom from diff plugin and odom yawrate and gps with fillter wiht ekf
![image](https://github.com/user-attachments/assets/1c9daf59-4095-474d-84df-f2510b4f5c97)

- base from resutls the this have much lage in term of odom due to process to filter is yawrate and gps -> ekf -> odom filter however is req to tune to improve odom to close to odom diff drive
