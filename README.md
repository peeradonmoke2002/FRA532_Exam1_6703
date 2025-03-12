# FRA532 Mobile Robot : Exam1
This `Exam 1` repository is used in the `FRA532 Mobile Robot` class at FIBO. It covers kinematics, odometry, slam and navigation for carver outdoor robot which **executes in Gazebo simulation environment**.

## Table of Contents

- [System Overview](#system-overview)
- [Installation](#installation)



## System Overview


Forder stucture 
```markdown
└── src
    ├── carver_controller
    ├── carver_description
    ├── carver_gazebo
    ├── carver_navigation
    ├── carver_odometry
    └── carver_slam
```

## Installation

> [!WARNING]  
> Make sure you have installed **ROS 2 Humble**.  
> If not, please visit the official ROS 2 installation guide:  
> [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)


### Step 1: Clone the Repository
```bash
git clone https://github.com/peeradonmoke2002/FRA532_Exam1_6703.git
```

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

