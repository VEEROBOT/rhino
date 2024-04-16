# Setting up Ubuntu and ROS

**_NOTE:_** All things need to setup on both host and remote system

## Ubuntu Setup

1. **Download Ubuntu Image:** 
   - Go to [Ubuntu Releases](https://releases.ubuntu.com/jammy/) and download the image suitable for your system.

2. **Flash Ubuntu Image:**
   - Watch this [Tutorial](https://www.youtube.com/watch?v=QKn5U2esuRk&pp=ygUSdWJ1bnR1IDIyIGR1YWwgYm9v) to learn how to flash Ubuntu on both your host and remote PC.

## ROS Installation

1. **Install ROS Humble:**
   - Follow the instructions provided in the [ROS Documentation](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) to install ROS Humble on your system.

2. **Create ROS Workspace:**
   - Refer to the ROS tutorials to [Create a Workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).
   - Once created, remember that the setup.bash file needs to be sourced every time you open a new terminal. To automate this, follow these steps:
   **_NOTE:_** Skip 3 while creating workspace instead clone this repo inside src dir

      ```bash
      nano ~/.bashrc
      # Add the following lines to the end of the file
      source /opt/ros/humble/setup.bash
      source ~/your_workspace_name/install/setup.bash
      ```

      Replace `your_workspace_name` with the name of your workspace.

3. **Install ROS Navigation Stack Package:**
   - Install the ROS Navigation stack package along with other dependencies using the following command:

      ```bash
      sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-xacro  ros-humble-rviz2 ros-humble-tf-transformations ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-tf2-* ros-humble-slam-toolbox ros-humble-rplidar-ros-* 
      ```
      ```bash
      sudo pip install transform3d
      ```
## Micro_ROS Setup

Follow this link to install the Micro_ROS agent on the host: [Micro_ROS Installation Guide](https://micro.ros.org/docs/tutorials/core/first_application_linux/)

**_NOTE:_** No need to follow firmware step just need to create agent

## Udev Rules setup

```bash
cd /etc/udev/rules.d
sudo touch 99-my_rules.rules
sudo nano 99-my_rules.rules
```

```bash
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="ttyesp32", GROUP="<ADD_GROUP_NAME>", MODE="0660"

ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", SYMLINK+="ttyLidar", GROUP="<ADD_GROUP_NAME>", MODE="0660"

```
**you can find group using cmd 'groups'**

```bash
sudo chmod 644 99-my_rules.rules 
sudo chown root:root 99-my_rules.rules 
sudo udevadm control --reload-rules        
sudo udevadm trigger
```

## Platformio
Install Platforio on vscode and Upload micro_ros firmware in esp32 (**in micro_ros package**)
[Platformio Installation Guide](https://www.youtube.com/watch?v=MeIcL9igsbM)
**_NOTE:_** Change main.cpp code accordingly to main_moto_only.cpp if needed

## Launch Files

### To start Mapping
```bash
ros2 launch four_w_amr_nav2 slam.launch.py
```
### To save Map
```bash
ros2 run nav2_map_server map_saver_cli -f map
```
**Replace created map(ymal and pgm) with map in four_w_amr_nav2/maps/**


### To run base controller, lidar & navigation

#### To run without realsense

**On Remote machine using ssh**
```bash
ros2 launch four_w_amr bringup.launch.py
ros2 launch four_w_amr lidar.launch.py
```

**On Host machine**
```bash
ros2 launch four_w_amr display.launch.py
ros2 launch four_w_amr_nav2 navigation_launch.py
```

#### To run with realsense

**On Remote machine using ssh**
```bash
ros2 launch four_w_amr realsense.launch.py
ros2 launch four_w_amr lidar.launch.py
```

**On Host machine**
```bash
ros2 launch four_w_amr display.launch.py
ros2 launch four_w_amr_nav2 navigation_launch.py
```