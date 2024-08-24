In the previous guide, we had installed just the OS- Ubuntu 22.04. In this guide, we will install ROS 2 Humble, inside Ubuntu 22.04. Think of it as installing Windows 10 first, then installing Python.

Later on, we will also see just like how Python has many packages, so does ROS2. For example- Gazebo, Turtlebot4, std_msgs, etc.

**NOTE**: Docker container and Virtual Machine users, skip this guide as it comes preinstalled with ROS 2 Humble and Gazebo

# ROS 2 Humble Hawksbill installation guide:
1. Open a terminal using `Ctrl+Alt+T`
1. Follow these [instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) carefully
1. Try out the [examples](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#try-some-examples) as suggested ensuring everything works.
1. In order to source your colcon workspace everytime you open a new terminal automatically, run this command in your terminal:
    * (Do not copy the '\$' sign, the '\$' sign is an indicator that this command must be run in the terminal)
    * $`echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc`
1. Limit ROS2 communication to this device only
    * $`echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc`
1. Install colcon and pip:
    * $`sudo apt install python3-colcon-common-extensions`
    * $`sudo apt install python3-rosdep2`
    * $`sudo apt install python3-pip`

Once you're done setting up Ubuntu 22.04 and ROS 2 Humble, read [this](https://github.com/naslab-projects/ME597-Fall2024/blob/main/0-Setup/Resources/Software_pkg_%26_OS.md) to understand what we have just done till now. 

# Simulation Installation guide
Now we will install the simulation. For this course we will use a simulation of the Turtlebot3. There is a Turtlebot4 simulator available with Gazebo Ignition Fortress simulation software, but it has higher graphics requirements, so we will be using Gazebo Classic 11.10 simulation software, with Turtlebot3 models. Functionally, they will be the same: sensors, topics, etc. If you are using a Mac please look at the following link for installation: [Install Gazebo on Mac](https://classic.gazebosim.org/tutorials?tut=install_on_mac&cat=install).

### Gazebo 11.10 (Classic) simulation software installation:
1. Execute these commands in the terminal to install Gazebo:
    ```
    sudo apt update
    sudo apt install ros-humble-gazebo-ros-pkgs
    ```
1. Open a new terminal and run this command to ensure everything is working fine:
    * If you get an error in this step, reboot the system
    * $`gazebo --verbose /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world`
    * The Gazebo world along with a bot should have been loaded and rendered
    * $`ros2 topic pub /demo/cmd_demo geometry_msgs/msg/Twist '{linear: {x: 1.0}}' -1`
    * You should see the bot move in the Gazebo world
    * To kill any process in the linux terminal, press `Ctrl+c`


### Turtlebot3 model installation (Available starting lab 2):

1. Simply save the turtlebot3_gazebo package to a workspace e.g., `~/sim_ws/src/turtlebot3_gazebo`.  You can find it here: [turtlebot3_gazebo](https://github.com/naslab-projects/ME597_Final_project_pkgs_ros2)

3. Add turtlebot3 environment variable to .bashrc file
    ```
    echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
    ```
2. Run these to install useful packages you will use in the simulator.
    ```
    sudo apt install ros-humble-turtlebot3-teleop
    sudo apt install ros-humble-slam-toolbox
    sudo apt install ros-humble-navigation2
    ```

### Using the simulation for lab assignments:
You will be using this simulation package in future tasks: Lab2, Lab3, Lab4, and Final Project. These will each have their own workspace. How can we use the simulator package, which belongs in its own workspace, in a different workspace? Learn what this means, and how, [on this page](https://github.com/naslab-projects/ME597-Fall2024/blob/main/1-ROS_2_Basics/4-ROS_2_workspaces.md).


You're now ready to learn and play with ROS 2! Wohoo!
