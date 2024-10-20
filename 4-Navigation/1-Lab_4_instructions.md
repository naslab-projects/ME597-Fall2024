# ME 597- Lab 4

## Autonomous Navigation using TurtleBot4

For the fourth lab and final lab, your primary task will be learning and implementing Simultaneous Localization And Mapping (SLAM) and Navigation techniques using the TurtleBot4.

### Introduction
Mobile robots are used in different environments to perform a wide range of tasks such as cleaning, patrolling, and rescue operations. In order to perform these tasks, the robots must move freely and autonomously through either static or dynamic environments. This process is navigation and it allows a robot to move through the current environment while performing tasks according to predefined rules. Examples of these rules include obstacle avoidance, cost minimization, and driving comfort.

In order to navigate, a mobile robot must gather information about the environment it is in. A common way to generate a map is with simultaneous localization and mapping (SLAM). The map can either be generated before a robot starts navigation (graph-based algorithms) or can be generated as the robot navigates an unknown environment (sampling-based algorithms). In this lab, we will be using an graph-based algorithm.

Combined mapping and navigation allow robots to move through a predefined path, to adjust its trajectory due to changes in the environment, or to correct motion online due to measurement errors. It is also important to consider extreme cases in which the robot gets lost or crashes into something, by implementing recovery strategies that bring the robot back to a safe known state. 

#### Words of advice:
* Whenever you're lost or have a doubt, Google it! Self-help will take you a long way in this course. A list of dependable and trustworthy resources (websites) is [here](https://github.com/naslab-projects/ME597/blob/master/0-Setup/Resources/References.md).
* Students who make mistakes AND attempt to correct it will learn way more than those who finish the tasks without any errors/bugs.

### Instructions
In this lab, your primary task will be to implement SLAM and navigation capabilities in simulation and hardware. This includes 4 sub-tasks:

Task_7:
1) Mapping
2) Localization
3) Planning
4) Execution 

This will be a three-week lab, with the following tasks each week:
* Week 1: Mapping (hardware), mapping (simulation), A*
* Week 2: Navigation (simulation)
* Week 3: Navigation (hardware)

Please note that all hardware tasks should be done with your partner. All other tasks must be done individually. Solutions may be shared for partnered work, but individual work must still be unique.

**Please do not use preexisting navigation packages like Nav2.**

### Week 8
This week, you will begin by generating a map using SLAM and implementing an A* algorithm for path planning.

#### Part A: Mapping (Hardware) (1 hr) (Partnered)
* `task_7`- Generating map data

NOTE: The robot used in this class has a namespace of `/robot`, so when you are launching a file, remember to include the namespace argument, e.g. `ros2 launch task_7 gen_sync_map.launch.py namespace:=/robot`

Using [this](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html) as reference:
1. Create a ROS 2 `ament_python` package called `task_7`, with two sub-folders: `maps`and `launch`.
2. Write a launch file called `gen_sync_map_launch.py` which performs the following:
    * Launch the `slam.launch.py` file (which belongs to `turtlebot4_navigation` pkg) from within this launch file to start SLAM.
    * Launch the `view_robot.launch.py` file (which belongs to `turtlebot4_viz` pkg) from within this launch file to view the map in Rviz2.
    * Run the `teleop_twist_keyboard` node (which belongs to `teleop_twist_keyboard` [pkg](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html)) from within this launch file to teleoperate the TurtleBot4.
3. Complete the relevant tag details in the `package.xml` file, build and run the ROS 2 node.

NOTE: If the robot doesn't move as you press the keyboard, you can run this command seperately: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/robot/cmd_vel`. The reason why the robot doesn't move is that the message is sent to a topic called `/cmd_vel` instead of `/robot/cmd_vel`, so we use this line of command to change the target topic name. 

4. Also verify with these commands:
    * $`ros2 node list`
    * $`ros2 topic list`
    * $`ros2 node info <node_name>`
    * $`ros2 topic echo <topic_name>`
5. Once you're satisfied with clarity of the generated map, manually [save the map](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html#save-the-map) as `classroom_map`, before killing `slam.launch.py`.

NOTE: Again, if the saving fails, that would be caused by lack of namespace, so use this command instead: `ros2 run nav2_map_server map_saver_cli -f "map_name" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/namespace`.


#### Part B: Mapping (Software) (10 min) (Individual)

Now we will map in simulation.

1. Follow the instructions [here](https://github.com/naslab-projects/sim_ws.git) to use the simulation environment. Note the specific launch instructions in the `src/turtlebot3_gazebo` subdirectory.
2. Manually map the `turtlebot3_house` world:
   * Launch the `mapper.launch.py` file (which belongs to `turtlebot3_gazebo` pkg). 
   * Run the `teleop_twist_keyboard` node (which belongs to `teleop_twist_keyboard` [pkg](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html)) from within this launch file to teleoperate the TurtleBot4.
3. Once you're satisfied with clarity of the generated map, manually [save the map](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/generate_map.html#save-the-map) as `sync_classroom_map`, before killing `mapper.launch.py`. 
4. Copy and paste the map files to `turtlebot3_gazebo/maps`


#### Part C: Path Finding Algorithms (1 hr) (Individual)

From the prior part, you should have 2 map files, one containing an image of the map(`*.pgm`)and another one containing meta-data of the map(`*.yaml`). Note: We have also provided 2 map files on Brightspace that you may use this week instead.

You will need these two files in order to implement your own A* algorithm. Go to the course site on Brightspace, and download the Jupyter Notebook. 

You will need to upload this file into [Google Colab](https://colab.research.google.com/) to work with it. Go through the jupyter notebook and implement the A* algorithm for the map obtained during previous tasks with the navigation stack.

Take this opportunity to learn how the BFS, Dijstra and A* algorithms are implemented. 

### Week 9: Navigation, Part I
In this lab, you will perform navigation in simulation, in preparation for hardware implementation next week.

* `task_7`- Autonomous navigation in a known environment

We will continue developing with the `task_7` `ament_python` ROS2 pkg developed in Week 8, Part B.

The task is to 
1. get the goal pose, 
2. use the A* algorithm previously created, and 
3. implement a path-follower to drive the vehicle through the path. 

The primary focus of this task should be the logic of the path planning and path follower. It is assumed that the path is free of obstacles.
 
#### Tasks: Simulation (2 hr) (Individual)
In this part of the lab, we will work with the robot in simulation. This allows you to test functionality before working with the robot in a field environment.

1. Within the `task_7` package, create a Python node called `auto_navigator.py`. We have provided skeleton code to help you get started. The `auto_navigator.py` node must do the following:
        
    1. Implement the path planning and path following logic to move the TurtleBot4 from position A to B.
    2. Maintain a record of the initial, goal, and current TB4 pose.
    3. Perform the A* path planning logic and a path follower logic which moves the TB4 through the generated path.
        
        *The path follower logic must make sure the TB4 passes through the waypoints with an acceptable error threshold for the heading and speed vectors.*

**Do not use preexisting packages like Nav2 to implement this node.**

Please give this node the alias `auto_navigator.py` in your `setup.py` file to start up the auto-navigator node. You will use this node both in simulation and in hardware (the launch files will be different).

Compared to other nodes, `auto_navigator.py` is predicted to have a relatively high number of lines of code. We recommend breaking your logic into subfunctions and following good coding styles.

After completing this node, 

2. Follow the instructions [here](https://github.com/naslab-projects/sim_ws.git) to use the simulation environment. Note the specific launch instructions in the `src/turtlebot3_gazebo` subdirectory.
3. Launch the `navigator.launch.py` file (which belongs to `turtlebot3_gazebo` pkg). 
3. Test the `auto_navigator.py` by pick a [target pose in RVIz](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/navigation.html). Make sure it can generate a path the goal and navigate towards the goal.
4. Record the `/scan`and `/cmd_vel` topics using rosbag (see Deliverables section at end of document).

### Week 10: Navigation, Part II

Last week, you implemented Turtlebot navigation capabilities in simulation. This week, you will implement it on the physical robot and demonstrate navigation capabilities in hardware.

#### Tasks: Hardware (2 hr) (Partnered)

You will use the `auto_navigator.py` node from last week (Week 9, Part A, Step 1). However, this week, you will add it to a launch file to be used to navigate on hardware with the physical robot.

1.  Write a launch file called `turtlebot4_navigator.launch.py` which performs the following:
          
    1. Launch the `nav_bringup.launch.py` file (which belongs to `turtlebot4_navigation` pkg).
        * This is for localization. 
        * You must also pass the relative path for the `map` parameter, which should be `src/task_7/maps/sync_room_map.yaml`.
    
    2. Launch the `view_robot.launch.py` file (which belongs to `turtlebot4_viz` pkg).
        * This allows you to view the map in Rviz2.
    
    3. Run the `auto_navigator` node (which belongs to `task_7` pkg).
        * This starts your path planning and path following algorithm for the TurtleBot4.

2. Complete the relevant tag details in the `package.xml` file.

3.  Build and run the ROS 2 node you built.
    * Observe the robot's path through the environment.
    * Also verify with these commands:
        * $`ros2 node list`
        * $`ros2 topic list`
        * $`ros2 node info <node_name>`
        * $`ros2 topic echo <topic_name>`
4. Once the robot can navigate successfully, record a video of the robot navigating. You will upload this to Gradescope later.

Note: it is assumed the path is free of obstacles. However, if your robot keeps colliding into the walls, you might need to increment the safe margin with respect to the walls or improve your map accuracy.

### Deliverables
Source code
* One ROS 2 package:
    * `task_7`- Generating map data + TB4 path planning and following

ros2 bag files (Week 9)
* You will 'record' the data passing through your topics during simulation via `ros2 bag`:
    * in any directory outside from your workspace directory, run:
        * $`mkdir bag_files`
        * $`cd bag_files`
        * $`ros2 bag record -o task_7 /scan /cmd_vel`
    * NOTE: your topic must be alive for ros2 to record it
    * Use this as [reference](https://docs.ros.org/en/galactic/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html)

ros2 log files
* Upload `<ws_ros2>/log/` too

Video footage
* Recording of your robot performing autonomous simulation when you are running it on the physical robot

Your folder structure should be as such:

```
XX
|
|__bag_files
|       |__task_7
|            |__metadata.yaml
|            |__task_7_0.db3
|            
|__task_7
|     |__launch
|     |__maps
|     |__...
|
|__log
```

Where XX must be replaced with your roll-number.

### Rubric
Deviating from the names provided in the lab sheet will result in penalties.
* 60 pts: `task_7` pkg
* 05 pts: Week 8, Part B, `ros2 bag` file
* 05 pts: Week 8, Part A, map files
* 20 pts: Week 8, Part C, completed and commented `*.ipynb` NB
* 05 pts: Week 9 `ros2 bag` files
* 05 pts: Week 10, video recording
