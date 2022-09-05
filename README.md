# Awesome Dog 
Welcome to our repository  for AutonomousQuadruped. Here you will find out how to run our project.

### Getting Started  

1. Clone the repository and `cd` to the `AutonomousQuadruoped` folder.  

2. Before building, you may need to install some ros packages. In one terminal, run:  
    ```
    sudo apt-get install ros-noetic-octomap ros-noetic-octomap-server ros-noetic-octomap-ros ros-noetic-octomap-rviz-plugins ros-noetic-navigation
    ```

3. Then build the project, run `catkin build`. The first build may take a few minutes, as the unity files needed for the project will be downloaded automatically and you **don't** need to download and move them to the specified folder manually. 
4. You'll need 2 terminals to launch all the nodes. Remember to source the environment when you launch a new terminal. 
    ```
    source/devel/setup.bash
    ```

    - In first terminal, execute  
    ```
    roslaunch simulation simulation.launch
    ```
    Several nodes, a Unity file and RVIZ would be launched.  

    
    - In second terminal, execute 
    ```
    roslaunch planning planning.launch
    ```
    Controller and planner would be launched and the robot dog will perform its tasks automatically. In this terminal window, you can get information about the steps/slopes detected by the robot and the task it is performing, and when the robot reaches the end, you can see the time taken for the whole process.   


5. In the rviz window, you'll see a thin green line (the path designed by the global path planner), a solid orange line (the actual trajectory of the robot) and a small dynamic red dot (the current target waypoint of the robot dog). Note that because the robot is unfamiliar with the map at the beginning, the globally planned path may be unreasonable, but as the robot moves forwards and the occupancy map is gradually improved, the path will become reasonable under most cases. Besides, the current target point of the robot may be obtained based on the result of path planning at some previous moment, which means that it may not be on the currently planned path.

6. You are expected to see something as this [video](https://drive.google.com/file/d/1SIjrDl93RdoaB7c93GNVHmsPp5w93V0a/view?usp=sharing) or [this one](https://drive.google.com/file/d/1Ev4BGYuUePXea4E4gV0mWkTVYtVBBhBW/view?usp=sharing) with 8x speed.

### Tips

- This project is developed on UBuntu 20.04.4LTS with ROS noetic, it is **NOT** guaranteed to work on other Ubuntu or ROS distributions.
- Our program will dynamically generate point cloud, occupancy grid and perform path planning in real time, which places high demands on computer computer performance. Therefore, it is **highly recommended** to run the project on a **native** Ubuntu machine rather than a virtual machine. If the program is overloaded while running (as you may see in the first terminal), try to reduce the frequency of costmap updates in the `/src/simulation/param/global_costmap_params.yaml` (line 5 and 6).  However, this may have an impact on the robot's performance.  
- Global planner may produce a straight line towards destination with very low possibility. If our waypoint publisher accidentally selects point on this path, then the mission may fail. Upon this condition, we recommend you to rerun the program.

