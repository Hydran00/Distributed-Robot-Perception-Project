# Distributed-Robot-Perception-Project
![Build Humble](https://github.com/Hydran00/Distributed-Robot-Perception-Project/actions/workflows/main.yml/badge.svg)  
### Installation
Clone into your workspace
```
cd <path/to/yoour/ws>/src
git clone --recursive https://github.com/Hydran00/Distributed-Robot-Perception-Project.git project
cd .. && colcon build --symlink-install
```

g++ -o integration integration.cpp -I /usr/include/eigen3/

### Launch sim
```
cd ~/CoppeliaSim
./coppeliaSim.sh
```
Then open the correct double_robot_sim.ttt under ur_coppeliasim from `File -> Open Scene`
### Launch ROS sim
- Full simulation:
    ```
    ros2 launch project full_simulation.launch.py
    ```
- Voronoi calculator
    ```
    ros2 run project voronoi_calculator
    ```