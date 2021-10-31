## Prerequisite repositories

* **navigation**: git@github.com:ros-planning/navigation.git

* **costmap_2d**: a repository modified by me found [here](). It has an additional plugin layer `lig local static layer(?)`.



## ToDo

* **tf2 orientation 채우기**: map2odom and odom2base_link의 orientation이 현재는 (w,x,y,z) = (1,0,0,0)으로 세팅되어 있음 이걸 고쳐야함. (SLAM 파트에서 받기)


## Running Turtlebot3 Simulation

prerequisite

1. 2dcostmap should have been created by SLAM(gmapping) 
2. Check local_costmap_params.yaml
3. Check costmap_common_params_burger.yaml and uncomment bottom 3 lines

```sh
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_house.launch
# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml #$HOME is where map.yaml is saved
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/suhu/map.yaml #$HOME is where map.yaml is saved
```

## Create Global Costmap 2D 

1. ./maps/drawMap.ipynb 열기 
2. dpi (dots per inches) = 100 by default
3. Figure size = 12x12
4. Therefore, 1200x1200 pixel인데
5. radius = 300 meters
6. Therefore, resolution (m/pixel) = 1200/600 = 2 m/pix
7. Center의 GPS는 현재 static_goal.gps설정의 latitude: 36.3683644417, longitude: 127.362330045
8. 그래서, Costmap에서의 GPS위치를 정확히 판단할 수 있음. (Global Map과 Odom의 Frame Transformation을 구할 수 있음.)
9. fig.save할 때 map이 저장되는데, 보통 여백과 함께 저장되기 때문에 ( 지름 != image.width )이 된다. 그래서 여백을 없애는 각종 function을 써야함.


## Running w/ Rosbag (only PathPlanning)

1. Check local_costmap_params.yaml
2. Check costmap_common_params_burger.yaml and comment bottom 3 lines

```sh
cd ~/catkin_ws
roslaunch lig_path_planning rviz.launch
rosbag play --clock lig....
roslaunch lig_path_planning node.launch
```

## Running w/ Rosbag w/ SLAM pose and Costmap 

1. Check local_costmap_params.yaml
2. Check costmap_common_params_burger.yaml and comment bottom 3 lines
3. rviz.launch에서 odom_simulator node argument 고치기
4. globalNlocal_costmap_params.yaml의 global_frame, robot_base_frame에서 `pp_` prefix 붙이기

```sh
cd ~/catkin_ws
roslaunch lig_path_planning rviz.launch
rosbag play --clock lig....
roslaunch lig_path_planning node.launch
```


## Running in Real Time in the Real World (only my package)

현재 GPS 받아서 Global Cost Map에서 위치를 찾아내고 -> local cost map은 없는 상태로 움직이게끔하기 (물체가 앞에 있다면 목표점을 다른데로 옮겨서 직접 피하는 방식을 선택해야할 듯 LiDAR에 안잡히는 위치에서 움직여야할듯함)

1. rviz.launch에서 `<node pkg="lig_path_planning" type="odom_simulator" name="my_odom_simulator" output="screen" args="slam" />`의 파트에서 args="slam"을 args="slam_false"로 바꾸기 ("slam"만 아니면됨)

2.

```sh
# 
# 1. Turn on GPS
## SOME LAUNCH COMMAND


# 2. Turn on global costmap and visualization
roslaunch lig_path_planning rviz.launch

# 3. Turn on path planning and command

# roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml #$HOME is where map.yaml is saved
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/suhu/map.yaml #$HOME is where map.yaml is saved
```

## Running in Real Time in the Real World (SLAM + T-map + PP)

