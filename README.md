# LIGNex1


## Installation
```console
sudo apt-get install ros-melodic-jsk-recognition-msgs

sudo apt-get install ros-melodic-velodyne
```


---

## Execution
- Launch ouster (add ```replay:=true``` for replaying)

```console 
roslaunch ouster_ros ouster.launch
```

- Launch receive gps

```console
roslaunch receive_gps receive_gps.launch
```

- Resize images

```console
roscd lvi_sam && cd launch && bash resize.sh
```

- Run LVI-SAM

```console
roslaunch lvi_sam run.launch
```


