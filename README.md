# LIGNex1


## Installation
```bash
sudo apt-get install ros-melodic-jsk-recognition-msgs
```

```bash
sudo apt-get install ros-melodic-velodyne
```


---
## Preliminary
- How to setup metadata for os0
```bash
roscd ouster_ros && cp os-122039000077.local.json ~/.ros
```

---

## Execution
- Launch ouster (add ```replay:=true``` for replaying)

```bash 
roslaunch ouster_ros ouster.launch
```

- Launch receive gps

```bash
roslaunch receive_gps receive_gps.launch
```

- Resize images

```bash
roscd lvi_sam && cd launch && bash resize.sh
```

- Run LVI-SAM

```bash
roslaunch lvi_sam run.launch
```


