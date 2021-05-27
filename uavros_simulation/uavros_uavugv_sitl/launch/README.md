### Common Commands

```
roslaunch uavros_uavugv_sitl uavrover_sitl.launch
```

```
roslaunch uavros_uavugv_sitl uavrover_valley_sitl.launch
```

These commands will launch two drones and one rover formation in different simulation environment.

```
roslaunch uavros_uavugv_sitl uavugv_sitl.launch
```

uavugv_sitl.launch is used to launch two drones and one racecar formation which is outdated.

If you want to test single controller of Ackermann car or drone, use the following commands:

```
roslaunch uavros_uavugv_sitl singleUAV_circle_sitl.launch
```

```
roslaunch uavros_uavugv_sitl singleRover_sitl.launch
```

other launch files may be outdated. You can try them and adjust the parameters though.