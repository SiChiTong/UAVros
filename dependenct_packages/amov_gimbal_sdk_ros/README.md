# 阿木实验室G1吊舱ROS SDK 

## 介绍

1. 阿木实验室G1吊舱是一款高性能低成本的光学吊舱
2. 智能吊舱=云台+相机+AI芯片+人机交互软件+深度学习
3. ROS SDK(GCC 7.5.0)  

## 下载SDK：

1. 克隆仓库 

   `git clone https://gitee.com/amovlab/gimbal-sdk-ros.git`  

## 环境

我们在以下系统环境完成了测试：

1. 硬件平台：  Allspark（核心板：Jetson NX）
2. 操作系统  ：Ubuntu 18.04
3. C++版本：C++11 
4. ROS 版本：melodic
5. OpenCV：3.3.1

## 构建 Amov Gimbal SDK

如果你没有工作空间，就按照下面的命令进行创建并编译ROS SDK

```shell
mkdir ‐p ~/catkin_ws/src 
cd ~/catkin_ws/src
git clone https://gitee.com/amovlab/gimbal-sdk-ros.git
cd gimbal-sdk-ros
git submodule update --init  #更新子模块
cd ../.. 
catkin_make #编译SDK
```


## 运行示例

1. 首先进入`catkin_ws`目录

2. 通过`source devel/setup.bash`命令，把ROS SDK功能包加入到环境变量中

3. 运行示例：

   1.运行吊舱节点（通过`/amov_gimbal_ros/gimbal_control`话题控制吊舱、通过`/amov_gimbal_ros/gimbal_state`话题获取吊舱状态信息）

   ```
   roslaunch amov_gimbal_sdk_ros gimbal_G1.launch
   ```

   - 通过`/amov_gimbal_ros/set_camera_action`服务控制相机（0：录像和停止录像  ）视频会保存在TF卡中

   2.获取相机视频画面（通过`/amov_gimbal_ros/gimbal_image`话题获取ROS image图像）
   
   ```
   roslaunch amov_gimbal_sdk_ros gimbal_image_G1.launch
   ```

   

## 节点说明

|    节点     |              话题名称              |              服务名称              |
| :---------: | :--------------------------------: | :--------------------------------: |
| gimbal_node |   /amov_gimbal_ros/gimbal_state    | /amov_gimbal_ros/set_camera_action |
|             |  /amov_gimbal_ros/gimbal_control   |                                    |
| camera_node | /amov_gimbal_ros/amov_camera_image |                                    |



## 联系我们

- 阿木实验室官网：https://www.amovlab.com/
- 阿木实验室论坛：https://bbs.amovlab.com/

