# Instructions to run Robotics Coursework Project

### Each Instruction will require a new terminal session along with the source directory being the current working directory.

## Task 1
### Instructions
- Execute the following:
    - `gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so ./Task1\ Environment.world`
    - `python dqn_gazebo.py 1`
    - `python dqn_environment.py`
    - `python dqn_test.py`

## Task 2

### Setup
- Execute the following:
    - `mkdir models && cd $_`
    - `wget http://download.tensorflow.org/models/object_detection/tf2/20200711/centernet_hg104_512x512_coco17_tpu-8.tar.gz`
    - `tar -xvf centernet_hg104_512x512_coco17_tpu-8.tar.gz`
    - `cd ..`
    - Download this folder https://lunet-my.sharepoint.com/:f:/g/personal/coarm_lunet_lboro_ac_uk/EhjTjTJaKy9HqM4e56AtQQYBovHaw_iPH2ornN8K1uAfgQ?e=Evlakc and store inside the source directory. This link contains the stored object detection model for use within this coursework.


### Instructions
- Execute the following:
    - Choose the appropriate world from the following list:
        - `gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so ./Task2\ Environment\ 1\ up\ left.world`
        - `gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so ./Task2\ Environment\ 2\ down\ left.world`
        - `gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so ./Task2\ Environment\ 3\ down\ right.world`
        - `gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so ./Task2\ Environment\ 4\ up\ right.world`
    - These commands can be run on the same terminal:
        - `python inject_drone.py`
        - `python drone_client.py`
        - `python dqn_gazebo.py 2`
    - `python dqn_environment.py`
    - `python dqn_test.py`
    - `python drone_controller.py`

