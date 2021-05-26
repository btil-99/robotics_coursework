# Instructions to run Robotics Coursework Project

### Each Instruction will require a new terminal session along with the source directory being the current working directory.

## Task 1

- Execute the following:
    - `gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so ./Task1\ Environment.world`
    - `python dqn_gazebo.py 1`
    - `python dqn_environment.py`
    - `python dqn_test.py`

## Task 2
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