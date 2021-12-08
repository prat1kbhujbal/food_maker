# Food Making Robot

## Overview
The project demonstrates a robot preparing a pizza. Robot will streamline the tedious and repetitive process of pouring different kinds of toppings on pizzas and then placing them in a conveyor oven which will then be ready to serve.


## Dependencies
[![Ubuntu 18.04](https://img.shields.io/badge/Ubuntu18.04-Clickhere-brightgreen.svg?style=flat)](https://releases.ubuntu.com/18.04/)  
[![ROS Melodic](https://img.shields.io/badge/ROSMelodic-Clickhere-blue.svg?style=flat)](http://wiki.ros.org/melodic/Installation/Ubuntu)


## Clone and Build your package
```bash
mkdir -p <your_workspace_name>/src
cd <your_workspace_name>/src
git clone https://github.com/Prat33k-dev/walker_turtlebot.git
cd <your_workspace_name>
catkin_make
```

## Launching cafe world and node to move arms
1. Launching world using launch file
    ```bash
    cd <your_workspace_name>
    source devel/setup.bash
    roslaunch food_maker world.launch
    ```
2. For running publisher node to move arms (in new terminal)
    ```bash
    cd <your_workspace_name>
    source devel/setup.bash
    rosrun food_maker move.py
    ```
## [Demonstration Video](https://drive.google.com/file/d/1ToFM6Ej-sCzrsyW8UmaYlT9Ouy-rchKu/view?usp=sharing)

## Cad model
Following is the link to cad models of the robot - [link](https://drive.google.com/drive/folders/1eIMCfKIN6-10OEwQIi32q56CKl-o1Jfy?usp=sharing)