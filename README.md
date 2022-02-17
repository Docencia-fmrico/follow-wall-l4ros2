[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=6883246&assignment_repo_type=AssignmentRepo)

[![GitHub Action
Status](https://github.com/Docencia-fmrico/follow-wall-l4ros2/workflows/main/badge.svg)](https://github.com/Docencia-fmrico/follow-wall-l4ros2)
[![codecov](https://github.com/Docencia-fmrico/follow-wall-l4ros2/main/graph/badge.svg)](https://codecov.io/gh/Docencia-fmrico/follow-wall-l4ros2)

# Follow Wall task

In this task the robot will approach a wall and follow its countour.


# Members of L4ROS2

- Javier de la Canóniga: @javi-dbgr
- Iván López: @ivrolan
- Alejandro Moncalvillo: @Amglega
- Unai Sanz: @USanz


# Node diagram

![scheme](./wall_follower_diagram.png)

# Sensing node scheme

![scheme](./sensing_node_scheme.png)


# Actuation node diagram (state machine)

![scheme](./state_machine_diagram.png)

# Examples of behavior
You can find the full lenght videos in [this link](https://urjc-my.sharepoint.com/:f:/g/personal/a_moncalvillo_2019_alumnos_urjc_es/EnP7S7rvn4tAjBLFINh7QjkBLc0BIaSQbftx8fgklbykBQ?e=24uD1) 

### Initial conditions: Corner
![corner-init-vid](./corner_init.gif)

### Initial conditions: No wall near
![no-wall-vid](./no_wall_init.gif)

### Testing the algorithm in real life with the Kobuki robot

The code can be found in the branch `kobuki_irl`


![kobuki_irl](./kobuki_irl_test_480p.gif)

