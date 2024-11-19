# motion_bt_nodes
A collection of BT nodes for motion capabilities

## How to test it in [tiago_harmonic](https://github.com/Tiago-Harmonic/tiago_harmonic) simulator

### Run de simulator
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=house
```

### Activate navigation
```bash
ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=small_house
```

### Run the example
```bash
ros2 launch motion_bt_nodes nav.launch.py 
```
