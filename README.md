# UV LARM - TARS

### Name of the students :

- Thibault ROUX
- Arthur MATA

## Package GRP_TARS :

Ce package correspond au challenge 1.

### Launchfiles :

Il y a 3 launchfiles différents :

- `simulation_launch.yaml`: Lance une simulation dans laquelle le robot se déplace de manière autonome.
- `tbot_launch.yaml`: Le robot réel se déplace et détecte les bouteilles. Lorsqu'une bouteille est détectée, un message de type `String` est publié dans le topic `/detection`.
- `visualize_launch.yaml` : Il est équivalent à `tbot_launch.yaml`, en lançant également **Rviz**, et en publiant les images de la caméra vers **Rviz**.

### Tu run a launchfile :
```
ros2 launch grp_tars <launchfile>
```

## Tutos

### To run a node :

- Make a build : `colcon build`
- Run the setup : `source install/setup.bash`
- Start the node : `ros2 run tuto_kickoff myNode.py`

### To play a slam bagfile - Example

- Launch the simulation
- Run slam_toolbox offline : `ros2 launch slam_toolbox offline_launch.py use_sim_time:=False`
- Play the bag (-r option is for the speed) : `ros2 bag play -r 10 slam-challenge-1`

### Find the logs

When we use the `print` instruction in Python, the output is written in the log files.
The log files can be found at the root of the user workspace (not the ROS workspace), in the hidden folders.
