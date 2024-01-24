# UV LARM - TARS

**Notes** : Tous les noeud ROS ont été écrits en Python, et les launch files en YAML.

### Name of the students :

- Thibault ROUX
- Arthur MATA

## Package GRP_TARS :

Ce package correspond au challenge 2. Il comporte plusieurs launch files.

### Launchfiles :

Il y a 3 launch files différents :

- `simulation_launch.yaml`: Lance une simulation dans laquelle le robot se déplace de manière autonome.
- `tbot_launch.yaml`: Le robot réel se déplace et détecte les bouteilles. Lorsqu'une bouteille est détectée, un message de type `String` est publié dans le topic `/detection`. Ses paramètres ROS diffèrent de ceux de `simulation_launch.yaml`.
- `visualize_launch.yaml` : Il est équivalent à `tbot_launch.yaml`, en lançant également **Rviz**, et en publiant les images de la caméra vers **Rviz**.
- `static_launch.yaml` : Il est équivalent à `visualize_launch.yaml`, mais le robot ne se déplacera pas. Il a donc uniquement pour but d'effectuer des tests sur la détection de bouteille, sans être gêné par les mouvements du robot.

## Package Tuto_kickoff

Ce package ROS a été créé uniquement pour s'entraîner. Il comporte un noeud python basique, qui peut être exécuté via la commande `ros2 run` dans un terminal.

## Package Tuto_POO

Ce package contient des noeuds Python écrit en suivant le paradigme de **Programmation orientée objet**. Il a servi à la fois pour s'entraîner, et faire une première version de l'algorithme de déplacement autonome du robot, évitant les obstacles.

Le launch file `launch_sim.yaml` permet de lancer le noeud ROS `reactive_move.py`.

## Package Tuto_vision

Ce package comporte une première version du script servant à détcter la présence de bouteille. Il a été utile pour tester les différentes fonctionnalités liées à la caméra et au traitement d'images.

## Astuces

### To run a node :

- Make a build : `colcon build`
- Run the setup : `source install/setup.bash`
- Start the node : `ros2 run tuto_kickoff myNode.py`

- ### Tu run a launchfile :
```
ros2 launch grp_tars <launchfile>
```

### To play a slam bagfile - Example

- Launch the simulation
- Run slam_toolbox offline :
```
ros2 launch slam_toolbox offline_launch.py use_sim_time:=False
```
- Play the bag (-r option is for the speed) : `ros2 bag play -r 10 slam-challenge-1`

### Find the logs

When we use the `print` instruction in Python, the output is written in the log files.
The log files can be found at the root of the user workspace (not the ROS workspace), in the hidden folders.
