# UV LARM - TARS
### Name of the students :
- Thibault ROUX
- Arthur MATA

### To run a node :
- Make a build : `colcon build`
- Run the setup : `source install/setup.bash`
- Start the node : `ros2 run tuto_kickoff myNode.py`

### To play a slam bagfile - Example
- Launch the simulation
- Run slam_toolbox offline : `ros2 launch slam_toolbox offline_launch.py use_sim_time:=False`
- Play the bag (-r option is for the speed) : `ros2 bag play -r 10 slam-challenge-1`
