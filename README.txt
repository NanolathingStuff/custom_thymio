https://github.com/jeguzzi/ros-aseba/blob/client/thymio_description/urdf/base.urdf.xacro#L66 is required
be also sure to replace urdf/base.urdf.xacro on that repository with the one coming with this repo

remember to use: $"chmod 777 -R *" (and eventually "catkin_make") in source folder in order to be able to use the launchers

Launch with:
roslaunch custom_thymio racing_thymio_gazebo.launch name:=thymio10 world:=$WORLD_NAME type:=$CONTROLLER_NAME.py 
es CONTROLLER_NAME: controller_straight.py ; WORLD_NAME: circuit1

examples: 
        roslaunch custom_thymio racing_thymio_gazebo.launch name:=thymio10 world:=straight_test type:=controller_straight.py
        roslaunch custom_thymio racing_thymio_gazebo.launch name:=thymio10 world:=circuit3 type:=controller_open_loop.py


Controller list:
    - controller_straight (just bring the thymio straight and print the wheels's sensors data)
    -

Worlds list:
    - world_straight (world with a straight corridor directed 90 from the thymio)
    - straigth_test (world with a straight corridor directed as the thymio, walls are not at the same distance: used for test the sensors)
    - circuit1 (a test cisrcuit world with random parts)
    - circuit2 (a narrow loop circuit)
    - circuit3 (circular circuit done with 2 left turns)
    
edit worlds with: $ "gazebo path/to/file.world"
test if the odometry lap-detector [209-219 in almost all cntrollers] works
