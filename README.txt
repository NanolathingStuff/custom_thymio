To clone	https://github.com/jeguzzi/ros-aseba.git	is required
replace the file "ros-aseba/thymio_description/urdf/base.urdf.xacro", contained in the repository cloned according to the instructions in the previous line with the file "urdf/base.urdf.xacro" contained in our repository
src/ros-aseba/thymio_description/urdf$ chmod 777 base.urdf.xacro 


remember to use: $"chmod 777 -R *" (and eventually "catkin_make" and/or "~/.bashrc file") in source folder in order to be able to use the launchers

Launch with:
roslaunch custom_thymio racing_thymio_gazebo.launch name:=thymio10 world:=$WORLD_NAME type:=$CONTROLLER_NAME.py 
es CONTROLLER_NAME: controller_straight.py; WORLD_NAME: circuit1

examples: 
        roslaunch custom_thymio racing_thymio_gazebo.launch name:=thymio10 world:=straight_test type:=controller_straight.py
        roslaunch custom_thymio racing_thymio_gazebo.launch name:=thymio10 world:=circuit3 type:=controller_open_loop.py


Controller list:
    - controller_straight (just bring the thymio straight and print the wheels's sensors data)
    -

Worlds list:
    - world_straight (world with a straight corridor directed 90 from the thymio)
    - straigth_test (world with a straight corridor directed as the thymio, walls are not at the same distance: used for test the sensors)
    - circuit1 (a test cisrcuit world with random parts) DO NOT USE
    - circuit2 (a narrow loop circuit)
    - circuit3 (circular circuit done with 2 right turns)
    - circuit3 (circular circuit that seems to let the PID finish every lap)
    
edit worlds with: $ "gazebo path/to/file.world"
robot seems to go crazy if trns too fast or the curve is large
Limit max turn because sometimes it turned to itself and becaume upside-down
just lateral distances might not be sufficient?

test roslaunch custom_thymio racing_thymio_gazebo.launch name:=thymio10 world:=straight_test type:=controller_PID.py

other test might be made with a pair of distace sensors (one already implemented and one angled)
copy the file "urdf/my_robot.xacro" contained in our repository replace the file "ros-aseba/thymio_description/urdf/base.urdf.xacro", contained in the repository cloned (you obiouvsly need to rename the file my_robot.urdf to base.urdf.xacro)
src/ros-aseba/thymio_description/urdf$ chmod 777 base.urdf.xacro 
for now, only controller_proportional2.py support it