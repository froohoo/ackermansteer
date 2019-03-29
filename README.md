# ackermansteer



### Notes to self / Learnings:
* SetPosition worked abysmally to set wheel angles. I'm assuming because the resulting discontinuity requires temporary interuption of the simulation... not sure need to research.
* PID controller is good for setting forces on wheels (runs much smoother). Not sure though if this is the current 'best practice' for setting the joint angles as I understand the joints have embedded PID controllers... but havent found the documetnation on these yet.
* For some reason, specifiying gazebo and gazebo_plugins as a catkin component does not allow ommission of specifying GAZEBO_INCLUDE_DIRS and GAZEBO_LIBRARIES in the include and link_directories build header file locations... would have thought they would show up under a catkin_.... make variable.
