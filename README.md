# ackermansteer

A gazebo plugin to implement ackerman steering angles on a 4 wheeled robot. Note that this plugin just computes
and sets the correct angles to minimize wheel slip. It does not simulate an actual Ackermann linkage setup. 
Wheel angles and speeds are set using PID instances for both. 

### A video of current progress...
![Current Progress](InitialResults.gif)

### Notes to self / Learnings:
* SetPosition worked abysmally to set wheel angles. I'm assuming because the resulting discontinuity requires temporary interuption of the simulation... not sure need to research.
* Ackermann is actually spelled with 2 n's.
* PID controller is good for setting forces on wheels (runs much smoother). Not sure though if this is the current 'best practice' for setting the joint angles as I understand the joints have embedded PID controllers... but havent found the documetnation on these yet.
* For some reason, specifiying gazebo in makefile as a catkin component does not work. Need to find_package separately, and include/link separately as well. 
* If you get a `IGN_MASSMATRIX3_DEFAULT_TOLERANCE` error then you probably need to do a `apt-get install libignition-math2-dev`

### Steering angle equations:


<a href="https://www.codecogs.com/eqnedit.php?latex=Front&space;Left&space;Angle&space;=&space;\arctan&space;\left&space;[\frac{2.0&space;*&space;Wheelbase&space;*&space;sin(\phi&space;)}{2.0&space;*&space;Wheelbase&space;*&space;cos(\phi)&space;-&space;WheelSeparation&space;*&space;sin(\phi)}&space;\right&space;]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Front&space;Left&space;Angle&space;=&space;\arctan&space;\left&space;[\frac{2.0&space;*&space;Wheelbase&space;*&space;sin(\phi&space;)}{2.0&space;*&space;Wheelbase&space;*&space;cos(\phi)&space;-&space;WheelSeparation&space;*&space;sin(\phi)}&space;\right&space;]" title="Front Left Angle = \arctan \left [\frac{2.0 * Wheelbase * sin(\phi )}{2.0 * Wheelbase * cos(\phi) - WheelSeparation * sin(\phi)} \right ]" /></a>
    
<a href="https://www.codecogs.com/eqnedit.php?latex=Front&space;Right&space;Angle&space;=&space;\arctan&space;\left&space;[\frac{2.0&space;*&space;Wheelbase&space;*&space;sin(\phi&space;)}{2.0&space;*&space;Wheelbase&space;*&space;cos(\phi)&space;&plus;&space;WheelSeparation&space;*&space;sin(\phi)}&space;\right&space;]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Front&space;Right&space;Angle&space;=&space;\arctan&space;\left&space;[\frac{2.0&space;*&space;Wheelbase&space;*&space;sin(\phi&space;)}{2.0&space;*&space;Wheelbase&space;*&space;cos(\phi)&space;&plus;&space;WheelSeparation&space;*&space;sin(\phi)}&space;\right&space;]" title="Front Right Angle = \arctan \left [\frac{2.0 * Wheelbase * sin(\phi )}{2.0 * Wheelbase * cos(\phi) + WheelSeparation * sin(\phi)} \right ]" /></a>

These equations work as is for all input steering angles where -pi/2 < input angle < pi/2 and are graphed below. When input angle is negaative, Right wheel is the inner wheel and Left wheel is inner both angles are negative but Right wheel has a greater angle since it tracks a circular path of smaller radius. Same logic applies to positive turns as can be seen in positive turns, left wheel has a greater angle than the right. As we approach 0 steer angle, both angles converge to 0. (This plot assumes wheelbase=Track=1).

<img src="https://github.com/froohoo/ackermansteer/blob/master/SteerAngle.png" height="300">

