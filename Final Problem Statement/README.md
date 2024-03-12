# Ka-Boom-Robotrix-2024
ROBOTRIX FINAL PROBLEM STATEMENT


ht

https://github.com/Aerokrishna/Ka-Boom-Robotrix-2024/assets/122991458/96f39534-97f6-404c-934f-8f09fdf716c9

tps://github.com/Aerokrishna/Ka-Boom-Robotrix-2024/assets/122991458/84ff4537-9078-458c-a3fb-b8d8ba91bbd6


This branch contains two python scripts medbot_nav.py and medbot_sim.py. The medbot_nav python file contains a PID code for the robot to navigate autonomously. The medbot_sim contains the algorithm for the medbot to move the medbot while computing the inverse kinematics of the stewart platform to balance the sample(ball) while motion.

1) medbot_nav.py
   This code contains three seperate functions to execute a proportional control for x, y and theta motion. Each of these functions take the target x, y and theta values and give the motor speed of the bot computed by the controller. These functions are imported in the medbot_sim.py scipt to efficiently navigate the bot. We can also change the Kp value which acts as a parameter to tune the motor speed as per requirement.

2) medbot_sim.py
This code executes the navigation of the medbot, simultaneously keeping the sample on the stewart platform stable. Following are the main functions defined in this code:

i) move_bot()
This function defines the objects and joints that need to be controlled in the simulation. A waypoints array is defined which is an array of tupples containing the x, y and theta coordinates. This waypoints array is used as a target point provider which is directly fed into the medbot_nav function to calculate the linear and angular velocity of the medbot. A counter is 'cnt' is used to keep track of which waypoint the bot is currently following. This variable is increamented every time a waypoint is reached. A series of if statements help the medbot navigate sequentially while following the pre-defined waypoints.

ii) stewart_cam()
This function gets the pixel coordinates of the ball placed on the platform of the medbot. It starts by initializing the camera object with respect to the simulation. It captures an image in every loop to extract data of the ball's position in its frame. The center of the ball is tracked using the concept of "colour tracking". The red colour of the ball is masked out from the rest of the image. Then contours are added around the masked region. The center of the largest contour is considered and returned as pixel coordinates.

iii) stewart_control()
This function is used to compute actuation length of each of the prismatic joints attached to the platform. It takes the pixel coordinates(the stewart_cam function is called in every iteration) of the sample as input and computes 6 lengths for the 6 prismatic joints. The perpendicular distance of the ball from each of the rotating axes is calculated and fed it into a PID control to get the desired acctuation lengths of the joints. Since the actuators are placed 120 degrees apart on the circular plate calculating the angle with which the plate should rotate in each of the actuator axes is important. The perpendicular distance of the ball from the axes is calculated by applying the formula of 'perpendicular distance of a point from a line'.
             p = (Ax + By + C)/sqrt(A^2 + B^2)
The calculated p1 p2 and p3 are used in the PID controller to minimize their values to zero and keep the ball at the center of the plate.
Once the individual lengths are obtained from the PID, they are passed into the sim.setJointPosition function to simulate the movement of the platform.

Now the stewart_control and move_bot functions are called in an infinite loop to simulate the medbot navigate to the checkpoints autonomously while keeping the sample on stewart platform stable.

BEFORE READER EXECUTES THE CODE
The following code works exactly as it should. But unfortunately due to a glitch in coppeliasim it is not able to go beyond a certain point.

The video attached in this readme file contains the simulation of the medbot moving to the first checkpoint while keeping the sample stable.

