from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import sys,traceback
import multiprocessing
from PIL import Image
from medbot_nav import *
from time import sleep
import cv2
import numpy as np
import math

#waypoints for the bot to follow
waypoints = [(21.49,1.00,0.00),(21.49,1.00,4.71),(14.56,1.00,4.71),(14.56,1.00,6.28),(14.56,-8.8,6.28),
			(14.56,-8.8,3.14),(14.56,2.00,3.14),(14.56,1.00,4.71),(11.07,1.00,4.71),(11.07,1.00,6.28),(11.07,-6.00,6.28),
			(11.07,-6.00,4.71),(2.77,-6.00,4.71),(2.77,-6.00,6.28),(2.77,-8.00,6.28)] # x +- 4 y +- 4

# waypoints = [(4.56,1.00,3.14),(14.56,1.00,4.71),(11.07,1.00,4.71)]

# FSM VARIABLES INITIALIZE
cnt = 0
stat = 0
prev_motor_speed = 0

# FUNCTION TO CONTROL THE MEDBOT
def move_bot(sim):
	global bot_or,cnt,prev_motor_speed

	# define 
	medbot = sim.getObject('/Medbot')
	l_motor = sim.getObject('/Medbot/r_joint')
	r_motor = sim.getObject('/Medbot/l_joint')

	relativeToObjectHandle = sim.handle_world
	position = sim.getObjectPosition(medbot, relativeToObjectHandle)
	orientation = sim.getObjectOrientation(medbot, relativeToObjectHandle)

	bot_or = orientation[2]
	if orientation[2] < 0:
		bot_or = 6.28 + orientation[2]
	
	# DEFINE X AND Y TARGET FROM THE WAYPOINTS ARRAY
	x_target = waypoints[cnt][0]
	y_target = waypoints[cnt][1]
	or_target = waypoints[cnt][2]

	error_x = x_target - position[0]
	error_y = y_target - position[1]
	error_or = or_target - bot_or
	
	# START NAVIGATION
	if cnt == 0:
		motor_speed = goToY(position[1],y_target)
		sim.setJointTargetVelocity(l_motor,motor_speed)
		sim.setJointTargetVelocity(r_motor,-motor_speed)
		if abs(error_y) < 3:
				cnt += 1

	else :
		if x_target != waypoints[cnt - 1][0]:
			# call the x coordinate navigator
			motor_speed = goToX(position[0],x_target)
			sim.setJointTargetVelocity(l_motor,motor_speed)
			sim.setJointTargetVelocity(r_motor,-motor_speed)
			if abs(error_x) < 3:
				cnt += 1

		if y_target != waypoints[cnt - 1][1]:
			# call the y coordinate navigator
			motor_speed = goToY(position[1],y_target)
			sim.setJointTargetVelocity(l_motor,motor_speed)
			sim.setJointTargetVelocity(r_motor,-motor_speed)
			if abs(error_y) < 3:
				cnt += 1

		if or_target != waypoints[cnt - 1][2]:
			# call the yaw angle navigator
			motor_speed = goToOrientation(bot_or,or_target)
			sim.setJointTargetVelocity(l_motor,motor_speed)
			sim.setJointTargetVelocity(r_motor,motor_speed)
			if abs(error_or) < 0.1:
				cnt += 1
	
	print(cnt)

# FUNCTION TO GET THE SAMPLE POSITION ON THE STEWART PLATFORM
def stewart_cam(sim):
	# initialize the camera in the simulation
	cam=sim.getObject('/Medbot/stewartPlatform/Top_Cam')

	# initialize HSV colour range
	lower_color = np.array([0, 150, 150])
	upper_color = np.array([10, 255, 255])
	flag=1

	# time.sleep(0.01)

	# obtain the image from the camera
	img,res=sim.getVisionSensorImg(cam)
	img = Image.frombytes('RGB',(256,256),img)
	frame=np.array(img)
	hsv_frame= cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
	if flag:
		cv2.imwrite('img.png',frame)
		flag=0

	# mask the red colour of the ball on the platform
	color_mask = cv2.inRange(hsv_frame, lower_color, upper_color)

	# get the contour points and find the center of the masked region
	contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours) > 0:
		largest_contour = max(contours, key=cv2.contourArea)
		M = cv2.moments(largest_contour)
		if M["m00"] != 0:
			cX = int(M["m10"] / M["m00"])        
			cY = int(M["m01"] / M["m00"])
			print(cX,cY)

			# return the PIXEL COORDINATES
			return [cX,cY]

	else:
		return [0,0]
		print("BALL NOT FOUND")

# FUNCTION TO BALANCE THE SAMPLE ON THE PLATFORM USING PID 
def stewart_control(sim):
	global prev_p1,prev_p2,prev_p3

	# INITIALIZE VARIABLES FOR STEWART PLATFORM PID
	I1,I2,I3 = 0,0,0
	D1,D2,D3 = 0,0,0
	# Kp,Ki,Kd = 0.0095,0.00,0.007
	Kp,Ki,Kd = 0.014,0.00,0.0095
	
	print("BALANCING")

	# define objjects of stewart platform for COPPELIASIM
	medbot = sim.getObject('/Medbot')
	motor2 = sim.getObject('/Medbot/stewartPlatform/motor2')
	motor3 = sim.getObject('/Medbot/stewartPlatform/motor3')
	motor4 = sim.getObject('/Medbot/stewartPlatform/motor4')
	motor5 = sim.getObject('/Medbot/stewartPlatform/motor5')
	motor6 = sim.getObject('/Medbot/stewartPlatform/motor6')
	motor1 = sim.getObject('/Medbot/stewartPlatform/motor1')

	# get the pixel coordinates and it to x y coordinate system
	x_curr = (stewart_cam(sim)[0] - 112)/100
	y_curr = (stewart_cam(sim)[1] - 117)/100

	# calculate the perpendicular distance of the ball from each of the actuator axes
	p1 = -y_curr
	p2 = (y_curr - (1.732 * x_curr))/2
	p3 = (y_curr + (1.732 * x_curr))/2

	# integral
	I1+=p1
	I2+=p2
	I3+=p3

	# differential
	D1 = p1 - prev_p1
	D2 = p1 - prev_p2
	D3 = p1 - prev_p3

	# PID with the output of the length that each of the prismatic joint should move
	l1 = p1 * Kp + D1 * Kd + I1 * Ki
	l2 = p2 * Kp + D2 * Kd + I2 * Ki
	l3 = p3 * Kp + D3 * Kd + I3 * Ki

	# set the prismatic joint of each of the actuators of the STEWART PLATFORM
	sim.setJointPosition(motor3,l1)
	sim.setJointPosition(motor5,l1)
	sim.setJointPosition(motor1,-l3)
	sim.setJointPosition(motor6,l3)
	sim.setJointPosition(motor2,l2)
	sim.setJointPosition(motor4,l2)
	
	prev_p1 = p1
	prev_p2 = p2
	prev_p3 = p3

# COPPELIASIM 
#main
if __name__ == "__main__":
	client = RemoteAPIClient()
	sim = client.getObject('sim')

	try:
		try:
			return_code = sim.startSimulation()
			if sim.getSimulationState() != sim.simulation_stopped:
				print('\nSimulation started correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be started correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be started !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

		try:
			while True:
				# in an infinite loop call the functions stewart_control and move_bot
				stewart_control(sim)
				move_bot(sim)

		except Exception:
			print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually if required.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

		try:
			return_code = sim.stopSimulation()
			time.sleep(0.5)
			if sim.getSimulationState() == sim.simulation_stopped:
				print('\nSimulation stopped correctly in CoppeliaSim.')
			else:
				print('\nSimulation could not be stopped correctly in CoppeliaSim.')
				sys.exit()

		except Exception:
			print('\n[ERROR] Simulation could not be stopped !!')
			traceback.print_exc(file=sys.stdout)
			sys.exit()

	except KeyboardInterrupt:
		return_code = sim.stopSimulation()
		time.sleep(0.5)
		if sim.getSimulationState() == sim.simulation_stopped:
			print('\nSimulation interrupted by user in CoppeliaSim.')
		else:
			print('\nSimulation could not be interrupted. Stop the simulation manually .')
			sys.exit()
