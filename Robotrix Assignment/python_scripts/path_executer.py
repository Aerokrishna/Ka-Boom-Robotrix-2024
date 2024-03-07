#import the custom kp path planner
from path_planner import *
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import sys,traceback
import multiprocessing
from PIL import Image

# describe the home and target nodes
home = 0
goal = 2

go_to_destination = 0

ideal_path = []

#represents all the paths connecting the pick-up points
path_id = [(0,1),(0,3),(1,3),(1,4),(1,2),(3,4),(2,4),(3,0),(4,3)]
#the respective coordinates
target = [(-0.50,0.57,1.57),(-0.5,-2.37,0),(-0.5,-2.37,0),(1.5,-1.37,4.71),
		  (2.42,1.57,3.14),(1.5,-1.37,4.71),(1.5,-1.37,4.71),(-2.55,-1.83,4.71),(-0.5,-2.37,0)]

#some variables for the navigation
stat = 0
exec = 0
waypoint = 0

#function that returns the ideal path with the coordinates of the nodes to be sequentially traversed
def get_ideal_path():
		global waypoint,home,goal,go_to_destination

		#call the path planner class
		path = kp(home,goal)
		if go_to_destination == 0:
			pass
		if go_to_destination == 1:
			home = 2
			goal = 0
			path.curr = home
			path.open_nodes = [0,2,3,4]
			path.closed_nodes = [1]

		#get the shortest path 
		path.get_fcost((path.vertices[home]),path.vertices[goal])
		shortest_path = path.get_shortest_path(home,goal)

		#get the ideal path with the coordinates to be traversed
		for i in range(len(shortest_path)-1):
			ideal_path_id = (shortest_path[i],shortest_path[i+1])
			a = path_id.index(ideal_path_id)
			ideal_path.append(target[a])
		return ideal_path

#function that executes the path with respect to the ideal path
def path_executor(sim,goal):
	global stat,dir,exec,waypoint
	probot = sim.getObject('/Probot')
	l_motor = sim.getObject('/Probot/r_joint')
	r_motor = sim.getObject('/Probot/l_joint')

	relativeToObjectHandle = sim.handle_world
	position = sim.getObjectPosition(probot, relativeToObjectHandle)
	orientation = sim.getObjectOrientation(probot, relativeToObjectHandle)

	#getting yaw of the bot
	bot_or = orientation[2]
	if bot_or<0:
		bot_or = 6.28+bot_or
	
	#if theta goal has been reached switch status to move straight
	if goal[2] - 0.05 < bot_or < goal[2] + 0.05:
		stat = 0

	#if y goal has been reached switch status to turn or stop
	if goal[1]-0.1 < position[1] < goal[1]+0.1:
		if exec == 0:
			stat = 1
			exec = 2

		if exec == 1:
			stat = 2
			waypoint += 1
	
	#if x goal has been reached switch status to turn or stop
	if goal[0]-0.1 < position[0] < goal[0]+0.1:
		if exec == 0:
			stat = 1
			exec = 1

		if exec == 2:
			stat = 2
			waypoint += 1

	#DIFFERENT STATUS OF THE BOT TO MOVE
	if stat == 0:
		sim.setJointTargetVelocity(l_motor,1.5)
		sim.setJointTargetVelocity(r_motor,-1.5)
		print("moving straight")

	if stat == 1:
		print("turning")
		sim.setJointTargetVelocity(l_motor,1.5)
		sim.setJointTargetVelocity(r_motor,1.5)

	if stat == 2:
		print("stopped")
		sim.setJointTargetVelocity(l_motor,0)
		sim.setJointTargetVelocity(r_motor,0)
		stat = 0
		exec = 0

#COPPELIA SIM STUFF
#main
if __name__ == "__main__":
	client = RemoteAPIClient()
	sim = client.getObject('sim')

	try:

		## Start the simulation using ZeroMQ RemoteAPI
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

		## Runs the robot navigation logic written by participants
		try:
			while True:
				# the function get ideal path is called in this loop and goal coordinates are obtained

				if waypoint==0:
					path_executor(sim,get_ideal_path()[0])
				if waypoint==1:
					path_executor(sim,get_ideal_path()[1])


		except Exception:
			print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually if required.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

		
		## Stop the simulation using ZeroMQ RemoteAPI
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
		## Stop the simulation using ZeroMQ RemoteAPI
		return_code = sim.stopSimulation()
		time.sleep(0.5)
		if sim.getSimulationState() == sim.simulation_stopped:
			print('\nSimulation interrupted by user in CoppeliaSim.')
		else:
			print('\nSimulation could not be interrupted. Stop the simulation manually .')
			sys.exit()
