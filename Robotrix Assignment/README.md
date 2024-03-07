# Ka-Boom-Robotrix-2024
ROBOTRIX ASSIGNMENT SOLUTION

This branch contains a folder called as python scripts with two python scripts.
The first one is the custom made path_planner.py library and the other is the path_executer.py which contains the simulation code in coppeliasim.

PATH_PLANNER(path_planner.py)
This is a library written for planning the shortest path from a home node to a target node. 
As an input this library requires the home and the goal node, the node coordinates and the description of edges connecting the nodes.
It is represented by self.vertices and self.edges in the code.
Initially the list of the nodes that are explored and not explored are described.

The function get_fcost calculates the cost of each node with respect to the home and target nodes. The fcost is the combination of the distance of the node from the home and target nodes. 

The function get_next_node starts analyzing the nodes from the start node. It compares the fcost of the neighbour nodes and takes a decision of which edge to traverse in order to travel the minimum distance. This function returns a node that needs to be traversed next.

The function get_shortest_path returns a list of the nodes that need to be traversed sequentially to travel through the shortest path. this list is made by appending the output of the function get_next_node.

PATH EXECUTOR(path_executor.py)
We import the path_planner.py library into this script along with the things required for coppeliasim. This script takes input as the (x,y,theta) of the checkpoints in the provided World. 

In the function get_ideal_path the path planner class kp is called. It is followed by calculation of the fcost with respect to the home and goal node provided.With respect to the shortest path we get an ideal path is computed. This list contains the (x,y,theta) coordinates of the repective nodes in a sequential order.

In the path_executor function first the objects and position feedback are initialized. The algorithm to navigate the bot through the ideal path is a FSM containing states as move straight, turn and stop.
Three important if statements are made to switch between states. According to these state variables the power is given to the left and right motors of the differential drive bot to simulate it.
The variable waypoint makes the shift from one pick-up point to the other.

There is a infinite loop that runs in the try condition where the get_ideal_path function is called to get the waypoints of the nodes to be traversed.

{WHILE TESTING THE CODE}
Keep in mind that the execution part occurs in 2 ways. To make the robot go to the farthest pick-up point set the go_to_destination = 0 else if you want to return set go_to_destination = 1 and run the code making sure the orientation of the bot at start of the farthest node is 0 degree.




