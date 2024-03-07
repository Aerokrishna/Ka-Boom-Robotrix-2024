import math as m

#describe the start and end node
start_node = 0
target_node = 2

class kp():
    def __init__(self,start_node,target_node):
        self.start_node = start_node
        self.target_node = target_node

        #specify coordinates of the nodes
        self.vertices = [(-2.51,-2.33),(-0.50,0.57),(2.42,1.57),(-0.50,-2.37),(1.50,-1.37)] #x and y

        #describe the edges connecting those nodes
        self.edges = [(0,1),(0,3),(1,0),(1,3),(1,4),(1,2),
                      (2,1),(2,4),(3,0),(3,1),(3,4),(4,3),
                      (4,1),(4,2)] #start end
        
        #list of nodes that have been explored and not explored
        self.open_nodes = [0, 1, 2, 3, 4]
        self.closed_nodes = []
        
        self.f_cost = []
        self.curr = start_node

    #getting a value or cost of each node
    def get_fcost(self,home,goal): 
        for i in range(len(self.vertices)):
            x = self.vertices[i][0]
            y = self.vertices[i][1]

            g = m.sqrt((x - home[0])**2 + (y - home[1])**2)
            h = m.sqrt((x - goal[0])**2 + (y - goal[1])**2)
            f = 0.2 * g + 0.9 * h

            self.f_cost.append(f)

    #returns a list of the best node to be traversed next
    def get_next_node(self):
        neighbour_fcost = []
        neighbour_nodes= []
        open_edges = []

        #put current node in closed nodes list
        for i in range(len(self.open_nodes)):
            if self.curr == self.open_nodes[i]:
                self.closed_nodes.append(self.curr)
                self.open_nodes.remove(self.curr)
                break
        
        #remove the path already traversed

        for j in range(len(self.edges)):
            for k in range(len(self.open_nodes)):
                if self.edges[j][1] == self.open_nodes[k] and self.edges[j][0] == self.curr:
                    neighbour_nodes.append(self.edges[j][1])

                    #getting current node's neighbour's fcost and hcost
                    neighbour_fcost.append(self.f_cost[self.edges[j][1]])

        #getting minimum of the fcost and hcost
        min_fcost = neighbour_fcost.index(min(neighbour_fcost))
        
        #get the next node
        next_node = neighbour_nodes[min_fcost]
        self.curr = next_node

        return next_node
    
    #returns a list of nodes which is the sequence in which the nodes need to be traversed
    def get_shortest_path(self,home_node,goal_node):
        shortest_path = [home_node]
        while self.curr!=goal_node:
            shortest_path.append(self.get_next_node())
            
        return shortest_path

