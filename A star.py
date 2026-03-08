#A* Informed Search Algorithm Implementation in Python

import heapq #Helps us to select a node with the lowest value

#Edges with costs
e = {
    'S': {'A': 2, 'B': 5},
    'A': {'C': 4, 'D': 2},
    'B': {'D': 1},
    'C': {'G': 3},
    'D': {'G': 6, 'E': 2},
    'E': {'G': 1},
    'G': {}
}

#Heuristic values h(n) - For Nodes S, A, B, C, D, E, G
#Estimated cost from each node to the goal node G
h = {
    'S': 7,
    'A': 5,
    'B': 6,
    'C': 2,
    'D': 3,
    'E': 1,
    'G': 0
}

def lowest_cost(start_node, goal_node):     #A* Search Algorithm starts 
    
    #Priority queue (min heap)
    list = []                               #Store nodes
    heapq.heappush(list, (0, start_node))   #Push the start node

    least_value_path = {}                          #Stores the best path to reach each node
    node_cost = {node: float('inf') for node in e}   #It creates a dictionary to store the cost from the start node to each node, initialized to infinity
    node_cost[start_node] = 0

    total_cost = {node: float('inf') for node in e}   #Initializes the total_cost dictionary to store the estimated total cost from the start node to the goal node through each node, initialized to infinity
    total_cost[start_node] = h[start_node]

    while list:                             #It runs until it reaches the Goal node
        present = heapq.heappop(list)[1]    #Select the node with the lowest f_cost value from the priority queue

        if present == goal_node:            #If current node is Goal nod, stop here
            path = []                       #To store final path from start to goal
            while present in least_value_path:     #Goes backward from the goal node to the start node using the least_value_path dictionary to reconstruct the path
                path.append(present)           #Adds the current node to the path list
                present = least_value_path[present]   #& Moving to the current node 
            path.append(start_node)            #Includes the start node in the path
            path.reverse()                     #Reverses the path to get the correct order from start to goal
            return path, node_cost[goal_node]     #Returns the path and the total cost to reach the goal node

        for near_by, cost in e[present].items():  #Checks neighbors
            new_cost = node_cost[present] + cost           #Calculates new cost to reach the near_by through the current node

            if new_cost < node_cost[near_by]:             #Checks if the new cost is less than the previously recorded cost for the neighbor
                least_value_path[near_by] = present              #Saves the current node as the best path to reach the neighbor
                node_cost[near_by] = new_cost                  #Update g(N)
                total_cost[near_by] = new_cost + h[near_by]    #calculates total_cost(N) = g(N) + h(N) for the neighbor
                heapq.heappush(list, (total_cost[near_by], near_by))  #Add node to the priority queue with its total_cost(N) value

    return None, float('inf')             #If the goal node is not reachable, return None and infinity as cost


#Runs lowest cost search
#It starts from node 'S' and aims to reach node 'G', while calculating the path and the least cost using the A* algorithm implemented in the lowest_cost function.
path, cost = lowest_cost('S', 'G')

#Prints the resulting path and the least cost to reach the goal node 'G' from the start node 'S'. The path is displayed in a readable format, showing the sequence of nodes from start to goal, and the total cost is also printed.
print("Path:", " -> ".join(path))
print("Least Cost:", cost)