'''
Licensing Information: Please do not distribute or publish solutions to this
project. You are free to use and extend Driverless Car for educational
purposes. The Driverless Car project was developed at Stanford, primarily by
Chris Piech (piech@cs.stanford.edu). It was inspired by the Pacman projects.
'''
import util
import itertools
from turtle import Vec2D
from engine.const import Const
from engine.vector import Vec2d
from engine.model.car.car import Car
from engine.model.layout import Layout
from engine.model.car.junior import Junior
from configparser import InterpolationMissingOptionError

# Class: Graph
# -------------
# Utility class
class Graph(object):
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.edges = edges

# Class: IntelligentDriver
# ---------------------
# An intelligent driver that avoids collisions while visiting the given goal locations (or checkpoints) sequentially. 
class IntelligentDriver(Junior):

    # Funciton: Init
    def __init__(self, layout: Layout):
        self.burnInIterations = 30
        self.layout = layout 
        # self.worldGraph = None
        self.worldGraph = self.createWorldGraph()
        self.checkPoints = self.layout.getCheckPoints() # a list of single tile locations corresponding to each checkpoint
        self.transProb = util.loadTransProb()
        # print(self.checkPoints)   #list of tuples
        # _ = input()
    # ONE POSSIBLE WAY OF REPRESENTING THE GRID WORLD. FEEL FREE TO CREATE YOUR OWN REPRESENTATION.
    # Function: Create World Graph
    # ---------------------
    # Using self.layout of IntelligentDriver, create a graph representing the given layout.
    def createWorldGraph(self):
        nodes = []
        edges = []
        # create self.worldGraph using self.layout
        numRows, numCols = self.layout.getBeliefRows(), self.layout.getBeliefCols()

        # NODES #
        ## each tile represents a node
        nodes = [(x, y) for x, y in itertools.product(range(numRows), range(numCols))]
        
        # EDGES #
        ## We create an edge between adjacent nodes (nodes at a distance of 1 tile)
        ## avoid the tiles representing walls or blocks#
        ## YOU MAY WANT DIFFERENT NODE CONNECTIONS FOR YOUR OWN IMPLEMENTATION,
        ## FEEL FREE TO MODIFY THE EDGES ACCORDINGLY.

        ## Get the tiles corresponding to the blocks (or obstacles):
        blocks = self.layout.getBlockData()
        # print(blocks)
        # _ = input()
        blockTiles = []
        for block in blocks:
            # print("block:", block)
            # _ = input()
            row1, col1, row2, col2 = block[1], block[0], block[3], block[2] 
            # some padding to ensure the AutoCar doesn't crash into the blocks due to its size. (optional)
            row1, col1, row2, col2 = row1-1, col1-1, row2+1, col2+1
            blockWidth = col2-col1 
            blockHeight = row2-row1 

            for i in range(blockHeight):
                for j in range(blockWidth):
                    blockTile = (row1+i, col1+j)
                    blockTiles.append(blockTile)

        ## Remove blockTiles from 'nodes'
        nodes = [x for x in nodes if x not in blockTiles]

        for node in nodes:
            x, y = node[0], node[1]
            adjNodes = [(x, y-1), (x, y+1), (x-1, y), (x+1, y), (x+1, y-1), (x-1, y-1), (x+1, y+1), (x-1, y+1)]
            
            # only keep allowed (within boundary) adjacent nodes
            adjacentNodes = []
            for tile in adjNodes:
                if tile[0]>=0 and tile[1]>=0 and tile[0]<numRows and tile[1]<numCols:
                    if tile not in blockTiles:
                        adjacentNodes.append(tile)

            for tile in adjacentNodes:
                edges.append((node, tile))
                # edges.append((tile, node))
        return Graph(nodes, edges)

    #######################################################################################
    # Function: Get Next Goal Position
    # ---------------------
    # Given the current belief about where other cars are and a graph of how
    # one can driver around the world, chose the next position.
    #######################################################################################
    def getNextGoalPos(self, beliefOfOtherCars: list, parkedCars:list , chkPtsSoFar: int):
        '''
        Input:
        - beliefOfOtherCars: list of beliefs corresponding to all cars
        - parkedCars: list of booleans representing which cars are parked
        - chkPtsSoFar: the number of checkpoints that have been visited so far 
                       Note that chkPtsSoFar will only be updated when the checkpoints are updated in sequential order!

        Output:
        - goalPos: The position of the next tile on the path to the next goal location.
        - moveForward: Unset this to make the AutoCar stop and wait.

        Notes:
        - You can explore some files "layout.py", "model.py", "controller.py", etc.
         to find some methods that might help in your implementation. 
        '''

        currPos = self.getPos() # the current 2D location of the AutoCar (refer util.py to convert it to tile (or grid cell) coordinate)
        goalPos = (currPos[0],currPos[1]) # next tile 
        moveForward = True
        # BEGIN_YOUR_CODE 
        import math
        currTile = (util.yToRow(currPos[1]),util.xToCol(currPos[0]))
        checkpointTile = self.checkPoints[chkPtsSoFar]
        
        n_cars = len(beliefOfOtherCars)
        curr_pos = []
        for i in range(n_cars):
            temp_grid = beliefOfOtherCars[i].grid
            prob_dict = {}
            for j in range(len(temp_grid)):
                for k in range(len(temp_grid[j])):
                    prob_dict[(j,k)] = temp_grid[j][k]
            curr_pos.append(prob_dict)
            
        rows = len(beliefOfOtherCars[0].grid)
        cols = len(beliefOfOtherCars[0].grid[0])
        trans_mat = {}
        for i in range(rows):
            for j in range(cols):
                for k in range(rows):
                    for l in range(cols):
                        if (((i,j),(k,l))) in self.transProb:
                            trans_mat[((i,j),(k,l))] = self.transProb[((i,j),(k,l))]
                            
                        else:
                            trans_mat[((i,j),(k,l))] = 0

        next_pos = []
        for prev_grid in curr_pos:
            trans_grid = {}
            for i in range(rows):
                for j in range(cols):
                    next_prob = 0
                    for k in range(rows):
                        for l in range(cols):
                            next_prob += prev_grid[(k,l)]*trans_mat[((k,l),(i,j))]
                    trans_grid[(i,j)] = next_prob
            next_pos.append(trans_grid)
            

        prob_find = curr_pos[0].copy()
        
        for i in range(1,len(curr_pos)):
            for j in range(rows):
                for k in range(cols):
                    if curr_pos[i][(j,k)] > prob_find[(j,k)]:
                        prob_find[(j,k)] = curr_pos[i][(j,k)]

        for i in range(0,len(next_pos)):
            for j in range(rows):
                for k in range(cols):
                    if next_pos[i][(j,k)] > prob_find[(j,k)]:
                        prob_find[(j,k)] = next_pos[i][(j,k)]
        
        sorted_prob_find = dict(reversed(list(sorted(prob_find.items(), key=lambda item: item[1]))))
        prob_find=dict(sorted_prob_find)
        
        oppo_car_tiles = []
        for tile in prob_find:
            if prob_find[tile] > 0.01:
                oppo_car_tiles.append(tile)
        


        if currTile not in self.worldGraph.nodes:
            potential_gp = []
            temp_goalpos = [(currTile[0], currTile[1]-1), (currTile[0], currTile[1]+1), (currTile[0]-1, currTile[1]), 
                                    (currTile[0]+1, currTile[1]), (currTile[0]+1, currTile[1]-1), (currTile[0]-1, currTile[1]-1), 
                                    (currTile[0]+1, currTile[1]+1), (currTile[0]-1, currTile[1]+1)]
            for gp in temp_goalpos:
                for move in self.worldGraph.edges:
                    if move[0] == gp:
                        potential_gp.append(gp)
                        break
            for pos in potential_gp:
                if not pos in oppo_car_tiles:
                    goalPos = (util.colToX(pos[1]), util.rowToY(pos[0]))
                    return goalPos, moveForward

        if currTile in oppo_car_tiles:
            potential_gp = []
            temp_goalpos = [(currTile[0], currTile[1]-1), (currTile[0], currTile[1]+1), (currTile[0]-1, currTile[1]), 
                                    (currTile[0]+1, currTile[1]), (currTile[0]+1, currTile[1]-1), (currTile[0]-1, currTile[1]-1), 
                                    (currTile[0]+1, currTile[1]+1), (currTile[0]-1, currTile[1]+1)]
            for gp in temp_goalpos:
                for move in self.worldGraph.edges:
                    if move[0] == gp:
                        potential_gp.append(gp)
                        break
            for pos in potential_gp:
                if not pos in oppo_car_tiles:
                    goalPos = (util.colToX(pos[1]), util.rowToY(pos[0]))
                    return goalPos, moveForward




        # Python3 program for Bidirectional BFS
        # Search to check path between two vertices

        # Class definition for node to
        # be added to graph
        class AdjacentNode:
            
            def __init__(self, vertex):
                
                self.vertex = vertex
                self.next = None

        # BidirectionalSearch implementation
        class BidirectionalSearch:
            
            def __init__(self, allNodes):
                
                # Initializing queue for forward
                # and backward search
                self.source_queue = []
                self.dest_queue = []
                
                # Initialize vertices and
                # graph with vertices
                self.allNodes = allNodes
                self.graph = {}
                self.src_visited = {}
                self.dest_visited = {}
                self.src_parent = {}
                self.dest_parent = {}

                for nod in self.allNodes:
                    self.graph[nod] = None
                
                    # Initializing source and
                    # destination visited nodes as False
                    self.src_visited[nod] = False
                    self.dest_visited[nod] = False
                    
                    # Initializing source and destination
                    # parent nodes
                    self.src_parent[nod] = None
                    self.dest_parent[nod] = None
                
            # Function for adding undirected edge
            def create_edge(self, neigbours):
                source_node, dest_node =  neigbours[0], neigbours[1]
                # Add edges to graph

                # Add source to destination
                node = AdjacentNode(dest_node)
                node.next = self.graph[source_node]
                self.graph[source_node] = node

                # Since graph is undirected add
                # destination to source
                node = AdjacentNode(source_node)
                node.next = self.graph[dest_node]
                self.graph[dest_node] = node
                
            # Function for Breadth First Search
            def bfs(self, direction):
                
                if direction == 'forward':
                    
                    # BFS in forward direction
                    current = self.source_queue.pop(0)
                    connected_node = self.graph[current]
                    
                    while connected_node:
                        vertex = connected_node.vertex
                        
                        if not self.src_visited[vertex]:
                            self.source_queue.append(vertex)
                            self.src_visited[vertex] = True
                            self.src_parent[vertex] = current
                            
                        connected_node = connected_node.next
                else:
                    
                    # BFS in backward direction
                    current = self.dest_queue.pop(0)
                    connected_node = self.graph[current]
                    
                    while connected_node:
                        vertex = connected_node.vertex
                        
                        if not self.dest_visited[vertex]:
                            self.dest_queue.append(vertex)
                            self.dest_visited[vertex] = True
                            self.dest_parent[vertex] = current
                            
                        connected_node = connected_node.next
                        
            # Check for intersecting vertex
            def get_intersecting_node(self):
                
                # Returns intersecting node
                # if present else -1
                for i in range(len(self.allNodes)):
                    if (self.src_visited[self.allNodes[i]] and
                        self.dest_visited[self.allNodes[i]]):
                        return self.allNodes[i]
                        
                return -1

            # Print the path from source to target
            def get_path(self, intersecting_node,
                        source_node, dest_node):
                
                path = list()
                path.append(intersecting_node)
                i = intersecting_node
                
                while i != source_node:
                    path.append(self.src_parent[i])
                    i = self.src_parent[i]
                    
                path = path[::-1]
                i = intersecting_node
                
                while i != dest_node:
                    path.append(self.dest_parent[i])
                    i = self.dest_parent[i]
                    
                return path
            
            # Function for bidirectional searching
            def bidirectional_search(self, source_node, dest_node):
                
                self.source_queue.append(source_node)
                self.src_visited[source_node] = True
                self.src_parent[source_node] = -1
                
                self.dest_queue.append(dest_node)
                self.dest_visited[dest_node] = True
                self.dest_parent[dest_node] = -1

                while self.source_queue and self.dest_queue:
                    
                    self.bfs('forward')
                    
                    self.bfs('backward')
                    
                    intersecting_node = self.get_intersecting_node()
                    
                    if intersecting_node != -1:
                        path = self.get_path(intersecting_node, source_node, dest_node)
                        return path
                return [currTile, currTile]
        
        # Source Vertex
        source_node = currTile
        
        # Destination Vertex
        dest_node = checkpointTile
        
        # Create a graph
        graph = BidirectionalSearch(self.worldGraph.nodes)
        
        for neigbours in self.worldGraph.edges:
            graph.create_edge(neigbours)

        
        out = graph.bidirectional_search(source_node, dest_node)
        goal_tile = out[1]

        if goal_tile not in oppo_car_tiles:
            goalPos = (util.colToX(goal_tile[1]), util.rowToY(goal_tile[0]))
            return goalPos, moveForward

        else:
            potential_gp = []
            
            if goal_tile[0]>currTile[0] or goal_tile[0]<currTile[0]:
                if goal_tile[1]<currTile[1]:
                    temp_goalpos = [(goal_tile[0], goal_tile[1]+2), (goal_tile[0], goal_tile[1]+1)]
                elif goal_tile[1]==currTile[1]:
                    temp_goalpos = [(goal_tile[0], goal_tile[1]-1), (goal_tile[0], goal_tile[1]+1)]
                elif goal_tile[1]>currTile[1]:
                    temp_goalpos = [(goal_tile[0], goal_tile[1]-2), (goal_tile[0], goal_tile[1]-1)]  

            elif goal_tile[1]>currTile[1] or goal_tile[1]<currTile[1]:
                if goal_tile[0]<currTile[0]:
                    temp_goalpos = [(goal_tile[0]+2, goal_tile[1]), (goal_tile[0]+1, goal_tile[1])]
                elif goal_tile[0]==currTile[0]:
                    temp_goalpos = [(goal_tile[0]-1, goal_tile[1]), (goal_tile[0]+1, goal_tile[1])]
                elif goal_tile[0]>currTile[0]:
                    temp_goalpos = [(goal_tile[0]-2, goal_tile[1]), (goal_tile[0]-1, goal_tile[1])]                   

            for gp in temp_goalpos:
                for move in self.worldGraph.edges:
                    if move[0] == gp:
                        potential_gp.append(gp)
                        break
            for pos in potential_gp:
                if not pos in oppo_car_tiles:
                    goalPos = (util.colToX(pos[1]), util.rowToY(pos[0]))
                    return goalPos, moveForward
        
        moveForward = False
        return currPos, moveForward




    # DO NOT MODIFY THIS METHOD !
    # Function: Get Autonomous Actions
    # --------------------------------
    def getAutonomousActions(self, beliefOfOtherCars: list, parkedCars: list, chkPtsSoFar: int):
        # Don't start until after your burn in iterations have expired
        if self.burnInIterations > 0:
            self.burnInIterations -= 1
            return[]
       
        goalPos, df = self.getNextGoalPos(beliefOfOtherCars, parkedCars, chkPtsSoFar)
        vectorToGoal = goalPos - self.pos
        wheelAngle = -vectorToGoal.get_angle_between(self.dir)
        driveForward = df
        actions = {
            Car.TURN_WHEEL: wheelAngle
        }
        if driveForward:
            actions[Car.DRIVE_FORWARD] = 1.0
        return actions
    
    