# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

class SimpleGraph:
    def __init__(self):
        self.edges = {}
    
    def neighbors(self, id):
        return self.edges[id]

example_graph = SimpleGraph()
example_graph.edges = {
    'A': ['B'],
    'B': ['A', 'C', 'D'],
    'C': ['A'],
    'D': ['E', 'A'],
    'E': ['B']
}

import collections

class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, x):
        self.elements.append(x)
    
    def get(self):
        return self.elements.popleft()

# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)

def draw_tile(graph, id, style, width):
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = ">"
        if x2 == x1 - 1: r = "<"
        if y2 == y1 + 1: r = "v"
        if y2 == y1 - 1: r = "^"
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r

def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            print width,draw_tile(graph, (x, y), style, width),
        print 

# data from main article
DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]

class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

class GridWithWeights(SquareGrid):
    def __init__(self, width, height):
        #super().__init__(width, height)
        SquareGrid.__init__(self,width, height)
        self.weights = {}
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)

diagram4 = GridWithWeights(10, 10)
diagram4.walls = [(0, 7), (0, 8),(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8), (3, 5), (3, 6),(3, 3), (3, 4)]
'''
diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6), 
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6), 
                                       (5, 7), (5, 8), (6, 2), (6, 3), 
                                       (6, 4), (6, 5), (6, 6), (6, 7), 
                                       (7, 3), (7, 4), (7, 5)]}
'''

import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

# thanks to @m1sp <Jaiden Mispy> for this simpler version of
# reconstruct_path that doesn't have duplicate entries

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path
    
def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)         
            #print(next,cost_so_far[next])
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

def theta_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {} #parent
    cost_so_far = {} # g
    came_from[start] = start
    cost_so_far[start] = 0
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far : 
                if next not in cost_so_far : 
                    cost_so_far[next] = 99999999999999
                if(LineOfSight(came_from[current],next)):
                    if ((cost_so_far[came_from[current]] + graph.cost(came_from[current],next)) < cost_so_far[next]) :
                        came_from[next] = came_from[current]
                        cost_so_far[next] = cost_so_far[came_from[current]]+graph.cost(came_from[current],next)
                        priority = (cost_so_far[came_from[current]] + graph.cost(came_from[current],next)) + heuristic(goal, next)
                        frontier.put(next, priority)
                else:
                    if (new_cost < cost_so_far[next]):
                        cost_so_far[next] = new_cost
                        priority = new_cost + heuristic(goal, next)
                        frontier.put(next, priority)
                        came_from[next] = current 
    return came_from, cost_so_far
    
def theta_star_search2(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {} #parent
    cost_so_far = {} # g
    came_from[start] = start #start
    cost_so_far[start] = 0
    path = [start]
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or (new_cost < cost_so_far[next]): 
                if(LineOfSight(came_from[current],next)):
                    if(next not in cost_so_far):
                        cost_so_far[next] = 999999
                    if ((cost_so_far[came_from[current]] + graph.cost(came_from[current],next)) < cost_so_far[next]) :
                        if(came_from[current] not in came_from.values()):
                            path.append(came_from[current])
                        came_from[next] = came_from[current]
                        cost_so_far[next] = cost_so_far[came_from[current]]+graph.cost(came_from[current],next)
                        priority = (cost_so_far[came_from[current]] + graph.cost(came_from[current],next)) + heuristic(goal, next)
                        frontier.put(next, priority)
                else:
                    if(current not in came_from.values()):
                        path.append(current)
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
    path.append(goal)                
    return came_from, cost_so_far, path
    
def LineOfSight(s1,s2):
    x0=s1[0]
    y0=s1[1]
    x1=s2[0]
    y1=s2[1]
    dy=y1-y0
    dx=x1-x0
    f=0
    if(dy < 0):
        dy=-dy
        sy=-1
    else:
        sy=1
    if(dx < 0):
        dx=-dx
        sx=-1
    else:
        sx=1
    if(dx >= dy):
        while (x0 is not x1):
            f=f+dy
            if(f >= dx):
                if ((x0+((sx-1)/2),y0+((sy-1)/2)) in diagram4.walls):
                    return False
                y0 = y0+sy
                f = f-dx
            if (f is not 0) and ((x0+((sx-1)/2),y0+((sy-1)/2)) in diagram4.walls):
                return False
            if (dy is 0) and ((x0+((sx-1)/2),y0) in diagram4.walls) and ((x0+((sx-1)/2),y0-1) in diagram4.walls):
                return False
            x0=x0+sx
    else:
        while (y0 is not y1):
            f=f+dx
            if(f >= dy):
                if ((x0+((sx-1)/2),y0+((sy-1)/2)) in diagram4.walls):
                    return False
                x0 = x0+sx
                f = f-dy
            if (f is not 0) and ((x0+((sx-1)/2),y0+((sy-1)/2)) in diagram4.walls):
                return False
            if (dx is 0) and ((x0,y0+((sy-1)/2)) in diagram4.walls) and ((x0-1,y0+((sy-1)/2)) in diagram4.walls):
                return False
            y0=y0+sy
    return True
