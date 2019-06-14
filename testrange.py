from copy import deepcopy
from time import time


class Node:
    """
    Node class for pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        # g: traverse cost (depth when used in DFS search)
        # h: heuristic search cost
        # f = g + h
        self.g = 0
        self.h = 0
        self.f = 0


def constructPath(current_node: Node):
    """
    Prints out found solution path
    """
    path = []
    current = current_node
    print("Found a path:")
    while current is not None:
        path.append(current.position)
        current = current.parent
    path.reverse()
    for item in path:
        for row in item:
            print(row)
        print()


def findIndexOfZero(curr: Node):
    for i in range(3):
        for j in range(3):
            if curr.position[i][j] == 0:
                return i, j


def isInList(node, alist: list):
    """
    Determine if a node is within a list
    """
    for temp in alist:
        if temp.position == node.position:
            return True
    return False

def hammingDistance(this: list, that: list):
    """
    Calculates the hamming distance between two 3x3 lists
    """
    count = 0
    for i in range(3):
        for j in range(3):
            if this[i][j] != that[i][j]:
                count += 1
    return count

def manhattanDistance(current: list, target: list):
    """
    Calculates the Manhattan distance between two 3x3 lists
    """
    count = 0
    curr_1d = []
    tgt_1d = []
    for i in range(3):
        for j in range(3):
            curr_1d.append(current[i][j])
            tgt_1d.append(target[i][j])
    for x in tgt_1d:
        count += abs(tgt_1d.index(x) - curr_1d.index(x))
    return count

def unknownEvaluateFunction(current: list, target: list):
    pass

directions = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1]
]


def DFS(start: list, end: list):
    """
    Depth-first search algorithm for eight puzzle problem.
    """
    print("Running depth-first search...")

    # initialize start and end node
    startNode = Node(None, start)
    endNode = Node(None, end)

    # counter for expanded nodes
    expandedNodes = 0

    # initialize open and closed list
    openList = []
    closedList = []

    # append start node to open list
    openList.append(startNode)
    expandedNodes += 1

    while len(openList) > 0:
        # We use the open list as a stack in DFS,
        # pop() will remove the rightmost element.
        temp = openList.pop()

        # found a path
        if temp.position == endNode.position:
            print("DFS expanded nodes: {}".format(expandedNodes))
            return constructPath(temp)
        
        # limit search depth
        if temp.g > 15:
            continue

        if not isInList(temp, closedList):
            closedList.append(temp)

        # row and column of element '0'
        currRow, currCol = findIndexOfZero(temp)

        # iterate through possible moves
        for i in range(4):
            nextNode = deepcopy(temp)
            nextNode.parent = temp
            nextRow = currRow + directions[i][0]
            nextCol = currCol + directions[i][1]

            # discard out-of-bound moves 
            if nextRow > 2 or nextRow < 0 or nextCol > 2 or nextCol < 0:
                continue

            # add all eligible moves to open list
            nextNode.position[currRow][currCol], nextNode.position[nextRow][nextCol] = nextNode.position[nextRow][nextCol], nextNode.position[currRow][currCol]
            if not isInList(nextNode, openList):
                nextNode.g += 1
                openList.append(nextNode)
                expandedNodes += 1

    # failed to find a path within depth constraint
    print("The algorithm attempted but failed to find a path with {} expanded nodes.".format(expandedNodes))

def BFS(start: list, end: list):
    """
    Breadth-first search algorithm for eight puzzle problem.
    """
    print("Running breadth-first search...")

    # initialize start and end node
    startNode = Node(None, start)
    endNode = Node(None, end)

    # initialize open and closed list
    openList = []
    closedList = []

    # counter for expanded nodes
    expandedNodes = 0

    # append start node to open list
    openList.append(startNode)
    expandedNodes += 1

    while len(openList) > 0:
        # We use the open list as a queue in BFS,
        # pop(0) will remove the leftmost element.
        temp = openList.pop(0)

        # found a path
        if temp.position == endNode.position:
            print("BFS expanded nodes: {}".format(expandedNodes))
            return constructPath(temp)

        if not isInList(temp, closedList):
            closedList.append(temp)

        # row and column of element '0'
        currRow, currCol = findIndexOfZero(temp)

        # iterate through possible moves
        for i in range(4):
            nextNode = deepcopy(temp)
            nextNode.parent = temp
            nextRow = currRow + directions[i][0]
            nextCol = currCol + directions[i][1]

            # discard out-of-bound moves 
            if nextRow > 2 or nextRow < 0 or nextCol > 2 or nextCol < 0:
                continue
            
            # add all eligible moves to open list
            nextNode.position[currRow][currCol], nextNode.position[nextRow][nextCol] = nextNode.position[nextRow][nextCol], nextNode.position[currRow][currCol]
            if not isInList(nextNode, openList):
                openList.append(nextNode)
                expandedNodes += 1

def astar1(start: list, end: list):
    """A* search algorithm, with h as hamming distance"""
    print("Running A* search variant 1...")

    # initialize start and end node
    startNode = Node(None, start)
    endNode = Node(None, end)

    # initialize open and closed list
    openList = []
    closedList = []

    # counter for expanded nodes
    expandedNodes = 0

    # append start node to open list
    openList.append(startNode)
    expandedNodes += 1

    while len(openList) > 0:
        openList.sort(key=lambda x: x.f)
        temp = openList.pop(0)

        # found a path
        if temp.position == endNode.position:
            print("A*1 expanded nodes: {}".format(expandedNodes))
            return constructPath(temp)

        if not isInList(temp, closedList):
            closedList.append(temp)

        # row and column of element '0'
        currRow, currCol = findIndexOfZero(temp)

        # iterate through possible moves
        for i in range(4):
            nextNode = deepcopy(temp)
            nextNode.parent = temp
            nextRow = currRow + directions[i][0]
            nextCol = currCol + directions[i][1]

            # discard out-of-bound moves 
            if nextRow > 2 or nextRow < 0 or nextCol > 2 or nextCol < 0:
                continue
            
            nextNode.position[currRow][currCol], nextNode.position[nextRow][nextCol] = nextNode.position[nextRow][nextCol], nextNode.position[currRow][currCol]
            nextNode.g += 1
            nextNode.h = hammingDistance(nextNode.position, end)
            nextNode.f = nextNode.g + nextNode.h

            # add all eligible moves to open list
            if not isInList(nextNode, openList):
                openList.append(nextNode)
                expandedNodes += 1

def astar2(start: list, end: list):
    """A* search algorithm, with h as sum of manhattan distance of dices"""
    print("Running A* search variant 2...")

    # initialize start and end node
    startNode = Node(None, start)
    endNode = Node(None, end)

    # initialize open and closed list
    openList = []
    closedList = []

    # counter for expanded nodes
    expandedNodes = 0

    # append start node to open list
    openList.append(startNode)
    expandedNodes += 1

    while len(openList) > 0:
        openList.sort(key=lambda x: x.f)
        temp = openList.pop(0)

        # found a path
        if temp.position == endNode.position:
            print("A*1 expanded nodes: {}".format(expandedNodes))
            return constructPath(temp)

        if not isInList(temp, closedList):
            closedList.append(temp)

        # row and column of element '0'
        currRow, currCol = findIndexOfZero(temp)

        # iterate through possible moves
        for i in range(4):
            nextNode = deepcopy(temp)
            nextNode.parent = temp
            nextRow = currRow + directions[i][0]
            nextCol = currCol + directions[i][1]

            # discard out-of-bound moves 
            if nextRow > 2 or nextRow < 0 or nextCol > 2 or nextCol < 0:
                continue
            
            nextNode.position[currRow][currCol], nextNode.position[nextRow][nextCol] = nextNode.position[nextRow][nextCol], nextNode.position[currRow][currCol]
            nextNode.g += 1
            nextNode.h = manhattanDistance(nextNode.position, end)
            nextNode.f = nextNode.g + nextNode.h

            # add all eligible moves to open list
            if not isInList(nextNode, openList):
                openList.append(nextNode)
                expandedNodes += 1
                
start = [
    [1, 2, 3],
    [4, 0, 5],
    [7, 8, 6]
]
end = [
    [1, 2, 3],
    [4, 0, 5],
    [6, 7, 8]
]

# dfs_start = time()
# DFS(start, end)
# dfs_end = time()
# print("Elapsed time: {}".format(dfs_end - dfs_start))
# bfs_start = time()
# BFS(start, end)
# bfs_end = time()
# print("Elapsed time: {}".format(bfs_end - bfs_start))
# astar1(start,end)