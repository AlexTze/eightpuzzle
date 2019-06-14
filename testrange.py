from copy import deepcopy
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
    path = []
    current = current_node
    print("Path:")
    while current is not None:
        path.append(current.position)
        current = current.parent
    path.reverse()
    for item in path:
        print(item)


def findIndexOfZero(curr: Node):
    for i in range(3):
        for j in range(3):
            if curr.position[i][j] == 0:
                return i, j


def isInList(node, alist: list):
    for temp in alist:
        if temp.position == node.position:
            return True
    return False


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
        if temp.g > 10:
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

def BFS(start: list, end: list):
    """
    Breadth-first search algorithm for eight puzzle problem.
    """
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


start = [
    [1, 2, 3],
    [4, 0, 5],
    [7, 8, 6]
]
end = [
    [1, 2, 3],
    [4, 5, 0],
    [7, 8, 6]
]

DFS(start, end)
BFS(start,end)