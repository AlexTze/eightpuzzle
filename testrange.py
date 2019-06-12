from collections import deque
from copy import deepcopy


class Node:
    """
    Node class for pathfinding
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        # for use in A* search
        # g: traverse cost
        # h: heuristic search cost
        # f = g + h
        self.g = 0
        self.h = 0
        self.f = self.g + self.h

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f


def constructPath(current_node: Node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1]


def findIndexOfZero(curr: Node):
    for i in range(3):
        for j in range(3):
            if curr.position[i][j] == 0:
                return i, j


directions = [[1, 0], [-1, 0], [0, 1], [0, -1]]


def BFS(start: list, end: list):
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = deque()
    closed_list = []
    open_list.append(start_node)

    while len(open_list > 0):
        current_node = open_list.deque()

        if current_node.position == end_node.position:
            print(constructPath(current_node))

        currRow, currColumn = findIndexOfZero(current_node)

        for i in range(4):
            nextNode = deepcopy(current_node)
            nextRow = currRow + directions[i][0]
            nextColumn = currColumn + directions[i][1]
