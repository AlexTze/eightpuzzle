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

def isInList(node, alist: list):
    for temp in alist:
        if temp.position == node.position:
            return True
    return False
    
directions = [[1, 0], [-1, 0], [0, 1], [0, -1]]


def DFS(start: list, end: list):
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = []
    closed_list = []
    open_list.append(start_node)

    while len(open_list) > 0:
        current_node = open_list.pop()

        if current_node.position == end_node.position:
            print(constructPath(current_node))

        currRow, currColumn = findIndexOfZero(current_node)

        for i in range(4):
            nextNode = deepcopy(current_node)
            nextRow = currRow + directions[i][0]
            nextColumn = currColumn + directions[i][1]
            if nextRow > 2 or nextRow < 0 or nextColumn > 2 or nextColumn < 0:
                continue
            nextNode.position[currRow][currColumn], nextNode.position[nextRow][nextColumn] = \
                nextNode.position[nextRow][nextColumn], nextNode.position[currRow][currColumn]

            if (not isInList(nextNode, open_list)) and (not isInList(nextNode, closed_list)):
                open_list.append(nextNode)

        if not isInList(current_node, closed_list):
            closed_list.append(current_node)

    print(constructPath(current_node))

start = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 0]
]
end = [
    [1, 2, 3],
    [4, 5, 0],
    [7, 8, 6]
]
DFS(start,end)