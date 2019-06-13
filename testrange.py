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
    startNode = Node(None, start)
    endNode = Node(None, end)
    openList = []
    closedList = []
    openList.append(startNode)
    while len(openList) > 0:
        temp = openList.pop()
        if temp.position == endNode.position:
            print(constructPath(temp))
        if not isInList(temp, closedList):
            closedList.append(temp)
            currRow, currCol = findIndexOfZero(temp)
            for i in range(4):
                nextNode = temp
                nextNode.parent = temp
                nextRow = currRow + directions[i][0]
                nextCol = currCol + directions[i][1]
                if nextRow > 2 or nextRow < 0 or nextCol > 2 or nextCol < 0:
                    continue
                nextNode.position[currRow][currCol],
                nextNode.position[nextRow][nextCol] = \
                    nextNode.position[nextRow][nextCol],
                nextNode.position[currRow][currCol]
                if not isInList(nextNode, openList):
                    openList.append(nextNode)


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
DFS(start, end)
