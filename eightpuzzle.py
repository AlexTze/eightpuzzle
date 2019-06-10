# Eight puzzle problem solving with BFS, DFS, and A* algorithms
# Alex Tze, May 2019


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


def swapListItem(targetList: list, srcRow: int, srcColumn: int, destRow: int, destColumn: int):
    targetList[srcRow][srcColumn], targetList[destRow][destColumn] = \
        targetList[destRow][destColumn], targetList[srcRow][srcColumn]


def constructPath(current_node: Node):
    pass


def astar1(start: list, end: list):
    """
    A* algorithm, with h as hamming distance of digits
    """

    # calculates hamming distance
    def hammingDistance(this: Node, that: Node):
        k = 0
        for i, j in zip(this.position, that.position):
            for m, n in zip(i, j):
                if m != n:
                    k += 1
        return k

    # Initialize start and end node
    start_node = Node(None, start)
    end_node = Node(None, end)

    # Initialize open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    while len(open_list) > 0:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            return constructPath(current_node)


def bfs(start, end):
    pass


def dfs(start, end):
    pass


start = [
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 0]
]
end = [
    [1, 2, 3],
    [8, 0, 4],
    [7, 6, 5]
]

# astar1(start, end)
