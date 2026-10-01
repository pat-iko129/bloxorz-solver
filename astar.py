from math import sqrt

from utils import Direction, Position, Block, Node
from maze import Maze

DISTANCES = ["Euclidean", "Chebyshev"]

def planner(maze, dist_type) -> list:
    if dist_type not in DISTANCES:
        assert dist_type in DISTANCES, "Must be a valid type of distance metric (Euclidian, Chebyshev)"

    ####################  INITIALIZE  ####################
    # Track the nodes created (seen) as a dictionary indexed by state.
    nodes = {}

    # Set up the starting on-deck queue, with zero cost (parent = None)
    onDeck = []
    done = []

    start_pos = maze.start
    start_block = Block(start_pos, start_pos)         # always starts standing
    start_node = Node(start_block, direction=None, parent=None)

    goal_pos = maze.goal
    goal_block = Block(goal_pos, goal_pos)

    onDeck.append(start_node)

    while len(onDeck) > 0:
        curr_node = onDeck[0]
        curr_idx = 0

        for idx, item in enumerate(onDeck):
            if item.cost < curr_node.cost:
                curr_node = item
                curr_idx = idx
        
        onDeck.pop(curr_idx)
        done.append(curr_node)

        if maze.done(curr_node.block):
            path = []
            curr = curr_node

            while curr is not None:
                path.append(curr.move)
                curr = curr.parent
            return path[::-1]
        
        childs = children(curr_node, maze)

        for child in childs:
            if child in done:
                continue

            child.creach = curr_node.creach + 1

            if dist_type == 'Euclidean':
                child.ctogo = sqrt(((child.block.p2.x - goal_block.p2.x) ** 2) + ((child.block.p2.y - goal_block.p2.y) ** 2))
            elif dist_type == 'Chebyshev':
                delta_p1 = max(abs(child.block.p1.x - goal_block.p1.x), abs(child.block.p1.y - goal_block.p1.y))
                delta_p2 = max(abs(child.block.p2.x - goal_block.p2.x), abs(child.block.p2.y - goal_block.p2.y))
                child.ctogo = max(delta_p1, delta_p2)

            for node in onDeck:
                if child == node and child.creach > node.creach:
                    continue
            
            onDeck.append(child)

def children(curr_node, maze):
    legal_neighbors = maze.legal_neighbors(curr_node.block)
    children = []
    for (legal_neighbor, legal_move) in legal_neighbors:
        child = Node(block=legal_neighbor, direction=legal_move, parent=curr_node)
        children.append(child)
    return children

def get_moves(path):
        arr_path = []
        for move in path:
            if move is None:
                continue
            elif move == Direction.RIGHT:
                arr_path.append("R")
            elif move == Direction.LEFT:
                arr_path.append("L")
            elif move == Direction.DOWN:
                arr_path.append("D")
            elif move == Direction.UP:
                arr_path.append("U")
        return(arr_path)

def main():
    maze = Maze(level_file="maps/level01.txt")
    print(f"Start at: {maze.start}")
    print(f"Goal at: {maze.goal}")
    path = planner(maze, "Euclidean")
    print(get_moves(path))
   

if __name__ == "__main__":
    main()