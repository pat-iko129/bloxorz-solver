from utils import Direction, Position, Block

class Maze:
    def __init__(self, level_file="maps/level01.txt"):
        self.map = None
        self.start = None
        self.goal = None
        self.width, self.height = self.mazetomap(level_file)

    def legal_move(self, b: Block) -> bool:
        try:
            legal = self.map[b.p1.x][b.p1.y] and self.map[b.p2.x][b.p2.y]
        except IndexError:
            legal = False 
            print("Points:"+ str(b.p1) + " or " + str(b.p2) + " are out of map")
            
        return legal

    def neighbours(self, b: Block) -> list:
        return [(b.up(), Direction.UP), (b.down(), Direction.DOWN), (b.left(), Direction.LEFT), (b.right(), Direction.RIGHT)]

    def legal_neighbors(self, b: Block) -> list:
        return [(n, move) for (n, move) in self.neighbours(b) if self.legal_move(n)]

    def done(self, b: Block) -> bool:
        return b.is_standing() and b.p1.x == self.goal.x and b.p1.y == self.goal.y

    def mazetomap(self, level_file):
        """
        Contains T/F values of whether it is legal to move there or not
        """
        file = open(level_file, "r")
        self.map = []
        height = 0
        width = 0
        for x, line in enumerate(file):
            row = []
            for y, char in enumerate(line):
                if char == 'S':
                    self.start = Position(x, y)
                    row.append(True)
                elif char == 'T':
                    self.goal = Position(x, y)
                    row.append(True)
                elif char == '0':
                    row.append(True)
                elif char == '-':
                    row.append(False)
            self.map.append(row)
            width = len(row)
            height += 1
        file.close()
        return height, width