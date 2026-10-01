class Direction:
    RIGHT = 0
    UP    = 1
    LEFT  = 2
    DOWN  = 3

class Position:
    def __init__(self, x, y):
        self.x = x              # Column
        self.y = y              # Row

    def dx(self, d):
        return Position(self.x + d, self.y)

    def dy(self, d):
        return Position(self.x, self.y + d)
    
    # Define the "less-than" to enable sorting by (col,row).
    def __lt__(self, other):
        return ((self.y < other.y) or
                ((self.y == other.y) and (self.x < other.x)))

    # Print (for debugging).
    def __repr__(self):
        return("(%2d,%2d)" % (self.x, self.y))
    
class Block:
    """
    A Block contains two cube pieces. Can occupy one space while standing or two spaces while laying.
    """

    def __init__(self, p1: Position, p2: Position):
        # Assert that p1 is smaller than or equal to p2. p1<p2 in laying condition. p1 == p2 in standing condition.
        assert (abs(p1.x -p2.x)==1 and abs(p1.y -p2.y)==1)
        self.p1 = p1
        self.p2 = p2

    def is_standing(self) -> bool:
        return self.p1.x == self.p2.x and self.p1.y == self.p2.y

    def left(self):
        if self.is_standing():
            return self.dx(-2, -1)
        elif self.p1.x == self.p2.x: # laying horizontal
            return self.dx(-1, -2)
        else:                        # laying vertical
            return self.dx(-1, -1)

    def right(self):
        if self.is_standing():
            return self.dx(2, 1)
        elif self.p1.x ==self.p2.x: # laying horizontal
            return self.dx(2, 1)
        else:                        # laying vertical
            return self.dx(1, 1)

    def up(self):
        if self.is_standing():
            return self.dy(2, 1)
        elif self.p1.x == self.p2.x: # laying horizontal
            return self.dy(1, 1)
        else:                        # laying vertical
            return self.dy(1, 2)

    def down(self):
        if self.is_standing():
            return self.dy(-2, -1)
        elif self.p1.x == self.p2.x: # laying horizontal
            return self.dy(-1, -1)
        else:                        # laying vertical
            return self.dy(-2, -1)

    def dx(self, d1, d2):
        return Block(self.p1.dx(d1), self.p2.dx(d2))

    def dy(self, d1, d2):
        return Block(self.p1.dy(d1), self.p2.dy(d2))

    # Print (for debugging).
    def __repr__(self):
        return("(%2d,%2d)" % (self.p1, self.p2))
    
class Node:
    def __init__(self, block, direction, parent, cost=0, creach=0, ctogo=0):
        self.block = block
        self.parent = parent
        self.direction = direction

        self.cost = cost
        self.creach = creach
        self.ctogo = ctogo