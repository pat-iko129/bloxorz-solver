import math
import time
from visualgrid import VisualGrid
import numpy as np
import bisect
import copy

######################################################################
#
#  Define the grids:
#
#    # = wall
#    R = block (starting position)
#    x = box (starting position)
#    o = target (box goal position)
#    * = "both x and o", a box starting on a goal position
#

grid00 = ['###############',
          '# R        O  #',
          '###############',]

grid01 = ['############',
          '#    #######',
          '# R    #####',
          '#         ##',
          '#         ##',
          '######  O  #',
          '#######   ##',
          '############',]

grid001 = ['############',
          '#    #######',
          '# R  # #####',
          '#    #    ##',
          '#    #    ##',
          '######  O  #',
          '#######   ##',
          '############',]

grid02 = ['#################',
          '#######       ###',
          '#    ##   ##  ###',
          '#         ##    #',
          '# R  #######  O #',
          '#    #######    #',
          '#############   #',
          '#################',]

grid03 = ['#################',
          '############    #',
          '##            R #',
          '##    #######   #',
          '##    ###########',
          '##    ###########',
          '####   b      ###',
          '###########    b#',
          '#   #######     #',
          '# o   hh      ###',
          '#    ############',
          '#################']



grid04 = ['#################',
          '#              R#',
          '#  b            #',
          '#               #',
          '#               #',
          '# h             #',
          '#               #',
          '#               #',
          '#               #',
          '#hh             #',
          '#oh             #',
          '#################']



grid05 = ['###########',
          '#######o  #',
          '#######o# #',
          '#######o# #',
          '# R       #',
          '# # # # ###',
          '#       ###',
          '###########']

grid06 = ['    #### ',
          '  ###  ##',
          ' ## x   #',
          '## x  # #',
          '# R#xx  #',
          '# oo  ###',
          '# oo###  ',
          '#####    ']

grid07 = ['###########',
          '########  #',
          '## x      #',
          '##   x x  #',
          '## ########',
          '## # o  ###',
          '#  # #  ###',
          '#  R o ####',
          '## # # ####',
          '##   o ####',
          '###########']

grid08 = ['  ####         ',
          '  #  #    #####',
          '  #  #    #   #',
          '  #  ######o# #',
          '####  x    o  #',
          '#   xx# ###o# #',
          '#   #   # #   #',
          '######### #R ##',
          '          #  # ',
          '          #### ']

grid09 = ['########',
          '###   ##',
          '#oRx  ##',
          '### xo##',
          '#o##x ##',
          '# # o ##',
          '#x *xxo#',
          '#   o  #',
          '########']

grid10 = ['  ###### ',
          '  #    # ',
          '  #  x # ',
          ' ####x # ',
          '## x x # ',
          '#oooo# ##',
          '#     R #',
          '##  #   #',
          ' ########']

grid11 = ['  #######',
          '# #ddddd#',
          '# # # # #',
          '  # R x #',
          '### ### #',
          '#d  ### #',
          '#dx d##o#',
          '##dx d#o#',
          ' ##dx  o#',
          '# ##dx#o#',
          '## ## #o#',
          '### #ddd#',
          '### #####']

# Better reverse
grid12 = ['##################',
          '#    # # # o #   #',
          '# R  # #o#   #   #',
          '#    # # #   #   #',
          '#    # # #   #   #',
          '#  x             #',
          '#  x             #',
          '#      #   #######',
          '#      #         #',
          '##################']

# Better forward
grid13 = ['#################',
          '#    # # #  #   #',
          '# R  # # # x#   #',
          '#    # # #  #   #',
          '#    # # #  #   #',
          '#               #',
          '#  o            #',
          '#      #  #######',
          '#      #        #',
          '#################']

#
#   CONFIGURATION: Select the GRID, TESTING, and FORWARD/BACKWARD.
#

# Choose the grid
grid = grid04
MAZE = None
# Choose whether to try the testing.
TESTING = False

# Choose the search tree growth direction.
FORWARD = True

#
#   Colors
#
WHITE  = [1.00, 1.00, 1.00]
BLACK  = [0.00, 0.00, 0.00]
RED    = [1.00, 0.00, 0.00]
BROWN  = [0.59, 0.29, 0.00]
DARKGRAY  = [0.25, 0.25, 0.25]
GRAY  = [0.75, 0.75, 0.750]


######################################################################
#
#   General SPACE and MAZE Class
#
#   Define
#    a) Individual spaces, which have a row and col.  These are
#       sortable by row, then column.  The also show the adjacents
#       spaces (RIGHT, UP, LEFT, DOWN) which are either None or
#       another valid space.  And a list of all valid neighbors
#       (adjacent spaces).
#
#    b) The entire maze, which is a collection of the spaces, computed
#       from the above grids.  This also extracts the special spaces
#       (block, targets) and provides visualization.
#
# Define the directions (just to be more verbose/clear).
class Direction:
    RIGHT = 0
    UP    = 1
    LEFT  = 2
    DOWN  = 3
    DIRECTIONS = (  RIGHT,      UP,    LEFT,    DOWN)
    OPPOSITES  = (   LEFT,    DOWN,   RIGHT,      UP)
    DELTAS     = ((0,  1.5), (-1.5, 0), (0, -1.5), (1.5, 0))
    DELTAS_H   = ((0,  1.5), ( -1,  0), (0, -1.5), ( 1,  0))  #deltas for when block is horizontal
    DELTAS_V   = ((0,   1),  (-1.5, 0), (0,  -1 ), (1.5, 0))  #deltas for when block is horizontal

# Define the individual spaces.  Note (i) the class defines a
# less-than operator, so spaces can be sorted (row, then column).  And
# (ii) the class provides the neighbors both by direction (important
# when pushing) or as a general list.
class Space:
    def __init__(self, r, c, t='tile'):
        # Define the variables.
        self.r = r              # Row
        self.c = c              # Column
        self.type = t

        # The adjacent/neighbors by direction and as a list.
        self.adjacent  = [None] * len(Direction.DIRECTIONS)
        self.neighbors = []

    def get_r(self):
        return self.r

    def get_c(self):
        return self.c
    
    def check_exist(self, spaces_list):
        for space in spaces_list:
            if space.r == self.r and space.c == self.c and space.type != 'hidden-unpressed':
                return True
        return False


    # Define the "less-than" to enable sorting by (row,col).
    def __lt__(self, other):
        return ((self.r < other.r) or
                ((self.r == other.r) and (self.c < other.c)))
    def __eq__(self, other):
        return ((self.r == other.r) and (self.c == other.c))
    
    # Print (for debugging).
    def __repr__(self):
        return("(%2d,%2d)" % (self.r, self.c))


class Block:
    def __init__(self, orientation, center):
        self.orientation = orientation #v for vertical 1x2 block, h for horizontal 2x1, u for up 1x1
        self.center = center
        self.state = (center,orientation)

    def adjacent(self,direction):
        if self.orientation == 'h':
            center = np.array(self.center) + np.array(Direction.DELTAS_H[direction])
            orientation = 'h'
            if abs(center[0]-self.center[0]) == 1.5 or abs(center[1]-self.center[1]) == 1.5:
                orientation = 'u'
        if self.orientation == 'v':
            center = np.array(self.center) + np.array(Direction.DELTAS_V[direction])
            orientation = 'v'
            if abs(center[0]-self.center[0]) == 1.5 or abs(center[1]-self.center[1]) == 1.5:
                orientation = 'u'
        if self.orientation == 'u':
            center = np.array(self.center) + np.array(Direction.DELTAS[direction])
            #print(np.array(self.center),'+', np.array(Direction.DELTAS[direction]))
            #print('direction:',direction)
            if direction ==0 or direction == 2:
                orientation = 'h'
            if direction ==1 or direction == 3:
                orientation = 'v'

        
        return (orientation, tuple(center))
        





# Define the maze, being the collection of spaces, including the
# neighbor connections.  This also identifies the special spaces and
# provides a visualization.
class Maze:
    def __init__(self, grid):
        # Check the size of the grid (maze).
        self.currgrid = 0
        rows = len(grid)
        cols = len(grid[0])
        for line in grid:
            assert len(line) == cols, "Inconsistent lines in grid (maze)"

        # Create the visual.
        self.visual = VisualGrid(rows, cols)

        # Create a sorted list of legal (non-wall) spaces.  Else mark black.
        self.spaces = []
        button_counter = 0
        for r in range(rows):
            for c in range(cols):
                if grid[r][c] == '#':
                    self.visual.color(r, c, BLACK)
                elif grid[r][c] == 'b':
                    self.visual.color(r, c, BROWN)
                    self.spaces.append(Space(r,c, 'button'))
                    button_counter += 1
                elif grid[r][c] == 'h':
                    self.visual.color(r, c, DARKGRAY)
                    print(r,c)
                    self.spaces.append(Space(r,c, 'hidden-unpressed'))
                else:
                    self.spaces.append(Space(r,c))
        self.spaces.sort()      # To be safe: they should already be sorted.

        # Pull out the block, and targets (in sorted order!).
        block  = [s for s in self.spaces if grid[s.r][s.c] in 'Rr']
        target = [s for s in self.spaces if grid[s.r][s.c] in 'Oo*']
        assert len(block) == 1,            "Must have exactly one block"
        print(len(target))
        assert len(target) == 1,           "Must have exactly one taget"

        self.block   = Block('u',(block[0].r,block[0].c))
        self.target = target

        # Add the targets to the visual.
        for target in self.target:
            self.visual.write(target.r, target.c, 'o')
        self.show(Node(self.block,self.spaces), wait=0.1)


    # Show the visual.
    def show(self, node=None, wait=0.1):
        # Grab default values if not specified.
        # if node == None: 
        #     block = self.block
        # else:
        #print('center:', node.block.center)
        for space in node.spaces:
            if space.type == 'hidden-unpressed':

                self.visual.color(int(space.r), int(space.c), DARKGRAY)
            if space.type == 'hidden-pressed':
                self.visual.color(int(space.r), int(space.c), GRAY)
        block = node.block
        if block.orientation == 'u':
            self.visual.color(int(block.center[0]), int(block.center[1]), RED)
        if block.orientation == 'h':
            self.visual.color(int(block.center[0]), int(block.center[1]+0.5), RED)
            self.visual.color(int(block.center[0]), int(block.center[1]-0.5), RED)
        if block.orientation == 'v':
            self.visual.color(int(block.center[0]+0.5), int(block.center[1]), RED)
            self.visual.color(int(block.center[0]-0.5), int(block.center[1]), RED)
        
        
        # Mark the robot as red, boxes as brown.
        # Show
        self.visual.show(wait)
        
        # Clear the colors.
        spectiles = {}
        for space in node.spaces:
            if space.type == 'hidden-unpressed':
                spectiles[(space.r,space.c)] = DARKGRAY
            elif space.type == 'hidden-pressed':
                spectiles[(space.r,space.c)] = GRAY  
            elif space.type == 'button':
                spectiles[(space.r,space.c)] = BROWN    
            else:
                spectiles[(space.r,space.c)] = WHITE    

        # if block.orientation == 'u':
        #     for s in [block]: 
        #         self.visual.color(s.center[0], s.center[1], spectiles[(s.center[0], s.center[1])])
        #         #if spectiles[(s.center[0], s.center[1])]== BROWN:
        #             #print('................')
        #             #print('h:',(s.center[0], s.center[1]), (s.center[0], s.center[1]))
        # if block.orientation == 'h':
        #     for s in [block]: 
        #         self.visual.color(s.center[0], s.center[1]+0.5, spectiles[(s.center[0], s.center[1]+0.5)])
        #         self.visual.color(s.center[0], s.center[1]-0.5, spectiles[(s.center[0], s.center[1]-0.5)])
        #         #if spectiles[(s.center[0], s.center[1]+0.5)] == BROWN or spectiles[(s.center[0], s.center[1]-0.5)] == BROWN:
        #             #print('................')
        #             #print('h:',(s.center[0], s.center[1]+0.5), (s.center[0], s.center[1]-0.5))
        # if block.orientation == 'v':
        #     for s in [block]: 
        #         self.visual.color(s.center[0]+0.5, s.center[1], spectiles[(s.center[0]+0.5, s.center[1])])
        #         self.visual.color(s.center[0]-0.5, s.center[1], spectiles[(s.center[0]-0.5, s.center[1])])
        #         #if spectiles[(s.center[0]+0.5, s.center[1])]== BROWN or spectiles[(s.center[0]-0.5, s.center[1])]== BROWN:
        #             #print('................')
        #             #print('v:',(s.center[0]+0.5, s.center[1]), (s.center[0]-0.5, s.center[1]))


    

    # Print (for debugging).
    def __str__(self):
        return(f"Block:   {self.block}\r\n" +
               f"Target: {self.target}")
    def __repr__(self):
        s = str(self)
        for space in self.spaces:
            s += (f"r\n<Space {space} -> [%7s,%7s,%7s,%7s]>"
                  % tuple('' if a is None else str(a) for a in space.adjacent))
        return(s)
    

# Node class. This retains the state and the search tree data.  Note,
# as we are instantiating the node as we build the tree, we already
# know the parent and can set the cost at instantiation.  Otherwise,
# this contains the children() function to determine, instantiate, and
# return the possible child nodes (states).
class Node:
    def __init__(self, block, spaces, parent = None):
        # Make sure the list of boxes is sorted!  Note, which box goes
        # to which target is irrelevant, so we always keep the list of
        # boxes and targets ordered.  This avoids duplicate states and
        # makes is easy to compare boxes and targets!

        # Save the state
        self.block = block
        self.creach = math.inf
        self.seen = False
        self.done = False
        self.cost = math.inf
        self.parent = parent
        self.children = None
        self.spaces = spaces
        self.seenbutton = False
        self.costtogo = 0
        if self.spaces[61].type == 'hidden-pressed':
            self.seenbutton = True
    # Compute the children, i.e. the nodes/states that can be reached
    # by one of the four actions (movement directions).  Return a list
    # of these nodes which must consider what happens to the boxes.
            

    def adjustSpaces(self):
        new_spaces = copy.deepcopy(self.spaces)

        for i in range(len(new_spaces)):
            if new_spaces[i].type == 'hidden-unpressed':
                new_spaces[i].type = 'hidden-pressed'
            elif new_spaces[i].type == 'hidden-pressed': 
                new_spaces[i].type = 'hidden-unpressed'


#maybe flip seen button?
        self.spaces = new_spaces

    def checklegal(self, orientation, center):
        
        if orientation == 'u':
            if not Space(int(center[0]),int(center[1])).check_exist(self.spaces):
                #print('failed u:',center[0],center[1])
                return False
        if orientation == 'h':
            if not Space(int(center[0]),int(center[1]+0.5)).check_exist(self.spaces) or not Space(int(center[0]),int(center[1]-0.5)).check_exist(self.spaces):
                #print('failed h:',center[0],center[1]+0.5)
                return False
            
        if orientation == 'v':
            if not Space(int(center[0]+0.5),int(center[1])).check_exist(self.spaces) or not Space(int(center[0]-0.5),int(center[1])).check_exist(self.spaces):
                #print(f'failed v: {(int(center[0]+0.5), int(center[1]))}',Space(int(center[0]+0.5),int(center[1])).check_exist(self.spaces))
                #print(f'failed v: {(int(center[0]-0.5), int(center[1]))}' ,Space(int(center[0]-0.5),int(center[1])).check_exist(self.spaces))
                return False
        # print('success: ', int(center[0]))
        return True

    # State Transition Function (moving forward in time): Given the space
    # occupied by the robot together with the list of spaces occupied by
    # the bxoes, as well as a direction of movement, return either None if
    # the movement is not possible or the new robot space and new list of
    # boxes.
    def isbuttonpressed(self):
        buttonlocations = []
        center = self.block.center
        orientation = self.block.orientation
        for space in self.spaces:
            if space.type == 'button':
                buttonlocations.append((space.r,space.c))
        if orientation == 'u':
            if (int(center[0]),int(center[1])) in buttonlocations:
                # print('hit u')
                # print('buttons:',buttonlocations)
                # print('center',center)
                return True
        if orientation == 'h':
            if (int(center[0]),int(center[1]+0.5)) in buttonlocations or (int(center[0]),int(center[1]-0.5)) in buttonlocations:
                # print('hit h')
                # print('buttons:',buttonlocations)
                # print('center',center)
                return True
        if orientation == 'v':
            if (int(center[0]+0.5),int(center[1])) in buttonlocations or (int(center[0]-0.5),int(center[1])) in buttonlocations:
                # print('hit v')
                # print('buttons:',buttonlocations)
                # print('center',center)
                return True
        # print('success: ', int(center[0]))
        return False
    
    def transition(self, block, direction):

        neworientation,newcenter = block.adjacent(direction)
        #print('direction:',direction, 'orientation:',neworientation,'newcenter:', newcenter)
        # If the move isnt legal then you cant move
        #print(Maze.checklegal('h',(1,1)))
        if self.checklegal(neworientation,newcenter) == False: 
            # print('failed:',neworientation, newcenter)
            return None
        return (Block(neworientation, newcenter))

    def getChildren(self, dict=None):
        # Build up the list of children, trying in each direction.
        children = []
        for direction in Direction.DIRECTIONS:
            # Try the movement (copy the boxes list to isolate changes).
            result= self.transition(self.block, direction)
            #print('direction:', direction, 'result', result)
            if result is not None:
                if dict.get(result.state) is not None and dict[result.state].spaces[61].type == self.spaces[61].type:
                    #print('adding a previously seen node')
                    children.append(dict[result.state])
                else: 
                    child_spaces = copy.deepcopy(self.spaces)
                    children.append(Node(result,child_spaces, self))
        self.children = children 
        # print('................')
        # for child in children:
        #     print('child:',child.block.center)
        # print('................')
        return children
    
    def distance(self, other):
        return abs(self.block.center[0] - other.block.center[0]) + abs(self.block.center[1] - other.block.center[1])
        if abs(self.block.center[0] - other.block.center[0]) + abs(self.block.center[1] - other.block.center[1])<=2.5:

            if self.block.orientation == 'h' and abs(self.block.center[0] - other.block.center[0])== 0 and self.block.center[0] >= other.block.center[0]:
                return abs(self.block.center[0] - other.block.center[0]) + abs(self.block.center[1] - other.block.center[1]+1)
            elif self.block.orientation == 'h' and abs(self.block.center[0] - other.block.center[0])== 0 and self.block.center[0] <= other.block.center[0]:
                return abs(self.block.center[0] - other.block.center[0]) + abs(self.block.center[1] - other.block.center[1]-1.5)
            
            elif self.block.orientation == 'v' and abs(self.block.center[1] - other.block.center[1])== 0 and self.block.center[1] >= other.block.center[1]:
                return abs(self.block.center[0] - other.block.center[0]+1.5) + abs(self.block.center[1] - other.block.center[1])
            elif self.block.orientation == 'v' and abs(self.block.center[1] - other.block.center[1])== 0 and self.block.center[1] <= other.block.center[1]:
                return abs(self.block.center[0] - other.block.center[0]-1.5) + abs(self.block.center[1] - other.block.center[1])
        else:
            return abs(self.block.center[0] - other.block.center[0]) + abs(self.block.center[1] - other.block.center[1])
    
    def distancecheb(self, other):
        deltax = abs(self.block.center[0] - other.block.center[0])
        deltay = abs(self.block.center[1] - other.block.center[1])
        
        # deltaheight = 0
        # if max(deltax,deltay) == 2.5:
        #     if deltax == 2.5:
        #         if self.block.orientation != 'v':
        #             deltaheight = 3
        #     if deltay == 2.5:
        #         if self.block.orientation != 'h':
        #             deltaheight = 3

        # if self.block.orientation != other.block.orientation:

        #     deltaheight = 5
        return max(deltax,deltay)
    
    def __lt__(self, other):
        selfcost = self.cost.copy()
        othercost = other.cost.copy()
        if self.spaces[61].type == 'hidden-unpressed':
            selfcost += 1000
        if other.spaces[61].type == 'hidden-unpressed':
            othercost  += 1000
        
        return (selfcost < othercost)
    
    def __eq__(self, other):
        if self is not None and other is not None:
            return (self.cost == other.cost)
        else: return False
    
    # Print (for debugging).
    def __repr__(self):
        s = "<Node R %s, B" % str(self.block)
        if self.cost == math.inf:
            s += " with Cost inf>"
        else:
            s += " with Cost %d>" % self.cost
            deltax = abs(self.block.center[0] - 9)
            deltay = abs(self.block.center[1] - 2)
        s += "With cost to go  %s" % str(max(deltax, deltay))
        return s

# Estimate the cost to go from state to goal
TOGOFACTOR = 2            # 0 or 1 or 2 or 10
def costtogo(node, goal):
    return TOGOFACTOR * node.distance(goal)

def costtogo1(node, goal):
    (r, c) = node.block.center
    nearby_wall = 4
    for space in node.spaces:
        (sr, sc) = (space.r, space.c)
        if sr <= r + 1.5 and sr > r and sc ==c :
            nearby_wall = nearby_wall - 1
        elif sr >= r-1.5 and sr < r and sc == c:
            nearby_wall = nearby_wall - 1
        elif sc <= c + 1.5 and sc > c and sr ==r :
            nearby_wall = nearby_wall - 1
        elif sc >= c-1.5 and sc < c and sr == r:
            nearby_wall = nearby_wall - 1
    return TOGOFACTOR * node.distance(goal) + nearby_wall * 3

def astar(initblock, target):
    nodes = {}
    print('maze spaces:',copy.deepcopy(MAZE.spaces))

    start = Node(initblock, copy.deepcopy(MAZE.spaces), None)
    button = []
    for i in range(len(MAZE.spaces)):
        if MAZE.spaces[i].type == 'button':
            button = MAZE.spaces[i]

    goal = Node(Block('u', (target[0].r,target[0].c)), copy.deepcopy(MAZE.spaces),None)
    button = Node(Block('u', (button.r,button.c)), copy.deepcopy(MAZE.spaces),None)
    # Use the start node to initialize the on-deck queue: it has no
    # parent (being the start), zero cost to reach, and has been seen.
    start.seen   = True
    start.creach = 0
    start.cost   = 0
    start.costtogo = costtogo1(start, button)
    start.parent = None
    onDeck = [start]
    L = len(MAZE.spaces)
    B = 2
    Nmax = L * math.comb(L-1, B)
    print("# of spaces %d" % L)
    print("# of buttons  %d"          % B)
    print("# of possible states %d" % Nmax)
    

    def report(text, cost):
        Nseen = len(nodes)
        Ndeck = len(onDeck)
        print(f"{text} nodes {Nseen:7f} ({100*Nseen/Nmax:6.3f}%), " +
              f"done {Ndone:7f} ({100*Ndone/Nmax:6.3f}%), " +
              f"on-deck {Ndeck:6f}, cost {cost:3f}")
    # Continually expand/build the search tree.
    Ndone = 0
    buttonseen = False
    doonce = True
    while True:
        # Make sure we have something pending in the on-deck queue.
        # Otherwise we were unable to find a path!
        if Ndone % 10000 == 0:
            report("So far", onDeck[0].cost)
        if not (len(onDeck) > 0):
            print('failed :(')
            print('...........................')
            return None

        # Grab the next state (first on the storted on-deck list).
        node = onDeck.pop(0)
        nodes[node.block.state] = node
        node.done = True
        if node.block.center == goal.block.center:
            goal = node
            break

        # Check the neighbors
        node.getChildren(nodes)
        # if node.spaces[61].type == 'hidden-pressed':
        #     for child in node.children:

        #         print('child type:',child.spaces[61].type)
        #         print('child center:',child.block.center)
        #         print('node center:',node.block.center)

        #     print('..............')
        for child in node.children:

            # if c.parent != None:
                #print('type:',c.spaces[61].type)
            # if child.spaces[61].type == 'hidden-pressed':
            #     print('new node:')
            #     print('child is bridge pressed:',child.spaces[61].type)
            #     print('distance from parent:',np.array(child.block.center)- np.array(node.block.center))

            if child.done:
                continue
            # Pre-compoute the new cost to reach this neighbor 
            creach = node.creach + node.distance(child)
            # If already seen (thereby on deck), compare the costs.
            if child.seen:
                if child.creach <= creach:
                    child.spaces = copy.deepcopy(node.spaces)
                    buttonpressed = child.isbuttonpressed()
                    if buttonpressed:
                        print(creach + costtogo1(child, goal))
                        child.adjustSpaces()
                    # If the child had lower cost, skip any changes.
                    continue
                else:
                    # Otherwise, remove the previous entry from the onDeck
                    onDeck.remove(child)
            else:
                child.spaces = copy.deepcopy(node.spaces)
                buttonpressed = child.isbuttonpressed()
                if buttonpressed:
                    child.adjustSpaces()
            # Save this new path and add to onDeck (ordered by the new cost)
            child.seen = True
            child.creach = creach
            child.seenbutton = child.isbuttonpressed()
            if child.spaces[61].type =='hidden-pressed':
                child.cost = creach + costtogo1(child, goal)
                child.costtogo = costtogo1(child, goal)
            else:
                child.cost = creach + costtogo1(child, button)
                child.costtogo = costtogo1(child, button)

            child.parent = node
            bisect.insort(onDeck, child)
            Ndone += 1


    # Create the path to the goal (backwards) and show
    report("At END", node.cost)
    path = [goal]
    while path[0].parent:
        path.insert(0, path[0].parent)
    return (path, nodes.values())


######################################################################
#
#  Main Code
#
if __name__== "__main__":

    # Test the Node class.
    if TESTING:
        print("Testing results:")
        testmaze = Maze(grid04)
        MAZE = testmaze
        testnode = Node(testmaze.block, copy.deepcopy(MAZE.spaces), None)
        for node in [testnode] + testnode.children():
            print(node.block.center)
            print()
            print(node, end=' ')
            input('hit return')
            testmaze.show(block=node.block, wait=' ')
        print("Regular results:")

    # Create/show the maze.
    maze = Maze(grid)
    MAZE = maze
    print(maze)

    # Run the planner.
    start_time = time.time()
    (path, nodes) = astar(maze.block, maze.target)
    end_time = time.time()

    # Show the steps.
    if not path:
        print("UNABLE TO FIND A PATH")
        input("Hit return to show all nodes")
        for node in nodes:
            print(node, end=' ')
            maze.show(node, wait=' ')
    else:
        print("Found path with %d steps" % (len(path)-1))
        input("Hit return to show path")
        print('runtime of', end_time-start_time)
        for node in path:
            print(node)
            maze.show(node, wait=0.5)
            #input('see next step:')
            #print(node.spaces[61].type)