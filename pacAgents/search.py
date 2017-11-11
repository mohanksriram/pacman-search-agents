# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

class Node:
    """
    This class outlines the structure of a node in the search tree. 
    """
    def  __init__(self, moveList, state):
        """
         state: Search State
         moveList: Sequence of moves to reach the node.
        """
        self.moveList = moveList
        self.state = state

    def getMoveList(self):
        """
        Returns the sequence of moves for the search node.
        """
        return self.moveList
    
    def getState(self):
        """
        Returns the state for the search node.
        """
        return self.state

class Strategy:
    DFS = "Depth First Search"
    BFS = "Breadth First Search"
    UFS = "Uniform Cost Search"
    AStar = "A-Star Search"


def graphSearch(problem, strategy, corners = (0,0)):
    """
    Search based on the input strategy, and return a list of actions that reaches the
    goal.
    """
        
    moveList = []
    startState = problem.getStartState()

    #if problem.corners:
     #   corners = problem.corners

    for index, corner in enumerate(corners):
        # define closed node list, move list and default fringe
        closed = []
        fringe = util.Stack()

        # Decide fringe data structure based on the search strategy.
        if strategy == Strategy.DFS:
            fringe = util.Stack()
        elif strategy == Strategy.BFS:
            fringe = util.Queue()
        elif strategy == Strategy.UFS:
            fringe = util.PriorityQueue()
        elif strategy == Strategy.AStar:
            fringe = util.PriorityQueue()          


        
        print("latest start state is")
        print(startState)
        startNode = Node([], startState)

        # Load the node to fringe
        if strategy == Strategy.DFS or strategy == Strategy.BFS:
            fringe.push(startNode)
        else:
            fringe.push(startNode, 1)

        while(1):
            
            # Return failure if fringe is empty
            if fringe.isEmpty():
                return None

            node = fringe.pop()
            
            if problem.isGoalState(node.getState()):
                # return the move list for found goal state 
                print("goal state found")
                startState = node.getState()
                cornerMove = node.getMoveList()
                moveList.extend(cornerMove)
                break

            if node.getState() not in closed:
                # append node to already visited node list
                closed.append(node.getState())
                
                # Expand the node and add child nodes to fringe
                for childSuccessor in problem.getSuccessors(node.getState()):
                    # load state from the child successor
                    childState = childSuccessor[0]
                    childMove = childSuccessor[1]

                    childPriority = 0
                    
                    parentMove = node.getMoveList()

                    if childMove:
                        if node.getMoveList():
                            childNode = Node(parentMove + [childMove], childState)
                        else:
                            childNode = Node([childMove], childState)
                    else:
                        if node.getMoveList():
                            childNode = Node(parentMove, childState)        
                    
                    # Assign priority based on search type
                    if strategy == Strategy.UFS:
                        childPriority = childSuccessor[2]

                    elif strategy == Strategy.AStar:
                        childPriority = getPriorityValue(childNode, childState, problem)    
                            

                    if strategy == Strategy.DFS or strategy == Strategy.BFS:
                        fringe.push(childNode)
                    else:
                        fringe.push(childNode, childPriority)            

    print("returning move list")
    print(moveList)                    
    return moveList                    

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    return graphSearch(problem, Strategy.DFS)
                
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    return graphSearch(problem, Strategy.BFS)

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    return graphSearch(problem, Strategy.UFS)
    util.raiseNotDefined()

def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def getPriorityValue(node, position, problem):
    """
    Return A star priority value
    """
    moveList = node.getMoveList()
    g = problem.getCostOfActions(moveList)
    h = manhattanHeuristic(position, problem)

    return g + h    

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    return graphSearch(problem, Strategy.AStar)
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
