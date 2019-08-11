import util
from sudoku import SudokuSearchProblem
from maps import MapSearchProblem

################ Node structure to use for the search algorithm ################
class Node:
    def __init__(self, state, action, path_cost, parent_node, depth):
        self.state = state
        self.action = action
        self.path_cost = path_cost
        self.parent_node = parent_node
        self.depth = depth

########################## DFS for Sudoku ########################
## Choose some node to expand from the frontier with Stack like implementation
def sudokuDepthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Return the final values dictionary, i.e. the values dictionary which is the goal state  
    """

    def convertStateToHash(values):
        """ 
        values as a dictionary is not hashable and hence cannot be used directly in the explored/visited set.
        This function changes values dict into a unique hashable string which can be used in the explored set.
        You may or may not use this
        """
        l = list(sorted(values.items()))
        modl = [a+b for (a, b) in l]
        return ''.join(modl)

    ## YOUR CODE HERE
    # util.raiseNotDefined()
    visited = set()
    frontier = util.Stack()
    frontier.push(problem.getStartState())

    while True :
        state = frontier.pop()
        hashed = convertStateToHash(state)
        if hashed in visited :
            continue
        else :
            if problem.isGoalState(state) :
                return state
            visited.add(hashed)
            succ = problem.getSuccessors(state)
            for el in succ :
                frontier.push(el[0])




######################## A-Star and DFS for Map Problem ########################
## Choose some node to expand from the frontier with priority_queue like implementation

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def heuristic(state, problem):
    # It would take a while for Flat Earther's to get accustomed to this paradigm
    # but hang in there.

    """
        Takes the state and the problem as input and returns the heuristic for the state
        Returns a real number(Float)
    """
    # util.raiseNotDefined()
    p_state = problem.G.node[state]
    p_end = problem.G.node[problem.end_node]
    p1 = ((p_state['x'],0,0),(p_state['y'],0,0))
    p2 = ((p_end['x'],0,0),(p_end['y'],0,0))
    # print(p_state,p_end)
    return util.points2distance(p1,p2)

def AStar_search(problem, heuristic=nullHeuristic):

    """
        Search the node that has the lowest combined cost and heuristic first.
        Return the route as a list of nodes(Int) iterated through starting from the first to the final.
    """

    # util.raiseNotDefined()
    def get_path(parent,state) :
        ans = []
        while state != -1 :
            ans.append(state)
            state = parent[state]
        ans.reverse()
        return ans

    frontier = util.PriorityQueue()
    start = problem.getStartState()
    frontier.push(start,0)

    visited = set()

    min_cost = {}
    min_cost[start] = 0

    parent = {}
    parent[start] = -1

    while True :
        state = frontier.pop()
        if state in visited :
            continue
        visited.add(state)
        if problem.isGoalState(state) :
            return get_path(parent,state)
        else :
            succ = problem.getSuccessors(state)
            for el in succ :
                frontier.update(el[0],el[2] + min_cost[state] + heuristic(el[0],problem))

                if not el[0] in min_cost :
                    min_cost[el[0]] = el[2] + min_cost[state]
                    parent[el[0]] = state
                else :
                    if min_cost[el[0]] > el[2] + min_cost[state] :
                        min_cost[el[0]] = el[2] + min_cost[state]
                        parent[el[0]] = state

