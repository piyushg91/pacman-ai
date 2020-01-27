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

import heapq
from game import Directions
from position_search_problem import PositionSearchProblem
from queue import Queue



def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: PositionSearchProblem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    visited = set()
    stack = [start]
    parent_map = {}
    while stack:
        current = stack.pop()
        if current in visited:
            continue
        visited.add(current)
        if problem.isGoalState(current):
            directions = get_path_from_start_to_goal(parent_map, current)
            return directions
        successors = problem.getSuccessors(current)
        for neighbor in successors:
            coordinate, direction, cost = neighbor
            if coordinate in visited:
                continue
            parent_map[coordinate] = (current, direction, cost)
            stack.append(coordinate)
    raise Exception('Could not find path')

def get_path_from_start_to_goal(parent_map, goal):
    directions = []
    back_track_current = goal
    while back_track_current in parent_map:
        next_back_track_current, d, _ = parent_map[back_track_current]
        directions.append(d)
        back_track_current = next_back_track_current
    directions.reverse()
    print('Length of direction ' + str(len(directions)))
    return directions


def breadthFirstSearch(problem: PositionSearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    q = Queue()
    start = problem.getStartState()
    parent_map, visited = {}, set()
    q.put((start, 0))
    while not q.empty():
        next_node, node_cost = q.get()
        if next_node in visited:
            continue
        visited.add(next_node)
        if problem.isGoalState(next_node):
            directions = get_path_from_start_to_goal(parent_map, next_node)
            return directions
        for neighbor in problem.getSuccessors(next_node):
            coordinate, direction, cost = neighbor
            cost += node_cost
            if coordinate in visited:
                continue
            if coordinate not in parent_map or parent_map[coordinate][2] > cost:
                parent_map[coordinate] = (next_node, direction, cost)
            q.put((coordinate, cost))
    raise Exception('Could not find path')


def uniformCostSearch(problem: PositionSearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    parent_map, explored, start = {}, set(), problem.getStartState()
    count = 0
    priority_queue = [(0, count, start)]
    total_nodes_expanded = 0
    while priority_queue:
        frontier_cost, _, min_frontier = heapq.heappop(priority_queue)
        if min_frontier in explored:
            continue
        explored.add(min_frontier)
        total_nodes_expanded += 1
        if problem.isGoalState(min_frontier):
            directions = get_path_from_start_to_goal(parent_map, min_frontier)
            print('The total number of nodes expanded was ' + str(total_nodes_expanded))
            return directions
        for neighbor in problem.getSuccessors(min_frontier):
            coordinate, direction, cost = neighbor
            total_cost = frontier_cost + cost
            if coordinate not in explored:
                # Only update the parent_map if cost is less
                node_info = (min_frontier, direction, total_cost)
                count += 1
                heapq.heappush(priority_queue, (total_cost, count, coordinate))
                if coordinate not in parent_map:
                    parent_map[coordinate] = node_info
                else:
                    existing_total_cost = parent_map[coordinate][2]
                    if total_cost < existing_total_cost:
                        parent_map[coordinate] = node_info
    raise Exception('Could not find path')


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: PositionSearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    parent_map, explored, start = {}, set(), problem.getStartState()
    # full_cost, count, frontier_cost, frontier
    priority_queue = [(0, 0, 0, start)]
    count = 0
    total_nodes_expanded = 0
    while priority_queue:
        _, _, frontier_cost, min_frontier = heapq.heappop(priority_queue)
        if min_frontier in explored:
            continue
        explored.add(min_frontier)
        total_nodes_expanded += 1
        if problem.isGoalState(min_frontier):
            directions = get_path_from_start_to_goal(parent_map, min_frontier)
            print('The total number of nodes expanded was ' + str(total_nodes_expanded))
            return directions
        for neighbor in problem.getSuccessors(min_frontier):
            coordinate, direction, cost = neighbor
            total_cost = frontier_cost + cost
            total_cost_with_heuristic = total_cost + heuristic(coordinate, problem)
            if coordinate not in explored:
                # Only update the parent_map if cost is less
                node_info = (min_frontier, direction, total_cost)
                count += 1
                heapq.heappush(priority_queue, (total_cost_with_heuristic, count, total_cost, coordinate))
                if coordinate not in parent_map:
                    parent_map[coordinate] = node_info
                else:
                    existing_total_cost = parent_map[coordinate][2]
                    if total_cost < existing_total_cost:
                        parent_map[coordinate] = node_info
    raise Exception('Could not find path')


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
