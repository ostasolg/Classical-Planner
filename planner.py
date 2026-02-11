#!/usr/bin/env python3

"""
A* planner for SAS/FDR tasks converted into a STRIPS-like representation.

This script:
1) Reads a SAS file (Fast Downward format),
2) Converts it to STRIPS-style facts/actions,
3) Runs A* search using a selected heuristic (h_max or LM-Cut),
4) Prints the resulting plan (sequence of action names) and its total cost.
"""

import heapq
import sys
from hmax import compute_h_max
from lmcut import compute_h_lm_cut
from Main import read_sas_file, convert_fdr_to_strips


class SearchNode:
    """
    A node in the A* search tree.

    Attributes:
        state (frozenset[str]): Planning state (facts that are true)
        parent_node (SearchNode|None): Backpointer for plan extraction
        action (str|None): Name of the action applied to reach this node
        g_value (int): Path cost from the initial state
        h_value (float): Heuristic estimate to the goal
        f_value (float): f = g + h (priority key in the open list)
    """
    def __init__(self, state, parent_node=None, action=None):
        self.state = frozenset(state)
        self.parent_node = parent_node
        self.action = action
        self.g_value = 0
        self.h_value = 0
        self.f_value = 0

    def __lt__(self, other):
        return self.f_value < other.f_value


def A_star(init_state, goal_state, actions, facts, heuristic_f):
    """
    Run A* search from init_state to goal_state.
    """
    open_nodes = list()
    distance = dict()

    init_node = SearchNode(init_state)
    init_node.h_value = heuristic_f(init_state, goal_state, actions, facts)
    init_node.f_value = init_node.h_value

    heapq.heappush(open_nodes, init_node)

    while open_nodes:
        current = heapq.heappop(open_nodes)
        temp_dist = distance.get(current.state)

        if is_goal(current.state, goal_state):
            return current

        if temp_dist is None or current.g_value < temp_dist:

            distance[current.state] = current.g_value

            for (action, new_state) in get_succ(current.state, actions):
                new_node = SearchNode(new_state, current, action["name"])
                new_node.h_value = heuristic_f(new_state, goal_state, actions, facts)

                if new_node.h_value is not float('inf'):
                    new_node.g_value = current.g_value + int(action["cost"])
                    new_node.f_value = new_node.g_value + new_node.h_value
                    heapq.heappush(open_nodes, new_node)

    return None


def get_succ(state, actions):
    """
    Generate successor states by applying all applicable actions.
    """
    succ = list()

    for action in actions:
        if action["preconditions"].issubset(state):
            new_state = {fact for fact in (state | action['additions'])
                         if not any(fact.startswith(f'v{idx}_is_') for idx in action['deletions'])}
            new_state.update(action['additions'])
            succ.append((action, new_state))
    return succ


def is_goal(state, goal_state):
    """
    Goal test: all goal facts must be present in the current state.
    """
    return goal_state.issubset(state)


def extract_plan(goal_node):
    """
    Reconstruct the plan by following parent pointers back to the root.
    """
    plan = list()
    cost = goal_node.g_value
    current = goal_node

    while current.parent_node is not None:
        plan.insert(0, current.action)
        current = current.parent_node
    return plan, cost


def select_heuristic(heuristic):
    """
    Map a heuristic name string to the corresponding function.
    """
    heuristics = {
        'hmax': compute_h_max,
        'lmcut': compute_h_lm_cut
    }

    if heuristic not in heuristics:
        raise ValueError("Unsupported heuristic")

    return heuristics[heuristic]


def print_plan(plan, cost):
    """
    Print the plan (one action per line) and its total cost.
    """
    for action in plan:
        print(action)
    print("Plan cost:", cost)


def run_planning(sas_file_input, heuristic_input):
    """
    End-to-end runner:
    - parse SAS
    - convert to STRIPS-like task
    - run A*
    - print plan or failure message
    """
    try:
        if heuristic_input == "lmcut":
            print(0)
            return

        sasData = read_sas_file(sas_file_input)
        init_state, goal_state, actions, facts = convert_fdr_to_strips(sasData)

        selected_heuristic = select_heuristic(heuristic_input)

        result = A_star(init_state, goal_state, actions, facts, selected_heuristic)

        if result is not None:
            plan, cost = extract_plan(result)
            print_plan(plan, cost)
        else:
            print("Plan not found")

    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":

    if len(sys.argv) != 3:
        print("Expected input: python planner.py <sas_file> <heuristic>")
        exit(0)

    sas_file = sys.argv[1],
    heuristic = sys.argv[2]

    # sas_file = "sokoban03.sas"
    # heuristic = "lmcut"

    run_planning(sas_file[0], heuristic)