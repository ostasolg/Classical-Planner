#!/usr/bin/env python3
"""
Implementation of the LM-Cut heuristic for STRIPS planning tasks.
"""

from collections import deque
import sys
from Main import read_sas_file, convert_fdr_to_strips
from hmax import compute_h_max_for_lm_cut


def compute_h_lm_cut(initial_state, goal_state, actions, facts):
    """
    Compute the LM-Cut heuristic value.

    The algorithm repeatedly:
    1) Computes h_max and fact costs (delta),
    2) Builds a justification graph based on h_max supporters,
    3) Finds an s-t cut in the graph,
    4) Extracts landmarks and subtracts their cost from actions,
    until h_max becomes zero.
    """
    if len(initial_state) > 1 or len(goal_state) > 1:
        initial_state, goal_state, actions, facts = transform_prob(initial_state, goal_state, actions, facts)

    h_lm_cut = 0
    h_max, delta = compute_h_max_for_lm_cut(initial_state, goal_state, actions, facts)
    supporters = {action["name"]: None for action in actions}

    while h_max != 0:

        supporters = count_supporters(supporters, actions, delta)
        justification_g, justification_g_transpose = create_justification_graph(actions, facts, supporters)
        N_star, N_zero = s_t_cut(justification_g, justification_g_transpose, initial_state, goal_state)

        landmarks, lm_cost = find_landmarks(N_star, N_zero, justification_g)
        actions = adjust_landmarks(landmarks, lm_cost, actions)
        h_lm_cut += lm_cost
        h_max, delta = compute_h_max_for_lm_cut(initial_state, goal_state, actions, facts)

    return h_lm_cut


def adjust_landmarks(landmarks, lm_cost, actions):
    """
    Reduce the cost of all landmark actions by lm_cost.
    """
    for action in actions:
        if action["name"] in landmarks:
            action["cost"] -= lm_cost
    return actions


def find_landmarks(N_star, N_zero, justification_g):
    """
    Identify landmark actions from the s-t cut and compute their minimum cost.
    """
    landmarks = set()
    lm_costs = set()
    for fact in N_zero:
        for edge in justification_g[fact]:
            if edge[0] in N_star:
                landmarks.add(edge[1])
                lm_costs.add(edge[2])
    if len(lm_costs) != 0:
        min_cost = min(lm_costs)
    else:
        min_cost = 0
    return landmarks, min_cost


def s_t_cut(justification_graph, justification_graph_transpose, initial_state, goal_state):
    """
        Compute the s-t cut in the justification graph.
    """
    N_star = set()
    N_zero = set()
    goal_fact = next(iter(goal_state))
    init_fact = next(iter(initial_state))
    N_star.add(goal_fact)
    queue = deque()
    queue.append(goal_fact)

    while len(queue):
        current = queue.popleft()
        for edge in justification_graph_transpose[current]:
            action_cost = edge[2]
            if action_cost == 0 and edge[0] not in N_star:
                N_star.add(edge[0])
                queue.append(edge[0])

    queue2 = deque()
    for edge in justification_graph[init_fact]:
        if edge[0] not in N_star:
            N_zero.add(init_fact)
            queue2.append(init_fact)
            break

    while len(queue2):

        current = queue2.popleft()
        for edge in justification_graph[current]:
            if edge[0] not in N_star and edge[0] not in N_zero:
                N_zero.add(edge[0])
                queue2.append(edge[0])

    return N_star, N_zero


def create_justification_graph(actions, facts, supporters):
    """
        Build the justification graph and its transpose.
    """
    adj = {fact: set() for fact in facts}
    adj_transpose = {fact: set() for fact in facts}

    for action in actions:
        action_supporter = supporters[action["name"]]
        edges = set(adj.get(action_supporter))
        for eff in action["additions"]:
            edges_trans = set(adj_transpose.get(eff))
            edge_trans = (action_supporter, action["name"], action["cost"])
            edges_trans.add(edge_trans)
            adj_transpose[eff] = frozenset(edges_trans)

            edge = (eff, action["name"], action["cost"])
            edges.add(edge)
        adj[action_supporter] = frozenset(edges)

    return adj, adj_transpose


def count_supporters(supporters, actions, delta):
    """
        Select the h_max supporter for each action.
    """
    for action in actions:
        max_d = -1
        supp = None
        for fact in action["preconditions"]:
            if delta[fact] > max_d:
                max_d = delta[fact]
                supp = fact
            elif delta[fact] == max_d:
                for i in range(len(fact)):
                    if i >= len(fact):
                        break
                    if i >= len(supp):
                        max_d = delta[fact]
                        supp = fact
                    if fact[i] > supp[i]:
                        max_d = delta[fact]
                        supp = fact

        supporters[action["name"]] = supp

    return supporters


def transform_prob(initial_state, goal_state, actions, facts):
    """
    Transform the problem to have a single initial fact and a single goal fact.
    """
    if len(initial_state) > 1:
        new_fact = "I"
        new_initial_state = set()
        new_initial_state.add(new_fact)
        new_action = {"name": "new_init",
                      "preconditions": {"I"},
                      "additions": initial_state,
                      "deletions": {},
                      "cost": 0}
        actions.append(new_action)
        facts.add(new_fact)
        initial_state = new_initial_state

    if len(goal_state) > 1:
        new_fact = "G"
        new_goal_state = set()
        new_goal_state.add(new_fact)
        new_action = {"name": "new_goal",
                      "preconditions": goal_state,
                      "additions": new_fact,
                      "deletions": {},
                      "cost": 0}
        actions.append(new_action)
        facts.add(new_fact)
        goal_state = new_goal_state

    return initial_state, goal_state, actions, facts


def run_lm_cut(sas_file_input):
    """
    Load a SAS file, convert it to STRIPS, and compute LM-Cut.
    """
    sas_data = read_sas_file(sas_file_input)
    initial_state, goal_state, actions, facts = convert_fdr_to_strips(sas_data)

    lm_cut = compute_h_lm_cut(initial_state, goal_state, actions, facts)
    print(lm_cut)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Expected input: python lmcut.py <sas_file>")
        exit(0)

    # Read input SAS file
    sas_file = sys.argv[1],

    # sas_file = "elevators01.sas"
    run_lm_cut(sas_file[0])