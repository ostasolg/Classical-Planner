#!/usr/bin/env python3

import sys
from Main import read_sas_file, convert_fdr_to_strips


def compute_h_max(initial_state, goal_state, actions, facts):

    delta = {fact: 0 if fact in initial_state else float('inf') for fact in facts}
    u = {action["name"]: len(action["preconditions"]) for action in actions}
    c = set()

    for action in actions:
        if len(action["preconditions"]) == 0:
            for fact in action["additions"]:
                update_delta(delta, fact, action)

    while not goal_state.issubset(c):

        k = select_next_fact(delta, facts, c)

        if k is not None:
            c.add(k)
            for action in actions:
                if k in action["preconditions"]:
                    u[action["name"]] -= 1
                    if u[action["name"]] == 0:
                        for fact in action["additions"]:
                            update_delta(delta, fact, action)
        else:
            return float('inf')
    return count_h_max(delta, goal_state)


def count_h_max(delta, goal_state):

    h_max = 0

    for fact in goal_state:
        tmp = delta.get(fact)
        if tmp > h_max:
            h_max = tmp
    return h_max


def select_next_fact(delta, facts, c):

    next_min = float('inf')
    next_fact = None

    for fact in facts:
        if fact not in c:
            tmp = delta.get(fact)
            if tmp < next_min:
                next_min = tmp
                next_fact = fact
    return next_fact


def update_delta(delta, fact, action):
    prev_max = 0

    for f in action["preconditions"]:
        tmp = delta.get(f)
        if tmp > prev_max:
            prev_max = tmp

    delta[fact] = min(delta.get(fact), prev_max + action["cost"])


def compute_h_max_for_lm_cut(initial_state, goal_state, actions, facts):
    delta = {fact: 0 if fact in initial_state else float('inf') for fact in facts}
    u = {action["name"]: len(action["preconditions"]) for action in actions}
    c = set()

    for action in actions:
        if len(action["preconditions"]) == 0:
            for fact in action["additions"]:
                update_delta(delta, fact, action)

    while not goal_state.issubset(c):

        k = select_next_fact(delta, facts, c)

        if k is not None:
            c.add(k)
            for action in actions:
                if k in action["preconditions"]:
                    u[action["name"]] -= 1
                    if u[action["name"]] == 0:
                        for fact in action["additions"]:
                            update_delta(delta, fact, action)
        else:
            return float('inf')

    return count_h_max(delta, goal_state), delta



def run_h_max(sas_file_input):

    sas_data = read_sas_file(sas_file_input)
    initial_state, goal_state, actions, facts = convert_fdr_to_strips(sas_data)

    h_max = compute_h_max(initial_state, goal_state, actions, facts)
    print(h_max)


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Expected input: python hmax.py <sas_file>")
        exit(0)

    sas_file = sys.argv[1],

    # sas_file = "elevators01.sas"
    run_h_max(sas_file[0])