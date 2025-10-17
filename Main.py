#!/usr/bin/env python3
import sys


def get_init_state(sas_data):
    return {f'v{index}_is_{val}' for index, val in enumerate(sas_data['initial_state'])}


def get_actions(sas_data):

    actions = []
    facts = set()

    def process_conditions(conditions, type_prefix):
        processed = set()
        for variable_i, val in conditions:
            fact_name = f'v{variable_i}_is_{val}'
            processed.add(fact_name)
            facts.add(fact_name)
        return processed

    for oper_tmp in sas_data['operators']:
        preconditions = process_conditions(oper_tmp['prevail'], 'pre')
        additions, deletions = set(), set()

        for variable_i, pre_val, post_val in (map(int, eff[1:]) for eff in oper_tmp['effects']):
            if pre_val != -1:
                pre_fact = f'v{variable_i}_is_{pre_val}'
                preconditions.add(pre_fact)
                facts.add(pre_fact)

            post_fact = f'v{variable_i}_is_{post_val}'
            additions.add(post_fact)
            facts.add(post_fact)
            deletions.add(variable_i)

        actions.append({
            'name': oper_tmp['name'],
            'preconditions': preconditions,
            'additions': additions,
            'deletions': deletions,
            'cost': oper_tmp['cost']
        })

    return actions, facts


def get_goal_state(sas_data):
    goal_state = {f'v{variable_i}_is_{val}' for variable_i, val in sas_data['goal']}
    facts = set(goal_state)

    return goal_state, facts


def convert_fdr_to_strips(sas_data):

    initial_state = get_init_state(sas_data)
    actions, new_facts = get_actions(sas_data)
    goal_state, new_facts_g = get_goal_state(sas_data)
    facts = ((initial_state | new_facts) | new_facts_g)

    return initial_state, goal_state, actions, facts


def process_numeric_part(segment, operator):
    if 'num_prevail' not in operator:
        operator['num_prevail'] = int(segment[0])
        operator['reading_prevail'] = True
        operator['reading_effects'] = False
    elif 'num_effects' not in operator and operator['reading_prevail']:
        operator['num_effects'] = int(segment[0])
        operator['reading_effects'] = True
        operator['reading_prevail'] = False
    else:
        operator['cost'] = int(segment[0])


def process_prevail_conditions(segment, operator):
    operator['prevail'].append((int(segment[0]), int(segment[1])))


def process_effects(segment, operator):
    operator['effects'].append((int(segment[0]), int(segment[1]), int(segment[2]), int(segment[3])))


def read_sas_file(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()

    data_obj = {
        'version': None,
        'metric': None,
        'variables': [],
        'initial_state': [],
        'goal': [],
        'operators': []
    }

    setting = None
    tmp_variable = None
    tmp_operator = None

    for line in lines:
        processed = False
        line = line.strip()

        if line == 'begin_version':
            setting, processed = 'ver', True
        elif line == 'end_version':
            setting, processed = None, True
        elif line == 'begin_metric':
            setting, processed = 'met', True
        elif line == 'end_metric':
            setting, processed = None, True
        elif line == 'begin_variable':
            tmp_variable = {'name': None, 'range': None, 'atoms': []}
            setting, processed = 'var', True
        elif line == 'end_variable':
            data_obj['variables'].append(tmp_variable)
            tmp_variable, setting, processed = None, None, True
        elif line == 'begin_state':
            setting, processed = 'init_s', True
        elif line == 'end_state':
            setting, processed = None, True
        elif line == 'begin_goal':
            setting, processed = 'goal_s', True
        elif line == 'end_goal':
            setting, processed = None, True
        elif line == 'begin_operator':
            tmp_operator = {'name': '', 'cost': None, 'prevail': [], 'effects': []}
            setting, processed = 'oper', True
        elif line == 'end_operator':
            data_obj['operators'].append(tmp_operator)
            tmp_operator, setting, processed = None, None, True

        if not processed:

            if setting == 'ver':
                if data_obj['version'] is None:
                    data_obj['version'] = int(line)
            elif setting == 'met':
                if data_obj['metric'] is None:
                    data_obj['metric'] = int(line)
            elif setting == 'var':
                if tmp_variable['name'] is None:
                    tmp_variable['name'] = line
                elif tmp_variable['range'] is None:
                    tmp_variable['range'] = int(line.split()[-1])
                else:
                    tmp_variable['atoms'].append(line)
            elif setting == 'init_s':
                data_obj['initial_state'].extend(map(int, line.split()))
            elif setting == 'goal_s':
                if not line.isdigit():
                    segm = line.split()
                    data_obj['goal'].append((int(segm[0]), int(segm[1])))
                else:
                    continue

            elif setting == 'oper':
                if tmp_operator['name'] == '':
                    tmp_operator['name'] = line
                else:
                    segment = line.split()
                    if len(segment) == 1:
                        process_numeric_part(segment, tmp_operator)
                    else:
                        if tmp_operator['reading_prevail']:
                            process_prevail_conditions(segment, tmp_operator)
                        elif tmp_operator['reading_effects']:
                            process_effects(segment, tmp_operator)

    return data_obj


def run_parsing(sas_file_input):

    sas_data = read_sas_file(sas_file_input)
    initial, goal, actions, facts = convert_fdr_to_strips(sas_data)

    print("Initial State:", initial)
    print("Goal:", goal)
    print("Facts:")
    for fact in facts:
        print(fact)
    print("Actions:")
    for action in actions:
        print("Action:", action['name'], "Preconditions:", action['preconditions'], "Additions:",
              action['additions'], "Deletions:", action['deletions'], "Cost:", action['cost'])


if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("Expected input: python Main.py <sas_file>")
        exit(0)

    sas_file = sys.argv[1],

    # sas_file = "blocks-4-0.sas"
    run_parsing(sas_file[0])
