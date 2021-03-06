#!/usr/local/bin/python3
import itertools
import threading
import time
import sys
import argparse
from math import floor
from datetime import datetime
from subprocess import check_output

current_planner = ''
animation_active = False

def animate():
    global animation_active, current_planner
    animation_active = True
    for c in itertools.cycle(['|', '/', '-', '\\']):
        if not animation_active:
            break
        sys.stdout.write('\r' + c + ' ' + current_planner)
        sys.stdout.flush()
        time.sleep(0.1)

def execution_score_time(problem_name, planner_name):
    global animation_active, current_planner
    current_planner = planner_name
    t = threading.Thread(target=animate)
    t.start()
    start_time = datetime.now()
    byte_output = check_output('python ./src/scripts/execute.py {} --planner {}'.format(problem_name, planner_name), shell=True)
    end_time = datetime.now()
    elapsed_time = end_time - start_time
    animation_active = False
    sys.stdout.write('\r                        ')
    output_lines = byte_output.decode('utf-8').split("\n")
    score_line = output_lines[-2].split(" ")
    usedTurns_line = output_lines[-4].split(" ")

    score = 0
    turnsUsed = 0

    try:
        score = int(score_line[1])
        turnsUsed = int(usedTurns_line[2])
    except ValueError as e:
        print("Execution error in " + planner_name)
        sys.exit(1)
    return score, turnsUsed, floor(elapsed_time.microseconds / 1000)

def main():
    parser = argparse.ArgumentParser(description="Hashcode 2016")
    parser.add_argument('problem', help='The problem name (inside ./in folder)')
    parser.add_argument("--planner", help='The planner name (inside ./src/planners folder)', default='planner')
    parser.add_argument('--debug', help='Print debug messages', action='store_true')
    parser.add_argument('--quiet', help='Print only the score', action='store_true')

    args = parser.parse_args()

    problem_name = args.problem

    planners = ['drones_dfs', 'drones_shortest_random', 'drones_shortest',  'orders_dfs', 'orders_shortest_random', 'orders_shortest']
    # planners = ['planner',  'drones_planner', 'random_action_planner']
    planners_and_scores = []

    print()

    for planner_name in planners:
        score, turnsUsed, time = execution_score_time(problem_name, planner_name)
        planners_and_scores.append({
            'name': ' '.join(planner_name.capitalize().split("_")),
            'score': score,
            'time': str(time),
            'turns_used': turnsUsed
        })

    print()

    for p in planners_and_scores:
        print('- {}\n    Score: {}\n    Turns used: {}\n    Time: {}ms\n'.format(p['name'], p['score'], p['turns_used'], p['time']))

    planners_and_scores.sort(key=lambda p: p['score'], reverse=True)

    print('\nMax score by: {} ({})'.format(planners_and_scores[0]['name'], planners_and_scores[0]['score']))

if __name__ == '__main__':
    main()
