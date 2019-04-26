#!/usr/local/bin/python3
import time
import sys
import argparse
import datetime
import subprocess 
import os, sys
from shutil import copy

def main():
    parser = argparse.ArgumentParser(description="Hashcode 2016")
    parser.add_argument('problem', help='The problem name (inside ./in folder)')
    parser.add_argument('--desc', help='The comparison description')

    args = parser.parse_args()

    problem_name = args.problem
    problem_desc= args.desc

    planners = ['drones_dfs', 'drones_shortest_random', 'drones_shortest',  'orders_dfs', 'orders_shortest_random', 'orders_shortest']
    # planners = ['drones_dfs']

    planners_and_scores = []

    date = datetime.datetime.now().strftime("%Y%d%m_%H:%M:%S")
    folder = "tests/" + date + "_" + problem_desc
    os.makedirs(folder)
    # the in/generated.in in the new folder
    copy("in/" + problem_name + ".in", folder + "/")

    for planner_name in planners:
        f = open(folder + "/" + problem_name + "_" + planner_name, "w")
        f.write(datetime.datetime.now().strftime("%Y%d%m_%H:%M:%S") + "\n")
        subprocess.call("timeout 3600 python ./src/scripts/execute.py {} --planner {} --debug".format(problem_name, planner_name).split(), stdout=f)
        f.write(datetime.datetime.now().strftime("%Y%d%m_%H:%M:%S"))
        f.close()

if __name__ == '__main__':
    main()
