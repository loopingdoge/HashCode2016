import subprocess, os, sys

problem_name = sys.argv[1]

dirname = os.path.dirname(os.path.abspath(__file__))

print("-> 1/4 PARSING INPUT FILE -> {}.in \n".format(problem_name))
subprocess.call("python ./src/parser.py {}".format(problem_name).split())

print("-> 2/4 STARTED PLANNING -> {}.pl \n".format(problem_name))
subprocess.call("swipl -l out/{}.pl -g test -t halt -q".format(problem_name).split())

print("-> 3/4 PARSING OUTPUT FILE -> {}.out \n".format(problem_name))
subprocess.call("python ./src/output.py {}".format(problem_name).split())

print("-> 4/4 CALCULATING SCORE \n")
subprocess.call("python ./src/scoring.py {}".format(problem_name).split())


