import subprocess, os, sys, argparse

parser = argparse.ArgumentParser(description="Hashcode 2016")
parser.add_argument('problem', help='The problem name (inside ./in folder)')
parser.add_argument("--planner", help='The planner name (inside ./src folder)', default='planner')
parser.add_argument('--debug', help='Print debug messages', action='store_true')
parser.add_argument('--quiet', help='Print only the score', action='store_true')

args = parser.parse_args()

problem_name = args.problem
planner_name = args.planner
debug = '--debug' if args.debug else ''

dirname = os.path.dirname(os.path.abspath(__file__))

outPlannerFilePath = "out/{}.pl".format(problem_name)
outPlannerSolutionFilePath = "out/{}.cmds".format(problem_name)
outParsedSolutionFilePath = "out/{}.out".format(problem_name)

def quiet_print(text):
    if not args.quiet:
        print(text)

print("\n" + ' '.join(planner_name.capitalize().split("_")))

# remove old file if present
if os.path.isfile(outPlannerFilePath):
    os.remove(outPlannerFilePath)
quiet_print("\n- 1/4 PARSING INPUT FILE   (in/{}.in -> out/{}.pl)".format(problem_name, problem_name))
subprocess.call("python ./src/scripts/parser.py --planner {} {} {}".format(planner_name, debug, problem_name).split())

# continue only if the previous phase is completed correctly
if os.path.isfile(outPlannerFilePath):
    # remove old file if present
    if os.path.isfile(outPlannerSolutionFilePath):
        os.remove(outPlannerSolutionFilePath)
    quiet_print("\n- 2/4 STARTED PLANNING     (out/{}.pl -> out/{}.cmds)".format(problem_name, problem_name))
    if(debug):
        subprocess.call("swipl -G100g -T20g -L2g -l out/{}.pl -g profile(test) -t halt -q".format(problem_name).split())
        print("entered")
    else:
        subprocess.call("swipl -G100g -T20g -L2g -l out/{}.pl -g test -t halt -q".format(problem_name).split())
else:
    print("ERROR: Can't generate the planner for the input problem")
    sys.exit(0)

# continue only if the previous phase is completed correctly
if os.path.isfile(outPlannerSolutionFilePath):
    # remove old file if present
    if os.path.isfile(outParsedSolutionFilePath):
        os.remove(outParsedSolutionFilePath)
    quiet_print("\n- 3/4 PARSING OUTPUT FILE  (out/{}.cmds -> out/{}.out)".format(problem_name, problem_name))
    subprocess.call("python ./src/scripts/cmds2out.py {}".format(problem_name).split())
else:
    print("ERROR: Can't generate a solution for the input problem")
    sys.exit(0)

quiet_print("\n- 4/4 CALCULATING SCORE    (out/{}.out -> console)".format(problem_name))
sys.stdout.flush()
subprocess.call("python ./src/scripts/scoring.py {} {}".format(debug, problem_name).split())
