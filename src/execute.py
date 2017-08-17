import subprocess, os, sys

problem_name = sys.argv[1]
debugger = sys.argv[2]
planner_name = ('./src/' + sys.argv[3] + '.pl') if (len(sys.argv) > 3 and sys.argv[3]) else './src/planner.pl'

dirname = os.path.dirname(os.path.abspath(__file__))

outPlannerFilePath = "out/{}.pl".format(problem_name)
outPlannerSolutionFilePath = "out/{}.cmds".format(problem_name)
outParsedSolutionFilePath = "out/{}.out".format(problem_name)

# remove old file if present
if os.path.isfile(outPlannerFilePath):
    os.remove(outPlannerFilePath)
print("-> 1/4 PARSING INPUT FILE -> {}.in \n".format(problem_name))
subprocess.call("python ./src/parser.py {} {} {}".format(problem_name, debugger, planner_name).split())

# continue only if the previous phase is completed correctly
if os.path.isfile(outPlannerFilePath):
    # remove old file if present
    if os.path.isfile(outPlannerSolutionFilePath):
        os.remove(outPlannerSolutionFilePath)
    print("-> 2/4 STARTED PLANNING -> {}.pl \n".format(problem_name))

    subprocess.call("swipl -l out/{}.pl -g test -t halt -q".format(problem_name).split())
else:
    print("ERROR: Can't generate the planner for the input problem")
    sys.exit(0)

# continue only if the previous phase is completed correctly
if os.path.isfile(outPlannerSolutionFilePath):
    # remove old file if present
    if os.path.isfile(outParsedSolutionFilePath):
        os.remove(outParsedSolutionFilePath)
    print("-> 3/4 PARSING OUTPUT FILE -> {}.out \n".format(problem_name))
    subprocess.call("python ./src/output.py {}".format(problem_name).split())
else:
    print("ERROR: Can't generate a solution for the input problem")
    sys.exit(0)
    
print("-> 4/4 CALCULATING SCORE \n")
subprocess.call("python ./src/scoring.py {} {}".format(problem_name, debugger).split())
