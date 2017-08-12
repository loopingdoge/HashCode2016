from fileinput import filename

import sys

def __charAt(String, i):
    j = i+1
    return String[i:j]

file_name	= sys.argv[1]
moves = open('./out/' + file_name + ".cmds", "r") #cmds stands for commands
output = open('./out/' + file_name + ".out", "w")

num_lines = sum(1 for line in moves)
moves.close()
moves = open('./out/' + file_name + ".cmds", "r")
previous = "first" #a special value for first iteration
current = ""
command = ""
prod_quantity = 1

outStr = ""

for line in moves.readlines():

    i = len(line)
    char = __charAt(line, i)
    while char != ",":
        line = line[:i]
        i -= 1
        char = __charAt(line, i)

    current = line
    if previous != "first":
        if  previous == line:
            previous = line
            prod_quantity += 1
            num_lines -= 1
            continue
        else:
            command = command + " " + str(prod_quantity) + "\n"
            outStr += command
            command = ""
            prod_quantity = 1


    x = line[0:4]
    y = line[0:7]
    if x == "load":
        index = 10
        while __charAt(current, index) != ",": #reading drone number
            command += __charAt(current, index)
            index += 1
        command += " L "
        index += 1
        warehouse = ""
        while __charAt(current, index) != ",": #reading warehouse number
            if __charAt(current, index).isdigit():
                warehouse += __charAt(current, index)
            index += 1
        warehouse += " "
        index += 1
        while __charAt(current, index) != ",": #reading product id number
            if __charAt(current, index).isdigit():
                command += __charAt(current, index)
            index += 1
        command += " " + warehouse
        previous = line
    elif y == "deliver":
        index = 13
        while __charAt(current, index) != ",":  # reading drone number
            command += __charAt(current, index)
            index += 1
        command += " D "
        index += 1

        product = ""
        while __charAt(current, index) != ",": #reading produc id number
            if __charAt(current, index).isdigit():
                product += __charAt(current, index)
            index += 1
        index += 1

        while __charAt(current, index) != ",": #reading customer number
            c = __charAt(current, index)
            if __charAt(current, index).isdigit():
                command += __charAt(current, index)
            index += 1
        index += 1
        command = command + " " + product



else:
    command = command + " " + str(prod_quantity)
    outStr += command

output.write("{}\n{}".format(str(num_lines), outStr))
moves.close()
output.close()


