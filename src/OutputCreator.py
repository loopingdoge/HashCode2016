from fileinput import filename

def __charAt(String, i):
    j = i+1
    return String[i:j]

moves = open("../examples/actions.txt", "r")
output = open("../examples/output.txt", "w")

num_lines = sum(1 for line in moves)
output.write(str(num_lines) +"\n")
moves.close()
moves = open("../examples/actions.txt", "r")
previous = "first" #a special value for first iteration
current = ""
command = ""
prod_quantity = 1

for line in moves.readlines():
    current = line
    if previous != "first":
        if  previous == line:
            previous = line
            prod_quantity += 1
            continue
        else:
            command = command + " " + str(prod_quantity) + "\n"
            output.write(command)
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
        while __charAt(current, index) != ",": #reading produc id number
            if __charAt(current, index).isdigit():
                command += __charAt(current, index)
            index += 1
        command += " "
        index += 1
        while __charAt(current, index) != ")": #reading warehouse number
            if __charAt(current, index).isdigit():
                command += __charAt(current, index)
            index += 1
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

        while __charAt(current, index) != ")": #reading customer number
            c = __charAt(current, index)
            if __charAt(current, index).isdigit():
                command += __charAt(current, index)
            index += 1
        index += 1
        command = command + " " + product



else:
    command = command + " " + str(prod_quantity) + "\n"
    output.write(command)

moves.close()
output.close()

