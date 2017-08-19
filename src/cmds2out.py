import argparse

def get_droneID(drone):
    return drone[5:]

def get_productID(product):
    return product[7:]

def get_orderID(order):
    return order[5:]

def get_warehouseID(warehouse):
    return warehouse[9:]

def parse_deliver(line):
    line_content = line[8:-1]
    tokens = line_content.split(',')
    drone, product, order, _ = tokens
    return "{} {} {} {} {}\n".format(
        get_droneID(drone),
        "D",
        get_orderID(order),
        get_productID(product),
        "1"
    )

def parse_load(line):
    line_content = line[5:-1]
    tokens = line_content.split(',')
    drone, product, warehouse, _ = tokens
    return "{} {} {} {} {}\n".format(
        get_droneID(drone),
        "L",
        get_warehouseID(warehouse),
        get_productID(product),
        "1"
    )

def parse(str_input):
    line_output = ""
    for line in str_input.split('\n'):
        if line.startswith('load'):
            line_output += parse_load(line)
        elif line.startswith('deliver'):
            line_output += parse_deliver(line)
    return line_output

def compress_two_lines(line1, line2):
    res = ""
    line1_tokens = line2.split(' ')
    line2_tokens = line2.split(' ')
    if line1[:-1] == line2[:-1]:
        res += '{} {} {} {} {}\n'.format(
            line2_tokens[0],
            line2_tokens[1],
            line2_tokens[2],
            line2_tokens[3],
            int(line1_tokens[4]) + int(line2_tokens[4])
        )
    else:
        res += line1 + '\n'
    return res

def compress(file_output):
    compressed_output = ""
    lines = file_output.split('\n')
    for i in range(1, len(lines)):
        compressed_output += compress_two_lines(lines[i - 1], lines[i])
    return compressed_output

def main():
    parser = argparse.ArgumentParser(description="Hashcode 2016 - .cmds -> .out")
    parser.add_argument('problem', help='The problem name (inside ./in folder)')

    args = parser.parse_args()

    problem_name = args.problem
    with open('./out/' + problem_name + '.cmds', 'r') as cmdsfile:
        file_content = ""
        for line in cmdsfile:
            file_content += line
        uncompressed_output = parse(file_content)
        compressed_output = compress(uncompressed_output)
        order_count = len(compressed_output.split('\n')) - 1
        final_output = str(order_count) + '\n' + compressed_output
        with open('./out/' + problem_name + '.out', 'w') as outfile:
            outfile.write(final_output)

if __name__ == '__main__':
    main()
