import os, errno, sys, jinja2, argparse
from functools import reduce

parser = argparse.ArgumentParser(description="Hashcode 2016 - Input parser")
parser.add_argument('problem', help='The problem name (inside ./in folder)')
parser.add_argument("--planner", help='The planner name (inside ./src folder)', default='./src/planner.pl')
parser.add_argument('--debug', help='Print debug messages', action='store_true')

args = parser.parse_args()

problem_name = args.problem
planner_name = args.planner
debug = args.debug

planner_path = 'src/planners/' + planner_name + '.pl'
outfile_path = 'out/' + problem_name + '.pl'

needsNum = 0

file    = open('./in/' + problem_name + ".in", "r")

defs    = file.readline().split()
ROWS	= int(defs[0])
COLS	= int(defs[1])
D 		= int(defs[2])	#DRONES
TURNS	= int(defs[3])
PAYLOAD = int(defs[4])
P 		= int(file.readline())	# #PRODUCTS
weights = [int(x) for x in file.readline().split()]
W 		= int(file.readline())
O 		= 0				# #Orders
warehousess = []

for w in range(W):
    coords = [int(x) for x in file.readline().split()]
    items = file.readline().split()
    warehousess.append({'coords': coords, 'items': items})

O = int(file.readline())

if debug:
    print('Rows \t\t', ROWS)
    print('Cols \t\t', COLS)
    print('Drones \t\t', D)
    print('Turns \t\t', TURNS)
    print('Payload \t', PAYLOAD)
    print('Product types \t', P)
    print('Warehouses \t', W)
    print('Orders \t\t', O)

orders = []
for o in range(O):
    coords 	= [int(x) for x in file.readline().split()]
    nitems  = int(file.readline())
    items 	= [int(x) for x in file.readline().split()] # items.length == nitems
    orders.append({'coords': coords, 'items': items})

file.close()

# PROLOG

def drones_facts(drones_number):
    # drone(drone0).
    drones_pl = ""
    for drone_id in range(drones_number):
        drones_pl += "drone(drone{}).\n".format(drone_id)
    return drones_pl

def warehouses_facts(warehouses_number, warehouses):
    # warehouse(warehouse0, [product5,product1,product0], coord(0, 0)).
    warehouses_pl = ""
    items = []
    itemsWithWarehouses = []
    lastItemId = 0
    for warehouse_id in range(warehouses_number):
        warehouse = warehouses[warehouse_id]
        products = []
        for product_id in range(P):
            products += ['product{}'.format(product_id)] * int(warehouse['items'][product_id])
        items += products
        for p in products:
            itemsWithWarehouses.append({ 'id': "item{}".format(lastItemId), 'warehouse': "warehouse{}".format(warehouse_id) })
            lastItemId = lastItemId + 1
        products = reduce((lambda x, y: x + ',' + y), products)
        r = warehouse['coords'][0]
        c = warehouse['coords'][1]
        warehouses_pl += "warehouse(warehouse{}, coord({}, {})).\n".format(warehouse_id, r, c)
    return warehouses_pl, items, itemsWithWarehouses

def products_facts(products_number):
    # product(product0, 100).
    products_pl = ""
    for p in range(products_number):
        weight = weights[p]
        # outfile.write("product(product{}, {}).\n".format(p, weight))
        products_pl += "product(product{}, {}).\n".format(p, weight)
    return products_pl

def needsNumber_facts():
    global needsNum
    return "needsNum({}).".format(needsNum)

def orders_facts(orders_number):
    orders_pl = ""
    for o in range(orders_number):
        # order(order1, [product3, product1], coord(1, 1)).
        order = orders[o]
        products = ['product{0}'.format(i) for i in order['items']]
        products = reduce((lambda x, y: x + ',' + y), products)
        r = order['coords'][0]
        c = order['coords'][1]
        orders_pl += "order(order{}, [{}], coord({}, {})).\n".format(o, products, r, c)
    return orders_pl

def items_facts(items):
    items_pl = ""
    for id, item  in enumerate(items):
        items_pl += "item(item{}, {}).\n".format(id, item)
    return items_pl

def initial_state(drones_number, itemsWithWarehouses, orders_number):
    global needsNum
    initial_state_pl = ""
    needId = 0
    for d in range(drones_number):
        initial_state_pl += "at(drone{}, coord(0, 0)),\n".format(d)
        initial_state_pl += "weighs(drone{}, 0),\n".format(d)
    for n, itemWithW in enumerate(itemsWithWarehouses):
        initial_state_pl += "at({}, {}){}\n".format(
            itemWithW['id'],
            itemWithW['warehouse'],
            ","
        )
    for o in range(orders_number):
        order = orders[o]
        products = ['product{0}'.format(i) for i in order['items']]
        for n, p in enumerate(products):
            initial_state_pl += "need({}, {}, order{}){}\n".format(
                needId,
                p,
                o,
                "" if o == orders_number-1 and n == len(products)-1 else ","
            )
            needId = needId + 1
    needsNum += needId
    return initial_state_pl

def final_state(order_number, orders):
    final_state_pl = ""
    atId = 0
    for o in range(order_number):
        order = orders[o]
        products = ['product{0}'.format(i) for i in order['items']]
        for n, p in enumerate(products):
            final_state_pl += "at({}, {}, order{}){}\n".format(
                atId,
                p,
                o,
                "" if o == order_number-1 and n == len(products)-1 else ","
            )
            atId = atId + 1
    return final_state_pl

def render_template(world_facts, initial_state, final_state, turns, payload, rows, cols, filename, template_path, debug):
    context = {
        "world_facts": world_facts,
        "initial_state": initial_state,
        "final_state": final_state,
        "max_turns": turns,
        "payload": payload,
        "rows": rows,
        "cols": cols,
        "filename": filename,
        "debug": debug
    }
    path, filename = os.path.split(template_path)
    return jinja2.Environment(
        loader=jinja2.FileSystemLoader(path or './')
    ).get_template(filename).render(context)

if __name__ == "__main__":
    warehouses_facts, items, itemsWithWarehouses = warehouses_facts(W, warehousess)
    world_facts = ""
    world_facts += drones_facts(D)
    world_facts += warehouses_facts
    world_facts += products_facts(P)
    world_facts += items_facts(items)
    world_facts += orders_facts(O)

    initial_state = initial_state(D, itemsWithWarehouses, O)
    final_state = final_state(O, orders)

    world_facts +=needsNumber_facts()

    debug_line = ', reverse_print_stack(Moves)' if debug else ''

    res = render_template(
        world_facts, initial_state, final_state, TURNS, PAYLOAD, ROWS, COLS, problem_name, planner_path, debug_line
    )

    if not os.path.exists(os.path.dirname(outfile_path)):
        try:
            os.makedirs(os.path.dirname(outfile_path))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    with open(outfile_path, "w") as fh:
        fh.write(res)