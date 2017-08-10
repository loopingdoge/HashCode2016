import os
import errno
import sys
from functools import reduce

file_name	= sys.argv[1]

file  = open('./in/' + file_name + ".in", "r")

defs = file.readline().split()
ROWS	= int(defs[0])
COLS	= int(defs[1])	
D 		= int(defs[2])	#DRONES
TURNS	= int(defs[3])
PAYLOAD = int(defs[4])
P 		= int(file.readline())	# #PRODUCTS
weights = [ int(x) for x in file.readline().split() ]
W 		= int(file.readline())
O 		= 0				# #Orders
warehouses = []

for w in range(W):
	coords 	= [ int(x) for x in file.readline().split() ]
	items 	= file.readline().split()
	warehouses.append({ 'coords': coords, 'items': items})

O = int(file.readline())

print('ROWS:', ROWS, 'COLS:', COLS, 'DRONES:', D, 'TURNS:', TURNS, 'PAYLOAD:', PAYLOAD, 'PRODUCT TYPES', P, 'WAREHOUSES', W, 'ORDERS', O)

orders = []
for o in range(O):
    coords 	= [ int(x) for x in file.readline().split() ]
    nitems  = int( file.readline() )
    items 	= [ int(x) for x in file.readline().split() ] # items.length == nitems
    orders.append({ 'coords': coords, 'items': items})

file.close() 

# PROLOG

outfile_name = "./out/" + file_name + ".pl"
if not os.path.exists(os.path.dirname(outfile_name)):
    try:
        os.makedirs(os.path.dirname(outfile_name))
    except OSError as exc: # Guard against race condition
        if exc.errno != errno.EEXIST:
            raise

outfile = open(outfile_name, "w")
items = []
itemsWithWarehouses = []
lastItemId = 0

for d in range(D):
	# drone(drone0).
	outfile.write("drone(drone{}).\n".format(d))

for w in range(W):
	# warehouse(warehouse0, [product5,product1,product0], coord(0, 0)).
	warehouse = warehouses[w]
	products = []
	for n in range(P):
		products += ['product{}'.format(n)] * int(warehouse['items'][n])
	items += products
	for p in products:
		itemsWithWarehouses.append({ 'id': "item{}".format(lastItemId), 'warehouse': "warehouse{}".format(w) })
		lastItemId = lastItemId + 1
	products = reduce((lambda x, y: x + ',' + y), products)	
	r = warehouse['coords'][0]
	c = warehouse['coords'][1]
	outfile.write("warehouse(warehouse{}, [{}], coord({}, {})).\n".format(w, products, r, c))

for p in range(P):
	# product(product0, 100).
	weight = weights[p]
	outfile.write("product(product{}, {}).\n".format(p, weight))

for o in range(O):
	# order(order1, [product3, product1], coord(1, 1)).
	order = orders[o]
	products = ['product{0}'.format(i) for i in order['items'] ]
	products = reduce((lambda x, y: x + ',' + y), products)	
	r = order['coords'][0]
	c = order['coords'][1]
	outfile.write("order(order{}, [{}], coord({}, {})).\n".format(d, products, r, c))

for id, item  in enumerate(items):
	outfile.write("item(item{}, {}).\n".format(id, item))

outfile.write("\n\n")
outfile.write("%%\n")
outfile.write("%%Initial State\n")
outfile.write("%%\n")

outfile.write("[\n")
for d in range(D):
	outfile.write("    at(drone{}, coord(0, 0)),\n".format(d))
	outfile.write("    weighs(drone{}, 0),\n".format(d))
for n, itemWithW in enumerate(itemsWithWarehouses):
	outfile.write("    at({}, {}){}\n".format(itemWithW['id'], itemWithW['warehouse'], "" if n == len(itemsWithWarehouses)-1 else ","))
outfile.write("]")

outfile.write("\n\n")
outfile.write("%%\n")
outfile.write("%%Final State\n")
outfile.write("%%\n")

outfile.write("[\n")
for o in range(O):
	order = orders[o]
	products = ['product{0}'.format(i) for i in order['items'] ]
	for p in range(len(products)):
		outfile.write("    at(product{}, order{}){}\n".format(p, o, "" if o == O-1 and p == len(products)-1 else ","))

outfile.write("]")


outfile.close()
