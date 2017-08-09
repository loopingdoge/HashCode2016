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
outfile = open("./out/" + file_name + ".pl", "w")
items = []

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

outfile.close()


