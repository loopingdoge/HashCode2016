
#python3 src/scoring.py test googleOutputExample

import sys

class Location:
    def __init__(self, r, c):
        self.r = r                      # row
        self.c = c                      # column

class Order:
    def __init__(self, loc, items):
        self.location   = loc               # Location
        self.items      = items             # {itemID:quantity}

    def delivered(self, itemID, quantity):
        itemQ = self.items.get(itemID, 0)
        if itemQ < quantity:                # Delivered not needed items
            raise Exception('Delivered items not in the order')
        elif itemQ == quantity:        # Delivered all item of type = itemID
            self.items.pop(itemID)
            if not self.items:
                orderCompleted()
        else:                                       # Delivered some items
            self.items[itemID] -= quantity
        # print(self.items)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        strOut = "coords: {},{}\n".format(self.location.r, self.location.c)
        for key in self.items:
            strOut += "product: {} quantity: {}\n".format(key, self.items[key])
        return strOut

class Warehouse:
    def __init__(self, loc, items):
        self.location    = loc               # Location
        self.items  = items             # {itemID:quantity}

    def getItems(self, itemID, quantity):
        itemQ = self.items.get(itemID, 0)
        if itemQ < quantity:
            raise Exception('Not enough items in the warehouse')
        else:
            self.items[itemID] -= quantity
            # print(self.items)
            return {itemID: quantity}

    def putItems(self, itemID, quantity):
        self.items[itemID] = self.items.get(itemID, 0) + quantity
        # print(self.items)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        strOut = "coords: {},{}\n".format(self.location.r, self.location.c)
        for key in self.items:
            strOut += "product: {} quantity: {}\n".format(key, self.items[key])
        return strOut

class Drone:
    items    = {}
    commands = []                       # Command Array
    def __init__(self, loc):
        self.location = loc               # Location

    def pushCommand(self, command):
        self.commands = self.commands + [command]
        # print(self.commands)

    def popCommand(self):
        return self.commands.pop(0)

    def loadItems(self, itemID, quantity):
        self.items[itemID] = self.items.get(itemID, 0) + quantity
        finalW = 0  # calulate final weight
        for ID in self.items:
            finalW += self.items[ID] * item[ID]
        if finalW > MAX_W:
            raise Exception('Max drone load weight passed')
        print(self.items)

    def unloadItems(self, itemID, quantity):
        itemQ = self.items.get(itemID, 0)
        if itemQ < quantity:
            raise Exception('Not enough items carried by the drone')
        else:
            self.items[itemID] -= quantity
            print(self.items)
            return {itemID: quantity}

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        strOut = "coords: {},{}\n".format(self.location.r, self.location.c)
        for command in self.commands:
            strOut += command.getString()
        return strOut

class Command:
    def __init__(self, tag, data, product, quantity):
        self.tag    = tag               # 'L': Load, U': Unload, 'D': Deliver, 'W': Wait
        self.data   = data              # 'L/U': WharehouseID, 'D': OrderID, 'W': #turns
        self.product = product          #product id
        self.quantity = quantity
        # self.items  = items


    def getString(self):
        return "{} {} {} {}\n".format(self.tag, self.data, self.product, self.quantity)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "{} {} {} {}".format(self.tag, self.data, self.product, self.quantity)
        #self.tag + ": " + self.data + " " + self.product + " " + self.quantity


def orderCompleted():
    # increments score
    global score
    score += int( ( ( T - turn ) / T ) * 100 ) + ( T - turn ) % T > 0       # rounded up
    print('Order completed: turn {}, score = {}'.format(turn, score))


# Google input parsing
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




# Google output parsing
file_name	= sys.argv[2]

file  = open('./in/' + file_name + ".in", "r")

defs = file.readline().split()

COMMANDS_NUM = int(defs[0])
dronesList = []      #drones array

for c in range(COMMANDS_NUM):
    defs = file.readline().split()
    droneID = int(defs[0])
    tag = defs[1] # 'L': Load, 'D': Deliver, 'W': Wait
    data = int(defs[2]) # 'L': WharehouseID, 'D': OrderID, 'W': #turns
    if tag is 'L' or tag is 'D':
        product = int(defs[3])
        quantity = int(defs[4])
        command = Command(tag, data, product, quantity)
        #print('droneID:', droneID, 'tag:', tag, 'data:', data, 'product:', product, 'quantity:', quantity)
    else:
        # wait case
        command = Command(tag, data, None, None)

    if len(dronesList) == droneID:        # the drone is not in the array yet
        drone = Drone(Location(0, 0))
        dronesList = dronesList + [drone]
    dronesList[droneID].pushCommand(command)

file.close()

#print drones
for i,drone in enumerate(dronesList):
    print("drone {}:".format(i))
    print(drone)

# initialize warehouses objects
warehousesList = []   #warehouse array
for i,warehouse in enumerate(warehouses):
    products = {}     #products dictionary
    r = warehouse['coords'][0]
    c = warehouse['coords'][1]
    for n in range(P):
        products[n] = int(warehouse['items'][n])

    w = Warehouse(Location(r, c), products)
    warehousesList = warehousesList + [w]

#print warehouses
for i,warehouse in enumerate(warehousesList):
    print("warehouse {}:".format(i))
    print(warehouse)

# initialize orders objects
ordersList = []
for i,order in enumerate(orders):
    r = order['coords'][0]
    c = order['coords'][1]
    products = {}

    for p in range(len(order['items'])):
        if products.get(order['items'][p]) == None:
            products[order['items'][p]] = 1
        else:
            products[order['items'][p]] +=1

    o = Order(Location(r,c), products)
    ordersList = ordersList + [o]

#print orders
for i,order in enumerate(ordersList):
    print("order {}:".format(i))
    print(order)
