#python3 src/scoring.py test googleOutputExample

import sys
import math
from math import sqrt

# True if the number of turns is insufficient to solve problem
# The score calculation is stopped
turnsLimitReached = False

# CLASSES

class Location:
    def __init__(self, r, c):
        self.r = r                      # row
        self.c = c                      # column

class Order:
    def __init__(self, id, loc, items):
        self.id         = id
        self.location   = loc               # Location
        self.items      = items             # {itemID:quantity}
        self.completedTurn = None

    def delivered(self, itemID, quantity):
        itemQ = self.items.get(itemID, 0)
        if itemQ < quantity:                # Delivered not needed items
            raise Exception('Delivered items not in the order')
        elif itemQ == quantity:        # Delivered all item of type = itemID
            self.items.pop(itemID)
            if not self.items:
                orderCompleted()
                self.completedTurn = turn
                print('    [COMPLETED] order{} turn{} score= {}'.format(self.id, self.completedTurn, self.getScore()))
        else:                                       # Delivered some items
            self.items[itemID] -= quantity
        # print(self.items)

    def getScore(self):
        if self.completedTurn: #if completed
            return int(math.ceil( ( (0.0 + T - self.completedTurn) / T ) * 100) )


    def __repr__(self):
        return self.__str__()

    def __str__(self):
        strOut = "coords: {},{}\n".format(self.location.r, self.location.c)
        for key in self.items:
            strOut += "product: {} quantity: {}\n".format(key, self.items[key])
        return strOut

class Warehouse:
    def __init__(self, loc, items):
        self.location   = loc               # Location
        self.items      = items             # {itemID:quantity}

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
    def __init__(self, id, loc):
        self.id        = id
        self.location  = loc               # Location
        self.items     = {}
        self.commands  = []                # Command Array

    def pushCommand(self, command):
        self.commands = self.commands + [command]
        # print(self.commands)

    def setLocation(self, loc):
        self.location = loc

    def getCommand(self):
        ret = []
        if self.commands:
            ret = self.commands[0]
        return ret

    def popCommand(self):
        ret = []
        if self.commands:
            ret = self.commands.pop(0)
        return ret

    def loadItems(self, itemID, quantity):
        self.items[itemID] = self.items.get(itemID, 0) + quantity
        finalW = 0  # calulate final weight
        for ID in self.items:    # for every item
            finalW += self.items[ID] * weights[ID]
        if finalW > payload:
            raise Exception('Max drone load weight passed')

    def unloadItems(self, itemID, quantity):
        itemQ = self.items.get(itemID, 0)
        if itemQ < quantity:
            raise Exception('Not enough items carried by the drone')
        else:
            self.items[itemID] -= quantity
            return {itemID: quantity}

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        strOut = "drone {} coords: {},{}\n".format(self.id, self.location.r, self.location.c)
        strOut += "items: "
        for itemID in self.items:
            strOut += str(itemID) + ": " + str(self.items[itemID]) + ", "
        strOut += "\n"
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
    global score, turnsLimitReached
    if not turnsLimitReached:
        print(score)
        score += int(math.ceil( ( (0.0 + T - turn) / T ) * 100) )      # rounded up
    #print('Order completed: turn {}, score = {}'.format(turn, score))

def distance(loc1, loc2):
    return int(math.ceil( sqrt( (loc2.r - loc1.r)**2 + (loc2.c - loc1.c)**2 ) ))

# Google input parsing
file_name   = sys.argv[1]

file  = open('./in/' + file_name + ".in", "r")

defs        = file.readline().split()
ROWS        = int(defs[0])
COLS        = int(defs[1])
D           = int(defs[2])  #DRONES
T           = int(defs[3])
payload     = int(defs[4])
P           = int(file.readline())  # #PRODUCTS
weights     = [ int(x) for x in file.readline().split() ]
W           = int(file.readline())
O           = 0             # #Orders
warehouses  = []
orders      = []

score = 0
turn = 0

warehousesList = []
for w in range(W):
    coords  = [ int(x) for x in file.readline().split() ]
    items   = file.readline().split()
    warehousesList.append({ 'coords': coords, 'items': items})

O = int(file.readline())

print('ROWS:', ROWS, 'COLS:', COLS, 'DRONES:', D, 'T:', T, 'PAYLOAD:', payload, 'PRODUCT TYPES', P, 'WAREHOUSES', W, 'ORDERS', O)

ordersList = []
for o in range(O):
    coords  = [ int(x) for x in file.readline().split() ]
    nitems  = int( file.readline() )
    items   = [ int(x) for x in file.readline().split() ] # items.length == nitems
    ordersList.append({ 'coords': coords, 'items': items})

file.close()

# Google output parsing

file  = open('./out/' + file_name + ".out", "r")

defs = file.readline().split()

COMMANDS_NUM = int(defs[0])
drones = []      #drones array

for d in range(D):
    drone = Drone(d, Location(0, 0))
    drones = drones + [drone]

for c in range(COMMANDS_NUM):
    defs = file.readline().split()
    print(defs)
    droneID = int(defs[0])
    tag = defs[1] # 'L': Load, 'D': Deliver, 'W': Wait
    data = int(defs[2]) # 'L': WharehouseID, 'D': OrderID, 'W': #turns
    if tag is 'L' or tag is 'D' or tag is 'U':
        product = int(defs[3])
        quantity = int(defs[4])
        command = Command(tag, data, product, quantity)
        #print('droneID:', droneID, 'tag:', tag, 'data:', data, 'product:', product, 'quantity:', quantity)
    else:
        # wait case
        command = Command(tag, data, None, None)
    drones[droneID].pushCommand(command)

file.close()

#print drones
# for i, drone in enumerate(drones):
#     print("drone {}:".format(i))
#     print(drone)

# initialize warehouses objects
warehouses = []   #warehouse array
for i, warehouse in enumerate(warehousesList):
    products = {}     #products dictionary
    r = warehouse['coords'][0]
    c = warehouse['coords'][1]
    for n in range(P):
        products[n] = int(warehouse['items'][n])

    w = Warehouse(Location(r, c), products)
    warehouses = warehouses + [w]

#print warehouses
# for i,warehouse in enumerate(warehouses):
#     print("warehouse {}:".format(i))
#     print(warehouse)

# initialize orders objects
orders = []
for i,order in enumerate(ordersList):
    r = order['coords'][0]
    c = order['coords'][1]
    products = {}

    for p in range(len(order['items'])):
        products[order['items'][p]] = products.get(order['items'][p], 0) + 1

    o = Order(i, Location(r,c), products)
    orders = orders + [o]

#print orders
# for i,order in enumerate(orders):
#     print("order {}:".format(i))
#     print(order)

MATRIX_SIZE_LIMIT = max(10000, T)

matrix = [[] for y in range(MATRIX_SIZE_LIMIT)]

def executeCommand(drone):
    command = drone.popCommand()
    print('DRONE{}: {}to{} - {}#prod{} - turn={}'.format(drone.id, command.tag, command.data, command.quantity, command.product, turn))
    if command.tag is 'L':
        w = warehouses[command.data]
        product = command.product
        quantity = command.quantity
        w.getItems(product, quantity)
        drone.loadItems(product, quantity)
    elif command.tag is 'D':
        order = orders[command.data]
        product = command.product
        quantity = command.quantity
        drone.unloadItems(product, quantity)
        order.delivered(product,quantity)
    elif command.tag is 'U':
        w = warehouses[command.data]
        product = command.product
        quantity = command.quantity
        drone.unloadItems(prodct, quantity)
        w.putItems(product, quantity)
    elif command.tag is 'W':
        print("WAITING")
        # do nothing

def setDroneBusy(drone):
    global turnsLimitReached
    command = drone.getCommand()
    if command:
        if command.tag is 'L' or command.tag is 'U':
            to_location = warehouses[command.data].location
            delta = distance(drone.location, to_location) + 1
            drone.setLocation(to_location)
        elif command.tag is 'D':
            to_location = orders[command.data].location
            delta = distance(drone.location, to_location) + 1
            drone.setLocation(to_location)
        elif command.tag is 'W':
            delta = command.data
        if turn + delta > MATRIX_SIZE_LIMIT:
            raise Exception('matrix size limit reached')
        if not turnsLimitReached and turn + delta > T:
            turnsLimitReached = True
        matrix[turn + delta] += [drone.id]

# INIT SIMULATION

for d in drones:
    command = d.getCommand()
    if command:
        if command.tag is 'L':
            to_location = warehouses[command.data].location
            delta = distance(d.location, to_location)
            d.setLocation(to_location)
        elif command.tag is 'W':
            delta = command.data
        if turn + delta > MATRIX_SIZE_LIMIT:
            raise Exception('matrix size limit reached')
        if not turnsLimitReached and turn + delta > T:
            turnsLimitReached = True
        matrix[turn + delta] += [d.id]

for t in range(MATRIX_SIZE_LIMIT):
    turn = t
    for d in matrix[t]:
        executeCommand(drones[d])
        setDroneBusy(drones[d])

print('\n\nFINAL SCORE = {}'.format(score))
if turnsLimitReached:
    print('The score was limited by the number of input turns')
