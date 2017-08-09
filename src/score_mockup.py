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


class Warehouse:
    def __init__(self, loc, items):
        self.loc    = loc               # Location
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


class Drone:
    items    = {}
    commands = []                       # Command Array
    def __init__(self, loc):
        self.location   = loc               # Location

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


class Command:
    def __init__(self, tag, data, items):
        self.tag    = tag               # 'L': Load, U': Unload, 'D': Deliver, 'W': Wait
        self.data   = data              # 'L/U': WharehouseID, 'D': OrderID, 'W': #turns 
        self.items  = items

def orderCompleted():
    # increments score
    global score
    score += int( ( ( T - turn ) / T ) * 100 ) + ( T - turn ) % T > 0       # rounded up
    print('Order completed: turn {}, score = {}'.format(turn, score))


MAX_W = 15
T = 100         # #Turns
W = 0           # #Wharehouses
I = 0           # #Items
D = 0           # #Drones
O = 0           # #Orders
Q = 0           # #Commands

score = 0
turn = 0


item = {'1': 5, '2': 1}       # weight Array #I
drone = []      # Drone Array #D
warehouse = []  # Wharehouse Array #W
order = []      # Order Array #O
turns = [[],[],[],[],[],[]]    # Turn's command Matrix #T, for every turn there's an array of commands to be executed in that turn

# TEST DRONE
# d = Drone(Location(3,5))
# d.loadItems('1', 2)
# d.loadItems('2', 5)
# d.unloadItems('3',3)
# print(d.commands)
# d.pushCommand('E')
# d.pushCommand('Ew')
# d.pushCommand('Ewe')
# d.pushCommand('Ere')
# print(d.popCommand())
# /TEST DRONE


# TEST WAREHOUSE
# w = Warehouse(Location(1,4), {'1': 2, '2': 1})
# w.putItems('1', 2)
# w.putItems('3', 2)
# print(w.getItems('1', 2))
# w.putItems('4', 1)
# print(w.getItems('1', 2))
# w.putItems('4', 1)
# /TEST WAREHOUSE

# TEST ORDER
# x = Order(Location(1,1), {'1': 2, '2': 1, '3':3})
# x.delivered('1', 2)
# x.delivered('2', 1)
# x.delivered('3', 2)
# x.delivered('3', 1)
# /TEST ORDER


# for t in turns:
#     print t
    


