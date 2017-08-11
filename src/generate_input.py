# python src/generate_input.py 50 50 10 500 250 5 3 3
from sys import argv
from random import randint

MAX_STOCKS = 5
MAX_ITEMS_PER_ORDER = 2

def coord_line(rows, cols):
    return "{} {}\n".format(randint(0, rows), randint(0, cols))

def first_line(rows, cols, drones, turns, payload):
    return "{} {} {} {} {}\n".format(rows, cols, drones, turns, payload)

def products_line(products_number):
    return "{}\n".format(products_number)

def weights_line(products_number, max_payload):
    weights = [randint(10, max_payload) for _ in range(products_number)]
    return " ".join(str(x) for x in weights) + "\n"

def warehouses_lines(warehouses_number, products_number, rows, cols):
    res = "{}\n".format(warehouses_number)
    for _ in range(warehouses_number):
        res += coord_line(rows, cols)
        for j in range(products_number):
            res += "{}".format(randint(1, MAX_STOCKS)) + (" " if j != products_number - 1 else "\n")
    return res

def order_lines(rows, cols, products_number):
    res = coord_line(rows, cols)
    items_n = randint(1, MAX_ITEMS_PER_ORDER)
    res += "{}\n".format(items_n)
    order_items = [randint(0, products_number - 1) for _ in range(items_n)]
    res += " ".join(str(x) for x in order_items) + "\n"
    return res


def orders_lines(orders_number, products_number, rows, cols):
    res = "{}\n".format(orders_number)
    for _ in range(orders_number):
        res += order_lines(rows, cols, products_number)
    return res

def main():
    config = {
        "rows": int(argv[1]),
        "cols": int(argv[2]),
        "drones": int(argv[3]),
        "turns": int(argv[4]),
        "payload": int(argv[5]),
        "products": int(argv[6]),
        "warehouses": int(argv[7]),
        "orders": int(argv[8])
    }
    res = first_line(
        config["rows"],
        config["cols"],
        config["drones"],
        config["turns"],
        config["payload"]
    )
    res += products_line(config["products"])
    res += weights_line(config["products"], config["payload"])
    res += warehouses_lines(
        config["warehouses"],
        config["products"],
        config["rows"],
        config["cols"]
    )
    res += orders_lines(config["orders"], config["products"], config["rows"], config["cols"])
    with open("./in/generated.in", "w") as file:
        file.write(res)

if __name__ == "__main__":
    main()
