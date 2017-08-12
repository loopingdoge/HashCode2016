# python src/generate_input.py 50 50 10 500 250 5 3 3
from sys import argv, exit
from random import randint

def coord_line(rows, cols):
    return "{} {}\n".format(randint(0, rows - 1), randint(0, cols - 1))

def first_line(rows, cols, drones, turns, payload):
    return "{} {} {} {} {}\n".format(rows, cols, drones, turns, payload)

def products_line(products_number):
    return "{}\n".format(products_number)

def weights_line(products_number, max_payload):
    weights = [randint(10, max_payload) for _ in range(products_number)]
    return " ".join(str(x) for x in weights) + "\n"

def warehouses_lines(warehouses_number, products_number, rows, cols, max_stocks):
    res = "{}\n".format(warehouses_number)
    items_per_product = []
    for j in range(products_number):
        items_per_product.append(randint(1, max_stocks))
    for _ in range(warehouses_number):
        res += coord_line(rows, cols)
        for j in range(products_number):
            res += "{}".format(items_per_product[j]) + (" " if j != products_number - 1 else "\n")
    return res, items_per_product

def order_lines(rows, cols, products_number, items_per_product, max_items_per_order):
    res = coord_line(rows, cols)
    items_n = randint(1, max_items_per_order)
    res += "{}\n".format(items_n)
    while True:
        if sum(items_per_product) == 0:
            main()
        product = randint(0, products_number - 1)
        if items_per_product[product] > 0:
            items_per_product[product] = items_per_product[product] - 1
            break
    order_items = [randint(0, products_number - 1) for _ in range(items_n)]
    res += " ".join(str(x) for x in order_items) + "\n"
    return res, items_per_product

def orders_lines(orders_number, products_number, rows, cols, items_per_product, max_items_per_order):
    res = "{}\n".format(orders_number)
    updated_items = items_per_product
    for _ in range(orders_number):
        order_line, updated_items = order_lines(rows, cols, products_number, updated_items, max_items_per_order)
        res += order_line
    return res

def main():
    max_stocks = 1
    max_items_per_order = 1
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
    while config["warehouses"] * max_stocks < config["orders"] * max_items_per_order:
        max_stocks = max_stocks + 1
    res = first_line(
        config["rows"],
        config["cols"],
        config["drones"],
        config["turns"],
        config["payload"]
    )
    res += products_line(config["products"])
    res += weights_line(config["products"], config["payload"])
    warehouse_line, items_per_product = warehouses_lines(
        config["warehouses"],
        config["products"],
        config["rows"],
        config["cols"],
        max_stocks
    )
    res += warehouse_line
    res += orders_lines(
        config["orders"],
        config["products"],
        config["rows"],
        config["cols"],
        items_per_product,
        max_items_per_order
    )
    with open("./in/generated.in", "w") as file:
        file.write(res)
    exit(0)

if __name__ == "__main__":
    main()
