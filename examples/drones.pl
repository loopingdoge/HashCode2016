%% World

Payload is 500.

coord([0, 0]).
%% ...
coord([N, N]).

drone(drone1).
%% ...
drone(droneN).

warehouse(warehouse1, coord([Warehouse1CoordX, Warehouse1CoordY])).
%% ...
warehouse(warehouseN, coord([WarehouseNCoordX, WarehouseNCoordY])).

product(product1, Product1Weight).
%% ...
product(productN, ProductNWeight).

item(item1, Item1Type).
%% ...
item(itemN, ItemNType).

order(order1, [HowMany1, product1], coord([Order1CoordX, Order1CoordY])).
%% ...
order(orderN, [HowManyN, productN], coord([OrderNCoord, OrderNCoordY])).


%% State

initial_state :-
	at(drone1, coord([0, 0])),
	%% ...
	at(droneN, coord([0, 0])),

	at(warehouse1, coord([Warehouse1X, Warehouse1Y])),
	%% ...
	at(warehouseN, coord([WarehouseNX, WarehouseNY])),

	at(item(item1, ), warehouse1),
	%% ...
	at(itemN, warehouseN).


final_state :-
	%% foreach order
	%%     foreach productType
	%%         at(item(Item, productType), coord([OrderICoordX, OrderICoordY]))
	%%         

%% Actions

%% move(Drone, Coord)
%% move preconditions
%% move adds
%% move deletes

%% load(Item, Drone)
%% load preconditions
%% load adds
%% load deletes

%% unload(Item, Drone)
%% unload preconditions
%% unload adds
%% unload deletes