% Constraint Logic Programming
:- use_module(library(dif)).		% Sound inequality
:- use_module(library(clpfd)).		% Finite domain constraints

% Your program goes here

equal_set(S1, S2) :- 
    subset(S1, S2), subset(S2, S1).

add_to_set(X, S, S) :- member(X, S), !.
add_to_set(X, S, [X|S]).

remove_from_set(_, [], []).
remove_from_set(E, [E|T], T) :- !.
remove_from_set(E, [H|T], [H|T_new]) :-
    remove_from_set(E, T, T_new), !.

empty_stack([]).

stack(E, S, [E|S]).

plan(State, Goal, _, Moves) :- 	
    subset(Goal, State), 
	write('moves are'), nl,
	reverse_print_stack(Moves).

plan(State, Goal, Been_list, Moves) :-
    % write(State), nl,
    % write(Been_list), nl,
	move(State, Name, Preconditions, Actions),
	conditions_met(Preconditions, State),
	change_state(State, Actions, Child_state),
	not(member_state(Child_state, Been_list)),
	stack(Child_state, Been_list, New_been_list),
	stack(Name, Moves, New_moves),
	plan(Child_state, Goal, New_been_list, New_moves),!.

change_state(S, [], S).
change_state(S, [add(P)|T], S_new) :-	change_state(S, T, S2),
					add_to_set(P, S2, S_new), !.
change_state(S, [del(P)|T], S_new) :-	change_state(S, T, S2),
					remove_from_set(P, S2, S_new), !.
conditions_met(P, S) :- subset(P, S).


member_state(S, [H|_]) :- 	equal_set(S, H).
member_state(S, [_|T]) :- 	member_state(S, T).

reverse_print_stack(S) :- 	empty_stack(S).
reverse_print_stack(S) :- 	stack(E, Rest, S), 
				reverse_print_stack(Rest),
		 		write(E), nl.

count_occurrences(List, Element, Counter) :-
    not(member(Element, List)),
    Counter is 0.
count_occurrences(List, Element, Counter) :-
    member(Element, List),
    bagof(true, member(Element, List), ReducedList),
    length(ReducedList, Counter).

/* sample moves */

payload(500).

coord(X, Y) :-
  X<100,
  X>=0,
  Y<100,
  Y>=0.

drone(drone1).
drone(drone2).
drone(drone3).

warehouse(warehouse1, coord(0, 0)).
warehouse(warehouse2, coord(5, 5)).

product(product1, 100).
product(product2, 5).
product(product3, 450).

item(item1, product1).
item(item2, product1).
item(item3, product1).
item(item4, product1).
item(item5, product1).
item(item6, product2).
item(item7, product2).
item(item8, product2).
item(item9, product2).
item(item10, product2).
item(item11, product2).
item(item12, product2).
item(item13, product2).
item(item14, product2).
item(item15, product2).
item(item16, product2).
item(item17, product3).
item(item18, product3).

order(order1, [product3, product1], coord(1, 1)).
order(order2, [product1], coord(3, 3)).
order(order3, [product3], coord(5, 6)).

drone_load([], _, _) :- fail.
drone_load([weighs(Drone, X)|_], Drone, Weight) :- Weight is X.
drone_load([_|T], Drone, Weight) :- drone_load(T, Drone, Weight).

drone_coords([], _, _) :- fail.
drone_coords([at(Drone, Coords)|_], Drone, Coords).
drone_coords([_|T], Drone, Coords) :- drone_coords(T, Drone, Coords).

need_to_load_more(State, Order, Product) :-
    order(Order, OrderList, _),
    count_occurrences(OrderList, Product, CountTotalProducts),
    count_occurrences(State, delivering(Product, Order), CountDelivering),
    count_occurrences(State, at(Product, Order), CountAlreadyDelivered),
    CountDelivering #< CountTotalProducts - CountAlreadyDelivered,
    !.

distance(coord(X1, Y1), coord(X2, Y2), Distance) :-
    coord(X1, Y1),
    coord(X2, Y2),
    Distance is ceil(sqrt(((X1 - X2) * (X1 - X2)) + ((Y1 - Y2) * (Y1 - Y2)))).
distance(Order, Warehouse, Distance) :-
    order(Order, _, OrderCoord),
    warehouse(Warehouse, WarehouseCoord),
    distance(OrderCoord, WarehouseCoord, Distance).
distance(State, Drone, Warehouse, Distance) :-
    drone(Drone),
    warehouse(Warehouse, WarehouseCoord),
    drone_coords(State, Drone, DroneCoords),
    distance(WarehouseCoord, DroneCoords, Distance).

nearest_warehouse_from_order(State, Order, Product, Warehouse) :-
    findall(
        [Distance, Warehouses],
        (warehouse(Warehouses, _), item(Item, Product), member(at(Item, Warehouses), State), distance(Order, Warehouses, Distance)),
        DistanceList
    ),
    sort(DistanceList, [[_, Warehouse]|_]).

nearest_drone_from_warehouse(State, Warehouse, Product, Drone, OldWeight, NewWeight) :-
    findall(
        [Distance, Drones, CurrentWeight, UpdatedWeight],
        (
        	drone(Drones),
            drone_load(State, Drone, CurrentWeight),
    		product(Product, ProductWeight),
    		payload(MaxWeight),
    		CurrentWeight + ProductWeight #< MaxWeight,
    		UpdatedWeight is CurrentWeight + ProductWeight,
            distance(State, Drones, Warehouse, Distance)
        ),
        DistanceList
    ),
    write(DistanceList), nl,
    sort(DistanceList, [[_, Drone, OldWeight, NewWeight]|_]).

move(
    State,
	load(Drone, Product, Warehouse),
    [at(Item, Warehouse)],
	[
    	del(at(Item, Warehouse)), del(weighs(Drone, CurrentWeight)),
      	add(at(Item, Drone)), add(weighs(Drone, NewWeight)), add(delivering(Product, Order)), add(at(Drone, Warehouse))
    ]
) :-
    item(Item, Product),								% 
    order(Order, OrderList, _OrderCoord),
    member(Product, OrderList),
    need_to_load_more(State, Order, Product),
    nearest_warehouse_from_order(State, Order, Product, Warehouse),
    drone(Drone),
    drone_load(State, Drone, CurrentWeight),
    product(Product, ProductWeight),
    payload(MaxWeight),
    CurrentWeight + ProductWeight #< MaxWeight,
    NewWeight is CurrentWeight + ProductWeight.
    % nearest_drone_from_warehouse(State, Warehouse, Product, Drone, CurrentWeight, NewWeight).

move(
    State,
    deliver(Drone, Product, Order),
    [at(Item, Drone)],
	[
    	del(at(Item, Drone)), del(weighs(Drone, CurrentWeight)), del(delivering(Product, Order)),
      	add(at(Product, Order)), add(weighs(Drone, NewWeight)), add(at(Drone, Order))
    ]
) :-
    drone(Drone),
    order(Order, _, _),
    item(Item, Product),
    drone_load(State, Drone, CurrentWeight),
    product(Product, ProductWeight),
    NewWeight is CurrentWeight - ProductWeight.

go(S, G) :- plan(S, G, [S], []).

test :- go(
	[
    	at(drone1, coord(0, 0)),
      	at(drone2, coord(0, 0)),
      	at(drone3, coord(0, 0)),
      
    	weighs(drone1, 0),
		weighs(drone2, 0),
      	weighs(drone3, 0),
      
		at(item1, warehouse1),
		at(item2, warehouse1),
		at(item3, warehouse1),
		at(item4, warehouse1),
		at(item5, warehouse1),
      	at(item6, warehouse1),
      	at(item7, warehouse2),
      	at(item8, warehouse2),
      	at(item9, warehouse2),
      	at(item10, warehouse2),
      	at(item11, warehouse2),
      	at(item12, warehouse2),
      	at(item13, warehouse2),
      	at(item14, warehouse2),
      	at(item15, warehouse2),
      	at(item16, warehouse2),
      	at(item17, warehouse2),
      	at(item18, warehouse2)
    ],
 	[
    	at(product1, order1),
		at(product3, order1),
		at(product1, order2),
		at(product3, order3)
    ]
).

/** <examples> Your example queries go here, e.g.
?- test.
? - distance([at(drone1, coord(0, 0))], drone1, warehouse2, D).
? - distance(order1, warehouse1, D).
?- drone_load([at(drone1, coord(0, 0)), at(item1, warehouse1), weighs(drone1, 30), at(item2, warehouse2)], drone1, W).
*/
