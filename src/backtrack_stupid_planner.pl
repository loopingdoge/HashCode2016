:- use_module(library(clpfd)).   %% Finite domain constraints

%%
%% STRIPS Planner
%%

equal_set(S1, S2) :- subset(S1, S2), subset(S2, S1).

add_to_set(X, S, S) :- member(X, S), !.
add_to_set(X, S, [X|S]).

remove_from_set(_, [], []).
remove_from_set(E, [E|T], T) :- !.
remove_from_set(E, [H|T], [H|T_new]) :- remove_from_set(E, T, T_new), !.

empty_stack([]).

stack(E, S, [E|S]).

count_drone_turns([], _, Turns) :- Turns is 0.
count_drone_turns([_|T], Drone, Turns) :-
    count_drone_turns(T, Drone, OtherTurns),
    Turns is OtherTurns.
count_drone_turns([load(Drone, _, _, MoveTurns)|T], Drone, Turns) :-
    count_drone_turns(T, Drone, OtherTurns),
    Turns is MoveTurns + OtherTurns.
count_drone_turns([deliver(Drone, _, _, MoveTurns)|T], Drone, Turns) :-
    count_drone_turns(T, Drone, OtherTurns),
    Turns is MoveTurns + OtherTurns.
count_drone_max_turns(Moves, Drone, Turns) :-
    findall(T, count_drone_turns(Moves, Drone, T), TurnsList),
    sort(TurnsList, SortedTurns),
    reverse(SortedTurns, [Turns|_]).
turns_used(Moves, Turns) :-
    findall(T, (drone(Drone), count_drone_max_turns(Moves, Drone, T)), TurnsList),
    sort(TurnsList, SortedTurns),
    reverse(SortedTurns, [Turns|_]).

export_moves(Mov, _) :- empty_stack(Mov).
export_moves(Mov, Stream) :-
    stack(E, Rest, Mov),
    export_moves(Rest, Stream),
    writeln(Stream, E).

%%
% True when a solution is found (the Goal state is a subset of the valued state)
% Print out the solution plan and comes out of recursion
%%
plan(State, Goal, _, Moves, _) :-
    subset(Goal, State),
    %turns_used(Moves, UsedTurns),
    %UsedTurns #=< MaxTurns,
    %% write(State), nl,
    open('out/{{filename}}.cmds', write, Stream),
    export_moves(Moves, Stream),
    close(Stream)
    {{ debug }}.
    %write('Turns: '), write(UsedTurns).

%%
% When the Goal state is a subset of the valued state
% Use a defined action to move through the state-space
%%
plan(State, Goal, Been_list, Moves, MaxTurns) :-
    %% write(State), nl, nl,
    %% write(Been_list), nl,
    % turns_used(Moves, UsedTurns),
    %UsedTurns #=< MaxTurns,
    move(State, Name, Preconditions, Actions),
    conditions_met(Preconditions, State),
    change_state(State, Actions, Child_state),
    not(member_state(Child_state, Been_list)),
    stack(Child_state, Been_list, New_been_list),
    stack(Name, Moves, New_moves),
    plan(Child_state, Goal, New_been_list, New_moves, MaxTurns).

change_state(S, [], S).
change_state(S, [add(P)|T], S_new) :-
    add_to_set(P, S, S2),
    change_state(S2, T, S_new), !.
change_state(S, [del(P)|T], S_new) :-
    remove_from_set(P, S, S2),
    change_state(S2, T, S_new), !.
conditions_met(P, S) :- subset(P, S).

member_state(S, [H|_]) :- equal_set(S, H).
member_state(S, [_|T]) :- member_state(S, T).

reverse_print_stack(S) :- empty_stack(S).
reverse_print_stack(S) :-
    stack(E, Rest, S),
    reverse_print_stack(Rest),
    write(E), nl.

count_occurrences(List, Element, Counter) :-
    not(member(Element, List)),
    Counter is 0.
count_occurrences(List, Element, Counter) :-
    member(Element, List),
    bagof(true, member(Element, List), ReducedList),
    length(ReducedList, Counter).

%%
%% Utility predicates
%%

%%
% returns the load weight of a drone
%%
drone_load([], _, _) :- fail, !.
drone_load([weighs(Drone, X)|_], Drone, Weight) :- Weight is X, !.
drone_load([_|T], Drone, Weight) :- drone_load(T, Drone, Weight).

%%
% returns the drone coordinates
%%
drone_coords([], _, _) :- fail.
drone_coords([at(Drone, coord(X, Y))|_], Drone, Coords) :- !, Coords = coord(X, Y), !.
drone_coords([at(Drone, Warehouse)|_], Drone, Coords) :- warehouse(Warehouse, Coords), !.
drone_coords([at(Drone, Order)|_], Drone, Coords) :- order(Order, _, Coords), !.
drone_coords([_|T], Drone, Coords) :- drone_coords(T, Drone, Coords).

drone_location([], _, _) :- fail.
drone_location([at(Drone, coord(X, Y))|_], Drone, coord(X, Y)) :- !.
drone_location([at(Drone, Warehouse)|_], Drone, Warehouse) :- warehouse(Warehouse, _), !.
drone_location([at(Drone, Order)|_], Drone, Order) :- order(Order, _, _), !.
drone_location([_|T], Drone, Location) :- drone_location(T, Drone, Location).

requested_product_and_order([], _, _, _) :- fail, !.
requested_product_and_order([need(NeedId, Product, Order)|_], Order, Product, NeedId) :- !.
requested_product_and_order([_|T], Order, Product, NeedId) :- requested_product_and_order(T, Order, Product, NeedId).

delivering_product_and_order([], _, _, _, _) :- fail, !.
delivering_product_and_order([delivering(NeedId, Item, Order, Drone)|_], Order, Item, NeedId, Drone) :- !.
delivering_product_and_order([_|T], Order, Item, NeedId, Drone) :- delivering_product_and_order(T, Order, Item, NeedId, Drone).

%%
% True if the order requires another product of that type
%%
need_to_load_more(State, Order, Product) :-
    order(Order, _, _),
    member(need(_, Product, Order), State),
    !.

%%
% return the Euclidean distance between two coordinates
%%
distance(coord(X1, Y1), coord(X2, Y2), Distance) :-
    !,
    coord(X1, Y1),
    coord(X2, Y2),
    Distance is ceil(sqrt(((X1 - X2) * (X1 - X2)) + ((Y1 - Y2) * (Y1 - Y2)))).
distance(Order, Warehouse, Distance) :-
    order(Order, _, OrderCoord),
    warehouse(Warehouse, WarehouseCoord),
    distance(OrderCoord, WarehouseCoord, Distance).
distance(State, Drone, Order, Distance) :-
    drone(Drone),
    order(Order, _, OrderCoords),
    drone_coords(State, Drone, DroneCoords),
    distance(OrderCoords, DroneCoords, Distance).
distance(State, Drone, Warehouse, Distance) :-
    drone(Drone),
    warehouse(Warehouse, WarehouseCoord),
    drone_coords(State, Drone, DroneCoords),
    distance(WarehouseCoord, DroneCoords, Distance).

%%
% returns the nearest warehouse from a order
%%
nearest_warehouse_from_order(State, Order, Product, Warehouse, Item) :-
    findall(
        [Distance, Warehouses, ItemInW],
        (
        	warehouse(Warehouses, _),
            item(ItemInW, Product),
            member(at(ItemInW, Warehouses), State),
            distance(Order, Warehouses, Distance)
        ),
        DistanceList
    ),
    sort(DistanceList, [[_, Warehouse, Item]|_]), !.


%%
% returns the nearest drone from a warehouse
%%
nearest_drone_from_warehouse(State, Warehouse, Product, Drone, OldWeight, NewWeight, Distance) :-
    findall(
        [Dist, Drones, CurrentWeight, UpdatedWeight],
        (
           drone(Drones),
           drone_load(State, Drones, CurrentWeight),
           product(Product, ProductWeight),
           payload(MaxWeight),
           CurrentWeight + ProductWeight #=< MaxWeight,
           UpdatedWeight is CurrentWeight + ProductWeight,
           distance(State, Drones, Warehouse, Dist)
        ),
        DistanceList
    ),
    sort(DistanceList, [[Distance, Drone, OldWeight, NewWeight]|_]), !.

%%
%% Actions
%%

%%
% Action schema rapresentation:
%
% move(
%    State,
%    actionName(list of all the variables used),
%    [preconditions],
%    [effects]
% )
% where preconditions defines the states in which the action can be executed
% and effects defines the result of executing the action.
%%

move(
    State,
    load(Drone, Product, Warehouse, 0),
    [at(Item, Warehouse), need(NeedId, Product, Order)],
    [
        del(at(Item, Warehouse)), del(weighs(Drone, CurrentWeight)), del(need(NeedId, Product, Order)), del(at(Drone, PrevDroneLocation)),
        add(at(Item, Drone)), add(weighs(Drone, NewWeight)), add(delivering(NeedId, Item, Order, Drone)), add(at(Drone, Warehouse))
    ]
) :-
    drone(Drone),
    warehouse(Warehouse, _),
    order(Order, _, _),
    product(Product, _),
    % % member(Product, ProductList),
    item(Item, Product),
    drone_location(State, Drone, PrevDroneLocation),
    drone_load(State, Drone, CurrentWeight),
    payload(MaxWeight),
    product(Product, ProductWeight),
    CurrentWeight + ProductWeight #=< MaxWeight,
    NewWeight is CurrentWeight + ProductWeight.
    %requested_product_and_order(State, Order, Product, NeedId).

move(
    State,
    deliver(Drone, Product, Order, 0),
    [at(Item, Drone), delivering(NeedId, Item, Order, Drone)],
    [
        del(at(Item, Drone)), del(weighs(Drone, CurrentWeight)), del(delivering(NeedId, Item, Order, Drone)), del(at(Drone, PrevDroneLocation)),
        add(at(NeedId, Product, Order)), add(weighs(Drone, NewWeight)), add(at(Drone, Order))
    ]
) :-
    % order(Order, _, _),
    % %member(Product, ProductList),
    % product(Product, ProductWeight),
    % item(Item, Product),
    % drone(Drone),
    % drone_location(State, Drone, PrevDroneLocation),
    % drone_load(State, Drone, CurrentWeight),
    % NewWeight is CurrentWeight - ProductWeight.
    %delivering_product_and_order(State, Order, Item, NeedId, Drone).

    item(Item, Product),
    drone_location(State, Drone, PrevDroneLocation),
    drone_load(State, Drone, CurrentWeight),
    product(Product, ProductWeight),
    NewWeight is CurrentWeight - ProductWeight.

    %%
%% World Facts
%%

payload({{ payload }}).

coord(X, Y) :-
  X<{{ rows }},
  X>=0,
  Y<{{ cols }},
  Y>=0.

{{ world_facts }}

%%
%% Main
%%

go(S, G) :- plan(S, G, [S], [], {{ max_turns }}).

test :- go(
    [{{ initial_state }}],
    [{{ final_state }}]
).

/** <examples> Your example queries go here, e.g.
?- test.
? - distance([at(drone1, coord(0, 0))], drone1, warehouse2, D).
? - distance(order1, warehouse1, D).
?- drone_load([at(drone1, coord(0, 0)), at(item1, warehouse1), weighs(drone1, 30), at(item2, warehouse2)], drone1, W).
*/
