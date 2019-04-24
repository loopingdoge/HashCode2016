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
count_drone_max_turns(Moves, Drone, TurnsMinusOne) :-
    findall(T, count_drone_turns(Moves, Drone, T), TurnsList),
    sort(TurnsList, SortedTurns),
    reverse(SortedTurns, [Turns|_]),
    TurnsMinusOne is Turns - 1.
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
    % turns_used(Moves, UsedTurns),
    % UsedTurns #=< MaxTurns,
    % write(State), nl,
    open('out/generated.cmds', write, Stream),
    export_moves(Moves, Stream),
    close(Stream)
    , reverse_print_stack(Moves).
    % write('Turns: '), write(UsedTurns).

%%
% When the Goal state is a subset of the valued state
% Use a defined action to move through the state-space
%%
plan(State, Goal, _, Moves, MaxTurns) :-
    % turns_used(Moves, UsedTurns),
    % UsedTurns #=< MaxTurns,
    move(State, Name, Preconditions, Actions),
    conditions_met(Preconditions, State),
    write(Name), nl,
    change_state(State, Actions, Child_state),
    % not(member_state(Child_state, Been_list)),
    % stack(Child_state, Been_list, New_been_list),
    stack(Name, Moves, New_moves),
    plan(Child_state, Goal, _, New_moves, MaxTurns).

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

%%
% Optimization for the unification of the "load" action.
% This will select an Order and a Product which are actually needed.
%%
requested_product_and_order([], _, _, _) :- fail, !.
requested_product_and_order([need(NeedId, Product, Order)|_], Order, Product, NeedId) :- !.
requested_product_and_order([_|T], Order, Product, NeedId) :- requested_product_and_order(T, Order, Product, NeedId).

%%
% Optimization for the unification of the "deliver" action.
% This will select an Order and a Product which are actually needed.
%%
delivering_product_and_order([], _, _, _, _) :- fail, !.
delivering_product_and_order([delivering(NeedId, Item, Order, Drone)|_], Order, Item, NeedId, Drone) :- !.
delivering_product_and_order([_|T], Order, Item, NeedId, Drone) :- delivering_product_and_order(T, Order, Item, NeedId, Drone).

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
    load(Drone, Product, Warehouse, TurnsConsumed),
    [at(Item, Warehouse), at(Drone, PrevDroneLocation), need(NeedId, Product, Order)],
    [
        del(at(Item, Warehouse)), del(weighs(Drone, CurrentWeight)), del(need(NeedId, Product, Order)), del(at(Drone, PrevDroneLocation)),
        add(at(Item, Drone)), add(weighs(Drone, NewWeight)), add(delivering(NeedId, Item, Order, Drone)), add(at(Drone, Warehouse))
    ]
) :-
    requested_product_and_order(State, Order, Product, NeedId),
    order(Order, ProductList, _),
    member(Product, ProductList),
    product(Product, _),
    nearest_warehouse_from_order(State, Order, Product, Warehouse, Item),
    nearest_drone_from_warehouse(State, Warehouse, Product, Drone, CurrentWeight, NewWeight, Distance),
    warehouse(Warehouse, _),
    item(Item, Product),
    drone(Drone),
    TurnsConsumed is Distance + 1.

move(
    State,
    deliver(Drone, Product, Order, TurnsConsumed),
    [at(Item, Drone), at(Drone, PrevDroneLocation), delivering(NeedId, Item, Order, Drone)],
    [
        del(at(Item, Drone)), del(weighs(Drone, CurrentWeight)), del(delivering(NeedId, Item, Order, Drone)), del(at(Drone, PrevDroneLocation)),
        add(at(NeedId, Product, Order)), add(weighs(Drone, NewWeight)), add(at(Drone, Order))
    ]
) :-
    delivering_product_and_order(State, Order, Item, NeedId, Drone),
    order(Order, ProductList, _),
    member(Product, ProductList),
    item(Item, Product),
    drone(Drone),
    drone_load(State, Drone, CurrentWeight),
    product(Product, ProductWeight),
    NewWeight is CurrentWeight - ProductWeight,
    distance(State, Drone, Order, Distance),
    TurnsConsumed is Distance + 1.

%%
%% World Facts
%%

payload(250).

coord(X, Y) :-
  X<50,
  X>=0,
  Y<50,
  Y>=0.

drone(drone0).
drone(drone1).
drone(drone2).
drone(drone3).
drone(drone4).
warehouse(warehouse0, coord(27, 33)).
warehouse(warehouse1, coord(49, 44)).
warehouse(warehouse2, coord(28, 45)).
warehouse(warehouse3, coord(2, 12)).
warehouse(warehouse4, coord(32, 15)).
product(product0, 221).
product(product1, 237).
product(product2, 100).
product(product3, 250).
product(product4, 126).
item(item0, product0).
item(item1, product0).
item(item2, product0).
item(item3, product0).
item(item4, product0).
item(item5, product0).
item(item6, product0).
item(item7, product0).
item(item8, product0).
item(item9, product1).
item(item10, product1).
item(item11, product1).
item(item12, product1).
item(item13, product1).
item(item14, product1).
item(item15, product1).
item(item16, product1).
item(item17, product1).
item(item18, product1).
item(item19, product2).
item(item20, product2).
item(item21, product2).
item(item22, product2).
item(item23, product2).
item(item24, product2).
item(item25, product2).
item(item26, product2).
item(item27, product2).
item(item28, product2).
item(item29, product3).
item(item30, product3).
item(item31, product3).
item(item32, product3).
item(item33, product3).
item(item34, product4).
item(item35, product4).
item(item36, product4).
item(item37, product4).
item(item38, product4).
item(item39, product4).
item(item40, product4).
item(item41, product4).
item(item42, product4).
item(item43, product4).
item(item44, product4).
item(item45, product4).
item(item46, product4).
item(item47, product4).
item(item48, product0).
item(item49, product0).
item(item50, product0).
item(item51, product0).
item(item52, product0).
item(item53, product0).
item(item54, product0).
item(item55, product0).
item(item56, product0).
item(item57, product0).
item(item58, product0).
item(item59, product0).
item(item60, product0).
item(item61, product0).
item(item62, product0).
item(item63, product0).
item(item64, product1).
item(item65, product1).
item(item66, product1).
item(item67, product1).
item(item68, product1).
item(item69, product1).
item(item70, product1).
item(item71, product1).
item(item72, product1).
item(item73, product1).
item(item74, product1).
item(item75, product1).
item(item76, product2).
item(item77, product2).
item(item78, product3).
item(item79, product3).
item(item80, product3).
item(item81, product3).
item(item82, product3).
item(item83, product3).
item(item84, product3).
item(item85, product3).
item(item86, product3).
item(item87, product3).
item(item88, product3).
item(item89, product3).
item(item90, product3).
item(item91, product3).
item(item92, product3).
item(item93, product3).
item(item94, product4).
item(item95, product4).
item(item96, product4).
item(item97, product4).
item(item98, product4).
item(item99, product4).
item(item100, product4).
item(item101, product4).
item(item102, product4).
item(item103, product4).
item(item104, product4).
item(item105, product4).
item(item106, product4).
item(item107, product4).
item(item108, product4).
item(item109, product4).
item(item110, product4).
item(item111, product4).
item(item112, product0).
item(item113, product0).
item(item114, product0).
item(item115, product0).
item(item116, product0).
item(item117, product0).
item(item118, product0).
item(item119, product0).
item(item120, product1).
item(item121, product1).
item(item122, product1).
item(item123, product1).
item(item124, product1).
item(item125, product1).
item(item126, product1).
item(item127, product1).
item(item128, product1).
item(item129, product1).
item(item130, product1).
item(item131, product1).
item(item132, product1).
item(item133, product1).
item(item134, product1).
item(item135, product1).
item(item136, product1).
item(item137, product1).
item(item138, product2).
item(item139, product2).
item(item140, product2).
item(item141, product3).
item(item142, product3).
item(item143, product3).
item(item144, product3).
item(item145, product3).
item(item146, product3).
item(item147, product3).
item(item148, product3).
item(item149, product3).
item(item150, product3).
item(item151, product3).
item(item152, product3).
item(item153, product3).
item(item154, product3).
item(item155, product3).
item(item156, product3).
item(item157, product3).
item(item158, product3).
item(item159, product4).
item(item160, product4).
item(item161, product4).
item(item162, product4).
item(item163, product4).
item(item164, product4).
item(item165, product4).
item(item166, product4).
item(item167, product4).
item(item168, product4).
item(item169, product0).
item(item170, product0).
item(item171, product0).
item(item172, product0).
item(item173, product0).
item(item174, product0).
item(item175, product0).
item(item176, product0).
item(item177, product0).
item(item178, product0).
item(item179, product0).
item(item180, product0).
item(item181, product0).
item(item182, product0).
item(item183, product0).
item(item184, product0).
item(item185, product1).
item(item186, product1).
item(item187, product1).
item(item188, product1).
item(item189, product1).
item(item190, product1).
item(item191, product1).
item(item192, product1).
item(item193, product1).
item(item194, product1).
item(item195, product1).
item(item196, product1).
item(item197, product1).
item(item198, product2).
item(item199, product2).
item(item200, product2).
item(item201, product2).
item(item202, product2).
item(item203, product2).
item(item204, product2).
item(item205, product2).
item(item206, product2).
item(item207, product2).
item(item208, product2).
item(item209, product3).
item(item210, product3).
item(item211, product3).
item(item212, product3).
item(item213, product3).
item(item214, product3).
item(item215, product3).
item(item216, product3).
item(item217, product3).
item(item218, product3).
item(item219, product3).
item(item220, product4).
item(item221, product4).
item(item222, product4).
item(item223, product4).
item(item224, product4).
item(item225, product4).
item(item226, product4).
item(item227, product4).
item(item228, product4).
item(item229, product4).
item(item230, product4).
item(item231, product4).
item(item232, product4).
item(item233, product4).
item(item234, product4).
item(item235, product0).
item(item236, product0).
item(item237, product0).
item(item238, product0).
item(item239, product0).
item(item240, product0).
item(item241, product0).
item(item242, product1).
item(item243, product1).
item(item244, product1).
item(item245, product1).
item(item246, product1).
item(item247, product1).
item(item248, product1).
item(item249, product1).
item(item250, product1).
item(item251, product2).
item(item252, product2).
item(item253, product2).
item(item254, product2).
item(item255, product3).
item(item256, product3).
item(item257, product3).
item(item258, product3).
item(item259, product3).
item(item260, product3).
item(item261, product3).
item(item262, product3).
item(item263, product3).
item(item264, product4).
item(item265, product4).
order(order0, [product1,product2], coord(34, 26)).
order(order1, [product0,product1], coord(18, 47)).
order(order2, [product0], coord(12, 0)).
order(order3, [product4,product0,product0], coord(2, 38)).
order(order4, [product1,product0,product1], coord(31, 22)).
order(order5, [product1], coord(33, 28)).
order(order6, [product3,product3], coord(41, 24)).
order(order7, [product3,product3,product1], coord(20, 34)).
order(order8, [product1,product3], coord(19, 13)).
order(order9, [product0], coord(6, 14)).
order(order10, [product4,product2,product4], coord(17, 30)).
order(order11, [product4,product1], coord(13, 36)).
order(order12, [product0], coord(32, 16)).
order(order13, [product4,product0], coord(22, 44)).
order(order14, [product3,product3], coord(34, 28)).
order(order15, [product3], coord(6, 30)).
order(order16, [product4,product0,product3], coord(2, 28)).
order(order17, [product4], coord(26, 3)).
order(order18, [product4,product0,product4], coord(45, 37)).
order(order19, [product3,product3,product0], coord(17, 39)).
order(order20, [product4,product2], coord(35, 34)).
order(order21, [product2,product1], coord(33, 25)).
order(order22, [product4,product3], coord(26, 10)).
order(order23, [product4,product2], coord(41, 15)).
order(order24, [product3], coord(10, 34)).
order(order25, [product1,product0], coord(11, 42)).
order(order26, [product1,product3], coord(10, 10)).
order(order27, [product0], coord(48, 18)).
order(order28, [product4,product3,product2], coord(20, 37)).
order(order29, [product4,product4,product3], coord(1, 19)).
needsNum(61).

%%
%% Main
%%

go(S, G) :- plan(S, G, [S], [], 50000).

test :- go(
    [at(drone0, coord(0, 0)),
weighs(drone0, 0),
at(drone1, coord(0, 0)),
weighs(drone1, 0),
at(drone2, coord(0, 0)),
weighs(drone2, 0),
at(drone3, coord(0, 0)),
weighs(drone3, 0),
at(drone4, coord(0, 0)),
weighs(drone4, 0),
at(item0, warehouse0),
at(item1, warehouse0),
at(item2, warehouse0),
at(item3, warehouse0),
at(item4, warehouse0),
at(item5, warehouse0),
at(item6, warehouse0),
at(item7, warehouse0),
at(item8, warehouse0),
at(item9, warehouse0),
at(item10, warehouse0),
at(item11, warehouse0),
at(item12, warehouse0),
at(item13, warehouse0),
at(item14, warehouse0),
at(item15, warehouse0),
at(item16, warehouse0),
at(item17, warehouse0),
at(item18, warehouse0),
at(item19, warehouse0),
at(item20, warehouse0),
at(item21, warehouse0),
at(item22, warehouse0),
at(item23, warehouse0),
at(item24, warehouse0),
at(item25, warehouse0),
at(item26, warehouse0),
at(item27, warehouse0),
at(item28, warehouse0),
at(item29, warehouse0),
at(item30, warehouse0),
at(item31, warehouse0),
at(item32, warehouse0),
at(item33, warehouse0),
at(item34, warehouse0),
at(item35, warehouse0),
at(item36, warehouse0),
at(item37, warehouse0),
at(item38, warehouse0),
at(item39, warehouse0),
at(item40, warehouse0),
at(item41, warehouse0),
at(item42, warehouse0),
at(item43, warehouse0),
at(item44, warehouse0),
at(item45, warehouse0),
at(item46, warehouse0),
at(item47, warehouse0),
at(item48, warehouse1),
at(item49, warehouse1),
at(item50, warehouse1),
at(item51, warehouse1),
at(item52, warehouse1),
at(item53, warehouse1),
at(item54, warehouse1),
at(item55, warehouse1),
at(item56, warehouse1),
at(item57, warehouse1),
at(item58, warehouse1),
at(item59, warehouse1),
at(item60, warehouse1),
at(item61, warehouse1),
at(item62, warehouse1),
at(item63, warehouse1),
at(item64, warehouse1),
at(item65, warehouse1),
at(item66, warehouse1),
at(item67, warehouse1),
at(item68, warehouse1),
at(item69, warehouse1),
at(item70, warehouse1),
at(item71, warehouse1),
at(item72, warehouse1),
at(item73, warehouse1),
at(item74, warehouse1),
at(item75, warehouse1),
at(item76, warehouse1),
at(item77, warehouse1),
at(item78, warehouse1),
at(item79, warehouse1),
at(item80, warehouse1),
at(item81, warehouse1),
at(item82, warehouse1),
at(item83, warehouse1),
at(item84, warehouse1),
at(item85, warehouse1),
at(item86, warehouse1),
at(item87, warehouse1),
at(item88, warehouse1),
at(item89, warehouse1),
at(item90, warehouse1),
at(item91, warehouse1),
at(item92, warehouse1),
at(item93, warehouse1),
at(item94, warehouse1),
at(item95, warehouse1),
at(item96, warehouse1),
at(item97, warehouse1),
at(item98, warehouse1),
at(item99, warehouse1),
at(item100, warehouse1),
at(item101, warehouse1),
at(item102, warehouse1),
at(item103, warehouse1),
at(item104, warehouse1),
at(item105, warehouse1),
at(item106, warehouse1),
at(item107, warehouse1),
at(item108, warehouse1),
at(item109, warehouse1),
at(item110, warehouse1),
at(item111, warehouse1),
at(item112, warehouse2),
at(item113, warehouse2),
at(item114, warehouse2),
at(item115, warehouse2),
at(item116, warehouse2),
at(item117, warehouse2),
at(item118, warehouse2),
at(item119, warehouse2),
at(item120, warehouse2),
at(item121, warehouse2),
at(item122, warehouse2),
at(item123, warehouse2),
at(item124, warehouse2),
at(item125, warehouse2),
at(item126, warehouse2),
at(item127, warehouse2),
at(item128, warehouse2),
at(item129, warehouse2),
at(item130, warehouse2),
at(item131, warehouse2),
at(item132, warehouse2),
at(item133, warehouse2),
at(item134, warehouse2),
at(item135, warehouse2),
at(item136, warehouse2),
at(item137, warehouse2),
at(item138, warehouse2),
at(item139, warehouse2),
at(item140, warehouse2),
at(item141, warehouse2),
at(item142, warehouse2),
at(item143, warehouse2),
at(item144, warehouse2),
at(item145, warehouse2),
at(item146, warehouse2),
at(item147, warehouse2),
at(item148, warehouse2),
at(item149, warehouse2),
at(item150, warehouse2),
at(item151, warehouse2),
at(item152, warehouse2),
at(item153, warehouse2),
at(item154, warehouse2),
at(item155, warehouse2),
at(item156, warehouse2),
at(item157, warehouse2),
at(item158, warehouse2),
at(item159, warehouse2),
at(item160, warehouse2),
at(item161, warehouse2),
at(item162, warehouse2),
at(item163, warehouse2),
at(item164, warehouse2),
at(item165, warehouse2),
at(item166, warehouse2),
at(item167, warehouse2),
at(item168, warehouse2),
at(item169, warehouse3),
at(item170, warehouse3),
at(item171, warehouse3),
at(item172, warehouse3),
at(item173, warehouse3),
at(item174, warehouse3),
at(item175, warehouse3),
at(item176, warehouse3),
at(item177, warehouse3),
at(item178, warehouse3),
at(item179, warehouse3),
at(item180, warehouse3),
at(item181, warehouse3),
at(item182, warehouse3),
at(item183, warehouse3),
at(item184, warehouse3),
at(item185, warehouse3),
at(item186, warehouse3),
at(item187, warehouse3),
at(item188, warehouse3),
at(item189, warehouse3),
at(item190, warehouse3),
at(item191, warehouse3),
at(item192, warehouse3),
at(item193, warehouse3),
at(item194, warehouse3),
at(item195, warehouse3),
at(item196, warehouse3),
at(item197, warehouse3),
at(item198, warehouse3),
at(item199, warehouse3),
at(item200, warehouse3),
at(item201, warehouse3),
at(item202, warehouse3),
at(item203, warehouse3),
at(item204, warehouse3),
at(item205, warehouse3),
at(item206, warehouse3),
at(item207, warehouse3),
at(item208, warehouse3),
at(item209, warehouse3),
at(item210, warehouse3),
at(item211, warehouse3),
at(item212, warehouse3),
at(item213, warehouse3),
at(item214, warehouse3),
at(item215, warehouse3),
at(item216, warehouse3),
at(item217, warehouse3),
at(item218, warehouse3),
at(item219, warehouse3),
at(item220, warehouse3),
at(item221, warehouse3),
at(item222, warehouse3),
at(item223, warehouse3),
at(item224, warehouse3),
at(item225, warehouse3),
at(item226, warehouse3),
at(item227, warehouse3),
at(item228, warehouse3),
at(item229, warehouse3),
at(item230, warehouse3),
at(item231, warehouse3),
at(item232, warehouse3),
at(item233, warehouse3),
at(item234, warehouse3),
at(item235, warehouse4),
at(item236, warehouse4),
at(item237, warehouse4),
at(item238, warehouse4),
at(item239, warehouse4),
at(item240, warehouse4),
at(item241, warehouse4),
at(item242, warehouse4),
at(item243, warehouse4),
at(item244, warehouse4),
at(item245, warehouse4),
at(item246, warehouse4),
at(item247, warehouse4),
at(item248, warehouse4),
at(item249, warehouse4),
at(item250, warehouse4),
at(item251, warehouse4),
at(item252, warehouse4),
at(item253, warehouse4),
at(item254, warehouse4),
at(item255, warehouse4),
at(item256, warehouse4),
at(item257, warehouse4),
at(item258, warehouse4),
at(item259, warehouse4),
at(item260, warehouse4),
at(item261, warehouse4),
at(item262, warehouse4),
at(item263, warehouse4),
at(item264, warehouse4),
at(item265, warehouse4),
need(0, product1, order0),
need(1, product2, order0),
need(2, product0, order1),
need(3, product1, order1),
need(4, product0, order2),
need(5, product4, order3),
need(6, product0, order3),
need(7, product0, order3),
need(8, product1, order4),
need(9, product0, order4),
need(10, product1, order4),
need(11, product1, order5),
need(12, product3, order6),
need(13, product3, order6),
need(14, product3, order7),
need(15, product3, order7),
need(16, product1, order7),
need(17, product1, order8),
need(18, product3, order8),
need(19, product0, order9),
need(20, product4, order10),
need(21, product2, order10),
need(22, product4, order10),
need(23, product4, order11),
need(24, product1, order11),
need(25, product0, order12),
need(26, product4, order13),
need(27, product0, order13),
need(28, product3, order14),
need(29, product3, order14),
need(30, product3, order15),
need(31, product4, order16),
need(32, product0, order16),
need(33, product3, order16),
need(34, product4, order17),
need(35, product4, order18),
need(36, product0, order18),
need(37, product4, order18),
need(38, product3, order19),
need(39, product3, order19),
need(40, product0, order19),
need(41, product4, order20),
need(42, product2, order20),
need(43, product2, order21),
need(44, product1, order21),
need(45, product4, order22),
need(46, product3, order22),
need(47, product4, order23),
need(48, product2, order23),
need(49, product3, order24),
need(50, product1, order25),
need(51, product0, order25),
need(52, product1, order26),
need(53, product3, order26),
need(54, product0, order27),
need(55, product4, order28),
need(56, product3, order28),
need(57, product2, order28),
need(58, product4, order29),
need(59, product4, order29),
need(60, product3, order29)
],
    [at(0, product1, order0),
at(1, product2, order0),
at(2, product0, order1),
at(3, product1, order1),
at(4, product0, order2),
at(5, product4, order3),
at(6, product0, order3),
at(7, product0, order3),
at(8, product1, order4),
at(9, product0, order4),
at(10, product1, order4),
at(11, product1, order5),
at(12, product3, order6),
at(13, product3, order6),
at(14, product3, order7),
at(15, product3, order7),
at(16, product1, order7),
at(17, product1, order8),
at(18, product3, order8),
at(19, product0, order9),
at(20, product4, order10),
at(21, product2, order10),
at(22, product4, order10),
at(23, product4, order11),
at(24, product1, order11),
at(25, product0, order12),
at(26, product4, order13),
at(27, product0, order13),
at(28, product3, order14),
at(29, product3, order14),
at(30, product3, order15),
at(31, product4, order16),
at(32, product0, order16),
at(33, product3, order16),
at(34, product4, order17),
at(35, product4, order18),
at(36, product0, order18),
at(37, product4, order18),
at(38, product3, order19),
at(39, product3, order19),
at(40, product0, order19),
at(41, product4, order20),
at(42, product2, order20),
at(43, product2, order21),
at(44, product1, order21),
at(45, product4, order22),
at(46, product3, order22),
at(47, product4, order23),
at(48, product2, order23),
at(49, product3, order24),
at(50, product1, order25),
at(51, product0, order25),
at(52, product1, order26),
at(53, product3, order26),
at(54, product0, order27),
at(55, product4, order28),
at(56, product3, order28),
at(57, product2, order28),
at(58, product4, order29),
at(59, product4, order29),
at(60, product3, order29)
]
).