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
    open('out/{{filename}}.cmds', write, Stream),
    export_moves(Moves, Stream),
    close(Stream)
    {{ debug }}.
    % write('Turns: '), write(UsedTurns).

%%
% When the Goal state is a subset of the valued state
% Use a defined action to move through the state-space
%%
plan(State, Goal, BeenList, Moves, MaxTurns) :-
    % turns_used(Moves, UsedTurns),
    % UsedTurns #=< MaxTurns,
    bagof(Drone, drone(Drone), DroneList),
    drones_move(DroneList, State, BeenList, Moves, ChildState, NewBeenList, NewMoves),
    % not(member_state(ChildState, BeenList)),
    plan(ChildState, Goal, NewBeenList, NewMoves, MaxTurns).

drones_move([], State, BeenList, Moves, OutState, OutBeenList, OutMoves) :-
    OutState = State,
    OutBeenList = BeenList,
    OutMoves = Moves.
drones_move([Drone|Tail], State, BeenList, Moves, OutState, OutBeenList, OutMoves) :-
    move(Drone, State, Name, Preconditions, Actions),
    conditions_met(Preconditions, State),
    write(Name), nl,
    change_state(State, Actions, ChildState),
    stack(ChildState, BeenList, NewBeenList),
    stack(Name, Moves, NewMoves),
    drones_move(Tail, ChildState, NewBeenList, NewMoves, OutState, OutBeenList, OutMoves).

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
    Drone,
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
    warehouse(Warehouse, _),
    item(Item, Product),
    drone(Drone),
    drone_load(State, Drone, CurrentWeight),
    payload(MaxWeight),
    product(Product, ProductWeight),
    CurrentWeight + ProductWeight #=< MaxWeight,
    NewWeight is CurrentWeight + ProductWeight,
    distance(State, Drone, Warehouse, Distance),
    TurnsConsumed is Distance + 1.

move(
    Drone,
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

move(
    Drone,
    _,
    wait(Drone),
    [],
    []
) :-
    drone(Drone).

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
