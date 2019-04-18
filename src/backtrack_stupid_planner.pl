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
    open('out/{{filename}}.cmds', write, Stream),
    export_moves(Moves, Stream),
    close(Stream)
    {{ debug }}.

%%
% When the Goal state is a subset of the valued state
% Use a defined action to move through the state-space
%%
plan(State, Goal, Been_list, Moves, MaxTurns) :-
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

%%
%% Utility predicates
%%

%%
% returns the load weight of a drone
%%
drone_load([], _, _) :- fail, !.
drone_load([weighs(Drone, X)|_], Drone, Weight) :- Weight is X, !.
drone_load([_|T], Drone, Weight) :- drone_load(T, Drone, Weight).

drone_location([], _, _) :- fail.
drone_location([at(Drone, coord(X, Y))|_], Drone, coord(X, Y)) :- !.
drone_location([at(Drone, Warehouse)|_], Drone, Warehouse) :- warehouse(Warehouse, _), !.
drone_location([at(Drone, Order)|_], Drone, Order) :- order(Order, _, _), !.
drone_location([_|T], Drone, Location) :- drone_location(T, Drone, Location).

requested_product_and_order([], _, _, _) :- fail, !.
requested_product_and_order([need(NeedId, Product, Order)|_], Order, Product, NeedId) :- !.
requested_product_and_order([_|T], Order, Product, NeedId) :- requested_product_and_order(T, Order, Product, NeedId).

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
    item(Item, Product),
    drone_location(State, Drone, PrevDroneLocation),
    drone_load(State, Drone, CurrentWeight),
    payload(MaxWeight),
    product(Product, ProductWeight),
    CurrentWeight + ProductWeight #=< MaxWeight,
    NewWeight is CurrentWeight + ProductWeight,
    requested_product_and_order(State, Order, Product, NeedId).

move(
    State,
    deliver(Drone, Product, Order, 0),
    [at(Item, Drone), delivering(NeedId, Item, Order, Drone)],
    [
        del(at(Item, Drone)), del(weighs(Drone, CurrentWeight)), del(delivering(NeedId, Item, Order, Drone)), del(at(Drone, PrevDroneLocation)),
        add(at(NeedId, Product, Order)), add(weighs(Drone, NewWeight)), add(at(Drone, Order))
    ]
) :-
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
