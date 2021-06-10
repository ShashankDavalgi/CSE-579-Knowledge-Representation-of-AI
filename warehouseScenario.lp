%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Auto Warehouse Project - Group 3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%
% Grid Constraints
%%%%%%%%%%%%%%%%%%%%%%%
node(C, D):- init(object (node, X) , value (at, pair(C, D))).

pickingStation(P, C, D):- init(object(pickingStation, P), value(at, pair(C ,D))).

highway(C, D):- init(object(highway, H), value(at, pair(C ,D))).


%%%%%%%%%%%%%%%%%%%%%%%
% Product Constraints
%%%%%%%%%%%%%%%%%%%%%%%
product(X):- init(object(product, X), value(on, pair(S ,Y))).

% On some Shelf, Product X (Y units)
on(X, shelf, S, Y, 0):- init(object(product, X), value(on, pair(S ,Y))).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Costraints for Shelf and Constraints for Robot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
object(X,I):- init(object(X, I), value(at, pair(C, D))), X!=highway, X!=pickingStation, X!=node.
object(shelf, shelly).

% shelf and robot init relation
carry(R, shelly, 0):- object(robot, R).

% Robot and Shelf init position
at(object(X, I), C, D, 0):- init(object(X, I), value(at, pair(C, D))), object(X, I).


%%%%%%%%%%%%%%%%%%%%%%%%%
% Constraints for Orders 
%%%%%%%%%%%%%%%%%%%%%%%%%
order(O, I, Y, 0):- init(object(order, O), value(line, pair(I, Y))).
orderPickStat(O, P):- init(object(order, O), value(pickingStation, P)).


%%%%%%%%%%%%%%%%%%%%%%%
% Moving Constraints
%%%%%%%%%%%%%%%%%%%%%%%
% Valid directions for the robot movement
move(0,-1).move(1,0).move(-1,0).move(0,1).

% Do not allow the robot to go outside the grid
:- not node(P, Q), at(object(robot, R), P, Q, T), T = 1..m.
{occurs(object(robot, R), move(C, D), T)}:- move(C, D), object(robot, R), T = 1..m.

% If there are two robots then do not allow them to swap the locations
:- at(object(robot, R2), C, D, T), at(object(robot, R2), P, Q, T-1), at(object(robot, R1), C, D, T-1), at(object(robot, R1), P, Q, T), R2!=R1, T=1..m. 

% At a given time two shelves should not be there
:- at(object(shelf, S2), C, D, T), at(object(shelf, S1), C, D, T), S1!=S2,T=0..m.

% Moving effect rules
at(object(shelf, S), P + C, Q + D, T):- object(shelf, S), occurs(object(robot, R), move(C, D), T), at(object(robot, R), P, Q, T - 1), carry(R, S, T - 1), S!=shelly, T = 1..m.
at(object(robot, R), P + C, Q + D, T):- object(robot, R), occurs(object(robot, R), move(C, D), T), at(object(robot ,R), P, Q, T-1), T = 1..m.

% At a given time two robots should not be there
:- at(object(robot, R1), C, D, T), at(object(robot, R2), C, D, T), R2 != R1, T=0..m.


%%%%%%%%%%%%%%%%%%%%%%%%
% Puting down Rules
%%%%%%%%%%%%%%%%%%%%%%%%
% After putting down robot must not carry
carry(R, shelly, T):- occurs(object(robot, R), putdown, T), T = 1..m.

% Putting down effect
at(object(shelf, S), C, D, T):- carry(R, S, T - 1), occurs(object(robot, R), putdown, T), at(object(robot, R), C, D, T - 1), T = 1..m.

% Highways should not contain shelf
:- highway(C, D), at(object (robot, R), C, D, T - 1), occurs(object(robot, R), putdown, T), T = 1..m.

{occurs(object(robot, R), putdown, T)} :- object(robot, R), carry(R, S, T - 1), S != shelly, T = 1..m.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constraints for Picking up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% If a robot is carrying then it should not pick again
:- carry(R, S, T - 1), occurs(object(robot, R), pickup, T), S!=shelly, T = 1..m.

% At a given time T - 1, Both robot and shelf should be in the same node
carry(R, S, T) :- at(object(robot, R), C, D, T-1), at(object(shelf, S), C, D, T - 1), occurs(object(robot, R), pickup, T), T = 1..m.

% Robot R can pickup shelf S if R and S are at the same node at T-1
{occurs(object(robot, R), pickup, T)} :- object(robot, R), at(object(robot, R), C, D, T - 1), at(object(shelf, S), C, D, T - 1), T = 1..m.


%%%%%%%%%%%%%%%%%%%%%%%%
% Rules for delivery
%%%%%%%%%%%%%%%%%%%%%%%%
% Do not deliver a 0 unit item
:- object(robot, R), occurs(object(robot, R), deliver(O, I, A), T), A = 0, T = 1..m.


% Subtraction of delivery amount (A) from shelf amount (Y) where X is shelf number
order(O,I,E-A,T):- order(O, I, E, T - 1), occurs(object(robot, R), deliver(O, I, A), T), T = 1..m.
on(I, shelf, X, Y - A, T) :- on(I, shelf, X, Y, T - 1), occurs(object(robot, R), deliver(O, I, A), T), T = 1..m.

% Order matching the shelf a robot carries and a Robot present at same node as picking station
{occurs(object(robot, R), deliver(O, I, Y), T)}:- at(object(robot, R), C, D, T - 1), pickingStation(P, C, D), orderPickStat(O, P), carry(R, S, T - 1), Y <= E, on(I, shelf, S, Y, T - 1), order(O, I, E, T - 1), T = 1..m.
{occurs(object(robot, R), deliver(O, I, E), T)}:- pickingStation(P, C, D), at(object(robot, R), C, D,T - 1), orderPickStat(O, P), carry(R, S, T - 1), Y > E, on(I, shelf, S, Y, T - 1), order(O, I, E, T - 1),T = 1..m.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constraints for the Commonsense law of inertia
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
{at(object (X ,I), C, D, T + 1)} :- at(object (X, I), C, D, T), T = 0..m-1.

{on(X, K, S, Y, T+ 1)} :- on(X, K, S, Y, T), T = 0..m-1.

{carry(R, S, T + 1)} :- carry(R, S, T), T = 0..m-1.

{order(O, I, E, T + 1)}:- order(O, I, E, T), T = 0..m-1.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Constraints for uniqueness
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:-not {order(O, I, E, T)}=1, order(O, I, Y, 0), T = 1..m.
:-not {at(object (robot ,R), C, D, T)}=1, object (robot ,R),T = 1..m.
:-not {carry(R, S, T)}=1, object (robot ,R),T = 1..m.
:-not {at(object (shelf ,S), C, D, T)}=1, object(shelf, S), S != shelly, T = 1..m.

% Only one occurs per robot R at time T
action(pickup).
action(putdown).
action(move(C, D)) :- move(C, D).
0{occurs(object(robot, R), A, T) : action(A)}1 :- object(robot, R), T = 1..m.
:- occurs(object(robot, R), deliver(O, I, P), T),occurs(object(robot, R), A, T), action(A), T = 1..m.


#show occurs/3.

