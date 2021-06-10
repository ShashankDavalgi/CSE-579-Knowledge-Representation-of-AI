
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Robot in node1 picks up shelf in node2 and crosses highway in node 3 to deliver to pickup in node 4

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%% set ROBOT 1 at 1*1 %%%%%%%%%
init(object(robot,1),value(at,pair(1,1))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%% init HIGHWAY %%%%%%%%%
init(object(highway,4),value(at,pair(3,1))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%% init TABLE with NODES from 1*1 to 4*1 %%%%%%%%%

init(object(node,1),value(at,pair(1,1))).

init(object(node,2),value(at,pair(2,1))).

init(object(node,3),value(at,pair(3,1))).

init(object(node,4),value(at,pair(4,1))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% Init the picking station 3 %%%%%%%%

init(object(pickingStation,3),value(at,pair(4,1))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% Init SHELF 1 and PRODUCT 1 on SHELF 1 %%%%%%%%%%

init(object(shelf,1),value(at,pair(2,1))).

init(object(product,1),value(on,pair(1,2))).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% set ORDER 4 with 2 number of PRODUCT 1 %%%%%%%%
init(object(order,4),value(line,pair(1,2))).

%%%%%%%% set ORDER 4 at picking station 3 %%%%%%%% 
init(object(order,4),value(pickingStation,3)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%% order should not have 0 number of product 1 %%%%%%%%
:- not order(4,3,1,0).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%