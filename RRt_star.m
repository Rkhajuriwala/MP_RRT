%%--------------------------------------------------------------------

% Module:  RRT_star.m

% This module contains all the computations related to RRT Star

% Search Algorithm

% Developed by Rishi.Khajuriwala@ WPI RBE.

%%--------------------------------------------------------------------*/

clc;
clear all;
close all;

grid = [10,10];
Obstacle1 = [5,1.5,2,2];
Obstacle2 = [2,5,2,2];
Obstacle3 = [7.5,2,2,2];
Obstacle4 = [8,7,2,2];

x_start.point = [0 0];
x_goal.point = [9 9];
n = 200;
nodes(1) = x_start;
figure;
axis([0 grid(1) 0 grid(2)]);
rectangle('Position',Obstacle1,'FaceColor',[0 0 0]);

hold on
rectangle('Position',Obstacle2,'FaceColor',[0 0 0]);
rectangle('Position',Obstacle3,'FaceColor',[0 0 0]);
rectangle('Position',Obstacle4,'FaceColor',[0 0 0]);

n_nodes = 1000; %No of nodes, decided by the user


% Sampling Function
for j = 1: n_nodes
   x_rand = [round(rand(1)*grid(1)) round(rand(1)*grid(2))];
   plot(x_rand(1),x_rand(2),'x','Color',[0 0 0]);
   
   ld_nodes = length(nodes);
    for i = 1: ld_nodes
        if nodes(i).point == x_goal.point
            break
        end
    end
    dist_closet_nodes = [];
    for k = 1:ld_nodes
       dist_closet_nodes = [dist_closet_nodes;NearestNeighorDist(nodes(k).point,x_rand)]; 
    end
    
    [min_dis,idcs] = min(dist_closet_nodes);
    x_near = nodes(idcs);
    
    x_new.point = Steer(x_rand,x_near.point,min_dis,n);
    plot(x_new.point);
end




%% Steering
% Given 2 points x and y, this function returns a point z such that it is
% closer to y than x.

function [new_x] = Steer(rand_x,nearest_node,min_node,n)
new_X = [0 0];

    if min_node >= n
       new_X(1) =  nearest_node(1) + ((rand_x(1)-nearest_node(1))*n)/NearestNeighorDist(rand_x,nearest_node);
       new_X(2) =  nearest_node(2) + ((rand_x(2)-nearest_node(2))*n)/NearestNeighorDist(rand_x,nearest_node);
    else
        new_X(1) = rand_x(1);
        new_X(2) = rand_x(2);

    end
    new_x = [new_X(1), new_X(2)];
end


%% Nearest Neighor
%Given a graph G(V,E) and point x, this function gives the 
%Euclidean Distance

function [distance] = NearestNeighorDist(x,y)
distance = sqrt((x(1)-y(1)^2)+(x(2)-y(2)^2));
end

%% Near Vertices:



%% Collision Test/ Obstacle Free

