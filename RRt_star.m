%%--------------------------------------------------------------------

% Module:  RRT_star.m

% This module contains all the computations related to RRT Star

% Search Algorithm

% Developed by Rishi.Khajuriwala@ WPI RBE.

%%--------------------------------------------------------------------*/

clc;
clear ;
close all;

grid = [100,100];
Obstacle1 = [50,15,20,20];
Obstacle2 = [20,50,20,10];
Obstacle3 = [70,20,20,20];
Obstacle4 = [20,70,10,20];

x_start.point = [0 0];
x_goal.point = [95 95];
x_start.cost = 0;
x_start.parent = 0;
n = 20;
nodes(1) = x_start;
figure;
axis([0 grid(1) 0 grid(2)]);
rectangle('Position',Obstacle1,'FaceColor',[0 0 0]);

hold on
rectangle('Position',Obstacle2,'FaceColor',[0 0 0]);
rectangle('Position',Obstacle3,'FaceColor',[0 0 0]);
rectangle('Position',Obstacle4,'FaceColor',[0 0 0]);

n_nodes = 400; %No of nodes, decided by the user


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
    if ObstacleFree(x_rand,x_near.point,Obstacle1) && ObstacleFree(x_rand,x_near.point,Obstacle2) && ObstacleFree(x_rand,x_near.point,Obstacle3) && ObstacleFree(x_rand,x_near.point,Obstacle4)
        line([x_near.point(1),x_new.point(1)],[x_near.point(2),x_new.point(2)],'Color','k','LineWidth',2);
        drawnow;
        hold on
        x_new.cost = NearestNeighorDist(x_new.point,x_near.point) + x_near.cost;
        x_nearest = [];
%         disp(ObstacleFree(x_rand,x_near.point,Obstacle1));
%         disp('2');
%         disp(ObstacleFree(x_rand,x_near.point,Obstacle2));
%         disp('3');
%         disp(ObstacleFree(x_rand,x_near.point,Obstacle3));
%         disp('4');
%         disp(ObstacleFree(x_rand,x_near.point,Obstacle4));
%         
        r = 15;
        sweep = 1;
        for j = 1:ld_nodes
            if ObstacleFree(nodes(j).point, x_new.point, Obstacle1) && ObstacleFree(nodes(j).point, x_new.point, Obstacle2) && ObstacleFree(nodes(j).point, x_new.point, Obstacle3) && ObstacleFree(nodes(j).point, x_new.point, Obstacle4) && NearestNeighorDist(nodes(j).point, x_new.point) <= r
                x_nearest(sweep).point = nodes(j).point;
                x_nearest(sweep).cost = nodes(j).cost;
                sweep = sweep+1;
%                 disp(x_nearest)
            end
        end  
        ld_x_nearest = length(x_nearest);
        x_min = x_near;
        min_cost = x_near.cost;
        for i = 1: ld_x_nearest
           if ObstacleFree(x_nearest(i).point, x_new.point, Obstacle1) && ObstacleFree(x_nearest(i).point, x_new.point, Obstacle2) && ObstacleFree(x_nearest(i).point, x_new.point, Obstacle3) && ObstacleFree(x_nearest(i).point, x_new.point, Obstacle4) && x_nearest(i).cost + NearestNeighorDist(x_nearest(i).point, x_new.point) < min_cost
%                disp('hi') 
               x_min = x_nearest(i);
                min_cost = x_nearest(i).cost + NearestNeighorDist(x_nearest(i).point,x_new.point);
                line([x_min.point(1), x_new.point(1)], [x_min.point(2), x_new.point(2)], 'Color', 'g','LineWidth',2);  
                hold on;
           end
        end
        
        for j = 1: ld_nodes
            if nodes(j).point == x_min.point
                x_new.parent = j;
            end
        end
        nodes = [nodes; x_new];
%     disp('HI')
    end
end
list = [];
for j = 1:ld_nodes
    list = [list;NearestNeighorDist(nodes(j).point, x_goal.point)];
end

% Search backwards from goal to start to find the optimal least cost path
[min_list, idcs] = min(list);
x_final = nodes(idcs);
x_goal.parent = idcs;
x_end = x_goal;
% nodes = [nodes ; x_end];
while x_end.parent ~= 0
    start = x_end.parent;
    line([x_end.point(1), nodes(start).point(1)], [x_end.point(2), nodes(start).point(2)], 'Color', 'r', 'LineWidth', 5);
    hold on
    x_end = nodes(start);
end
% % P = rectangle(Obstacle2);
% P = [8 7; 10 7;10 9; 8 9]';
% % rectangle('Position',Obstacle2,'FaceColor',[0 0 0]);
% s1 = [x_near.point;x_rand];
% 
% X = seg2poly(s1, P);
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
distance = sqrt((x(1)-y(1))^2+(x(2)-y(2))^2);
end

%% Near Vertices:



%% Collision Test/ Obstacle Free
% function no_collison = ObstacleFree(rand_x,near_x,Obstacle)
% Obstacle = [x y h b]; % here e the x, y corordinates on the lower left side of the rectange and h and b are height and breadth
% obs_x = [x,x,x+h,x+h];
% obs_y = [y,y+b,y+b,y];
% 
% 
% 
% 
% end




function [no_collison] = ObstacleFree(rand_x,near_x_point, P)
% function X = seg2poly(s1, P)
% Check if a line segment s1 intersects with a polygon P.
% INPUTS:
%   s is (2 x 2) where
%     s(:,1) is the first point
%     s(:,2) is the the second point of the segment.
%   P is (2 x n) array, each column is a vertices
% OUTPUT
%   X is (2 x m) array, each column is an intersecting point
%
%   Author: Bruno Luong <brunoluong@yahoo.com>
%   History:
%       Original 20-May-2010
% Citation: https://www.mathworks.com/matlabcentral/fileexchange/27673-2d-polygon-edges-intersection
% Translate so that first point is origin
s1 = [rand_x;near_x_point];
a = s1(:,1);
M = bsxfun(@minus, P, a);
b = s1(:,2)-a;
% Check if the points are on the left/right side
x = [b(2) -b(1)]*M;
sx = sign(x);
% x -coordinates has opposite signs
ind = sx(1:end-1).*sx(2:end) <= 0;
if any(ind)
    ind = find(ind);
    % cross point to the y-axis (along the segment)
    x1 = x(ind);
    x2 = x(ind+1);
    d = b.'/(b(1)^2+b(2)^2);
    y1 = d*M(:,ind);
    y2 = d*M(:,ind+1);
    dx = x2-x1;
    % We won't bother with the degenerate case of dx=0 and x1=0
    y = (y1.*x2-y2.*x1)./dx;
    % Check if the cross point is inside the segment
    ind = y>=0 & y<1;
    if any(ind)
        X = bsxfun(@plus, a, b*y(ind));
    else
        X = zeros(2,0);
    end
else
    X = zeros(2,0);

end
if X ~= 0
    no_collison = 0; % Flase
else
    no_collison = 1; % True
end
end
