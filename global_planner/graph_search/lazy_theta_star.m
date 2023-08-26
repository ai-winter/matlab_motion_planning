function [path, goal_reached, cost, EXPAND] = lazy_theta_star(map, start, goal)
% @file: lazy_theta_star.m
% @breif: Lazy Theta* motion planning
% @paper: Lazy Theta*: Any-Angle Path Planning and Path Length Analysis in 3D
% @author: Winter
% @update: 2023.8.26

%
%   == OPEN and CLOSED ==
%   [x, y, g, h, px, py]
%   =====================
%

% initialize
OPEN = [];
CLOSED = [];
EXPAND = [];

cost = 0;
goal_reached = false;
motion = [-1, -1, sqrt(2); ...
    0, -1, 1; ...
    1, -1, sqrt(2); ...
    -1, 0, 1; ...
    1, 0, 1; ...
    -1, 1, sqrt(2); ...
    0, 1, 1; ...
    1, 1, sqrt(2)];

motion_num = size(motion, 1);

node_s = [start, 0, h(start, goal), start];
OPEN = [OPEN; node_s];

while ~isempty(OPEN)
    % pop
    f = OPEN(:, 3) + OPEN(:, 4);
    [~, index] = min(f);
    cur_node = OPEN(index, :);
    OPEN(index, :) = [];
        
    % set vertex: path 1
    p_index = loc_list(cur_node(5: 6), CLOSED, [1, 2]);
    if p_index
        node_p = CLOSED(p_index, :);
        if line_of_sight(map, node_p, cur_node)
            cur_node(3) = inf;
            for i = 1:motion_num
                node_n_x = cur_node(1) + motion(i, 1);
                node_n_y = cur_node(2) + motion(i, 2);
                np_index = loc_list([node_n_x, node_n_y], CLOSED, [1, 2]);
                if np_index
                    node_n_p = CLOSED(np_index, :);
                    if cur_node(3) > node_n_p(3) + dist(node_n_p(1: 2), cur_node(1: 2)')
                        cur_node(3) = node_n_p(3) + dist(node_n_p(1: 2), cur_node(1: 2)');
                        cur_node(5) = node_n_x;
                        cur_node(6) = node_n_y;
                    end
                end
            end
       
        end
    end
    
    % exists in CLOSED set
    if loc_list(cur_node, CLOSED, [1, 2])
        continue
    end

    % update expand zone
    if ~loc_list(cur_node, EXPAND, [1, 2])
        EXPAND = [EXPAND; cur_node(1:2)];
    end

    % goal found
    if cur_node(1) == goal(1) && cur_node(2) == goal(2)
        CLOSED = [cur_node; CLOSED];
        goal_reached = true;
        cost = cur_node(3);
        break
    end
    if (cur_node(1) ==17) &&(cur_node(2) == 26)
        cur_node(1);
    end
    % explore neighbors
    for i = 1:motion_num
        % path 1
        node_n = [
            cur_node(1) + motion(i, 1), ...
            cur_node(2) + motion(i, 2), ...
            cur_node(3) + motion(i, 3), ...
            0, ...
            cur_node(1), cur_node(2)];
        node_n(4) = h(node_n(1:2), goal);

        % exists in CLOSED set
        if loc_list(node_n, CLOSED, [1, 2])
            continue
        end

        % obstacle
        if map(node_n(1), node_n(2)) == 2
            continue
        end

        p_index = loc_list(cur_node(5: 6), CLOSED, [1, 2]);
        if p_index
            node_p = CLOSED(p_index, :);
        else
            node_p = 0;
        end
        
        if node_p ~= 0
            node_n = update_vertex(node_p, node_n);
        end
        
        % update OPEN set
        OPEN = [OPEN; node_n];
    end
    CLOSED = [cur_node; CLOSED];
end

% extract path
path = extract_path(CLOSED, start);
end

%%
function h_val = h(node, goal)
% @breif: heuristic function (Euclidean distance)
h_val = dist(node(1: 2), goal');
end

function index = loc_list(node, list, range)
% @breif: locate the node in given list
num = size(list);
index = 0;

if ~num(1)
    return
else
    for i = 1:num(1)
        if isequal(node(range), list(i, range))
            index = i;
            return
        end
    end
end
end

function node_c = update_vertex(node_p, node_c)
% @breif: Update extend node information with current node's parent node.
    % path 2
    if node_p(3) + dist(node_c(1: 2), node_p(1: 2)') <= node_c(3)
        node_c(3) = node_p(3) + dist(node_c(1: 2), node_p(1: 2)');
        node_c(5: 6) = node_p(1: 2);
    end
end

function flag = line_of_sight(map, node1, node2)
% @breif: Judge collision when moving from node1 to node2 using Bresenham.
    if (map(node1(1), node1(2)) == 2) || (map(node2(1), node2(2)) == 2)
        flag = true;
        return
    end
    x1 = node1(1); y1 = node1(2);
    x2 = node2(1); y2 = node2(2);
    
    d_x = abs(x2 - x1);
    d_y = abs(y2 - y1);
    if  (x2 - x1) == 0
        s_x = 0;
    else
        s_x = (x2 - x1) / d_x;
    end
    if  (y2 - y1) == 0
        s_y = 0;
    else
        s_y = (y2 - y1) / d_y;
    end
    x = x1; y = y1; e = 0;
    
    % check if any obstacle exists between node1 and node2
    if d_x > d_y
        tao = (d_y - d_x) / 2;
        while x ~= x2
            if e > tao
                x = x + s_x;
                e = e - d_y;
            elseif e < tao
                y = y + s_y;
                e = e + d_x;
            else
                x = x + s_x;
                y = y + s_y;
                e = e + d_x - d_y;
            end
            if map(x, y) == 2
                flag = true;
                return;
            end
        end
    % swap x and y
    else
        tao = (d_x - d_y) / 2;
        while y ~= y2
            if e > tao
                y = y + s_y;
                e = e - d_x;
            elseif e < tao
                x = x + s_x;
                e = e + d_y;
            else
                x = x + s_x;
                y = y + s_y;
                e = e + d_y - d_x;
            end
            if map(x, y) == 2
                flag = true;
                return;
            end
        end
    end
    flag = false;
end

function path = extract_path(close, start)
% @breif: Extract the path based on the CLOSED set.
path = [];
closeNum = size(close, 1);
index = 1;

while 1
    path = [path; close(index, 1:2)];

    if isequal(close(index, 1:2), start)
        break
    end

    for i = 1:closeNum
        if isequal(close(i, 1:2), close(index, 5:6))
            index = i;
            break
        end
    end
end
end
