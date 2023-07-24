function [path, goal_reached, cost, EXPAND] = voronoi_plan(map, start, goal)
% @file: voronoi_plan.m
% @breif: Voronoi-based motion planning
% @author: Winter
% @update: 2023.7.24

%%
    % Maximum expansion distance one step
    max_dist = 3;
    % map size
    [y_range, x_range] = size(map);
    % resolution
    resolution = 0.1;
    % number of edges from one sampled point
    n_knn = 5;
    
    % construct Voronoi diagram
    [ox, oy] = find(map == 2);
    [vx, vy] = voronoi(oy, ox);
    start(:, [1 2]) = start(:, [2 1]);
    goal(:, [1 2]) = goal(:, [2 1]);

    % Voronoi diagram filter
    index_x = intersect(find(vx(1, :) > 0 & vx(1, :) < x_range), ...
                                     find(vx(2, :) > 0 & vx(2, :) < x_range));
    index_y = intersect(find(vy(1, :) > 0 & vy(1, :) < y_range), ...
                                     find(vy(2, :) > 0 & vy(2, :) < y_range));
    index = intersect(index_x, index_y);
    vx = vx(:, index); vy = vy(:, index);
    vd_vertex = [];
    EXPAND = [];
    for i = 1:length(index)
        node1 = [vx(1, i), vy(1, i)];
        node2 = [vx(2, i), vy(2, i)]; 

        if ~all(node1 == node2) && ~is_collision(node1, node2, map, -1, resolution)
            EXPAND = [EXPAND, [vx(:, i); vy(:, i)]];
            vd_vertex = [vd_vertex; [vx(:, i), vy(:, i)]];
        end
    end
    vd_vertex = [unique(vd_vertex, 'rows'); start; goal];
    
    % generate road map for voronoi nodes
    road_map = containers.Map();
    num_vd = size(vd_vertex, 1);
    for i = 1:num_vd
        knn_nodes = vd_vertex(knnsearch(vd_vertex, vd_vertex(i, :), 'K', num_vd), :);
        edges = [];
        for j = 1:num_vd
            if ~is_collision(vd_vertex(i, :), knn_nodes(j, :), map, max_dist, resolution)
                edges = [edges; knn_nodes(j, :)];
            end
            if size(edges, 1) == n_knn
                break;
            end
        end
        % hash-map: from grid index to edges
        road_map(string(vd_vertex(i, 1) + x_range * vd_vertex(i, 2))) = edges;
    end

    [path, goal_reached, cost] = get_shortest_path(road_map, start, goal, map, max_dist, resolution);
    if goal_reached
        path(:, [1 2]) = path(:, [2 1]);
    else
        path = [];
        cost = 0;
    end
end

%%
function [path, goal_reached, cost] = get_shortest_path(road_map, start, goal, map, max_dist, resolution)
%@breif: Calculate shortest path using graph search algorithm(A*, Dijkstra, etc).
    % initialize
    x_range = size(map, 2);
    OPEN = [];
    CLOSED = [];
    
    cost = 0;
    goal_reached = false;

    node_s = [start, 0, 0, start];
    OPEN = [OPEN; node_s];

    while ~isempty(OPEN)
        % pop
        [~, index] = min(OPEN(:, 3));
        cur_node = OPEN(index, :);
        OPEN(index, :) = [];

        % exists in CLOSED set
        if loc_list(cur_node, CLOSED, [1, 2])
            continue
        end

        % goal found
        if cur_node(1) == goal(1) && cur_node(2) == goal(2)
            CLOSED = [cur_node; CLOSED];
            goal_reached = true;
            cost = cur_node(3);
            break
        end

        % explore knn
        knn_nodes = road_map(string(cur_node(1) + x_range * cur_node(2)));
        for i = 1:size(knn_nodes, 1)
            node_n = [
                knn_nodes(i, 1), ...
                knn_nodes(i, 2), ...
                cur_node(3) + dist(cur_node(1: 2), knn_nodes(i, :)'), ...
                0, ...
                cur_node(1), cur_node(2)];

            % exists in CLOSED set
            if is_collision(cur_node(1:2), node_n(1:2), map, max_dist, resolution)
                continue
            end

            % update OPEN set
            OPEN = [OPEN; node_n];
        end
        CLOSED = [cur_node; CLOSED];
    end

    % extract path
    path = extract_path(CLOSED, start);

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

function flag = is_collision(node1, node2, map, max_dist, resolution)
%@breif: Judge collision when moving from node1 to node2.
    flag = true;
    theta = angle(node1, node2);
    distance = dist(node1, node2');

      % distance longer than the threshold
      if (max_dist > 0 && distance > max_dist)
          return
      end
        
      % sample the line between two nodes and check obstacle
      n_step = round(distance / resolution);
      for i=1:n_step
          x = node1(1) + i * resolution * cos(theta);
          y = node1(2) + i * resolution * sin(theta);
          if map(round(y), round(x)) == 2
              return
          end
      end
      
      flag = false;
end

 function theta = angle(node1, node2)
    theta = atan2(node2(2) - node1(2), node2(1) - node1(1));
 end
