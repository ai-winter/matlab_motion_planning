function [path, goal_reached, cost, EXPAND] = aco(map, start, goal)
%%
% @file: aco.m
% @breif: Ant Colony Optimization(ACO) motion planning
% @paper: Ant Colony Optimization: A New Meta-Heuristic
% @author: Winter
% @update: 2023.7.14

%%
    % Number of ants
    n_ants = 25;
    % Pheromone and heuristic factor weight coefficient
    alpha = 1.0;
    beta = 5.0;
    % Evaporation coefficient
    rho = 0.1;
    % Pheromone gain
    Q = 1.0;
    % Maximum iterations
    max_iter = 20;
    % Map size
    [x_range, y_range] = size(map);

    cost = 0;
    goal_reached = false;
    EXPAND = [];
    
    % Pheromone matrix
    % [x, y, nx, ny, value]
    pheromone_edges = [];
    for i=1:x_range
        for j=1:y_range
            % obstacle             
            if map(i, j) == 2
                continue
            end
            % neighbor detection
            neighbors = getNeighbor([i, j], map);
            for k=1:length(neighbors)
                pheromone_edges = [pheromone_edges; i, j, neighbors(k, 1), neighbors(k, 2), 1.0];
            end
        end
    end
    
    % heuristically set max steps
     max_steps = x_range * y_range / 2 + max(x_range, y_range);
    % iteration parameters      
     best_length_list = [];
     best_path = [];
     
     % main loop
     for i=1:max_iter
         ants_list = struct([]);
         for j=1:n_ants
             % ant initialization
             ants_list(j).steps = 0;
             ants_list(j).found_goal = false;
             ants_list(j).cur_node = start;
             ants_list(j).path = [start, -1, -1]; % [x, y, px, py]
             while ants_list(j).steps < max_steps && ~all(ants_list(j).cur_node == goal)
                 % candidate
                 prob_sum = 0;
                 next_positions = [];
                 next_probabilities = [];
                 % neighbor detection
                 neighbors = getNeighbor(ants_list(j).cur_node, map);
                 for k=1:length(neighbors)
                     node_n = neighbors(k, :);
                     % existed
                     if loc_list(node_n, ants_list(j).path, [1, 2])
                         continue
                     end
                     
                     % goal found
                     if all(node_n == goal)
                         ants_list(j).path = [ants_list(j).path; node_n, ants_list(j).cur_node];
                         ants_list(j).found_goal = true;
                         break;
                     end
                     
                     next_positions = [next_positions; node_n];
                     p_value = pheromone_edges(loc_list([ants_list(j).cur_node, node_n], pheromone_edges, [1, 4]));
                     prob_new = p_value ^ alpha * (1.0 / h(node_n, goal)) ^ beta;
                     next_probabilities = [next_probabilities, prob_new];
                     prob_sum = prob_sum + prob_new;
                 end

                if prob_sum == 0 || ants_list(j).found_goal
                    break
                end
                
                % roulette selection
                next_probabilities = next_probabilities ./ prob_sum;
                target = find(cumsum(next_probabilities) >= rand);
                ants_list(j).path = [ants_list(j).path; next_positions(target(1), :), ants_list(j).cur_node];
                ants_list(j).cur_node = next_positions(target(1), :);
                ants_list(j).steps = ants_list(j).steps + 1;
             end
         end
     
         % pheromone deterioration
         pheromone_edges(:, 5) = pheromone_edges(:, 5) * (1 - rho);
         % pheromone update based on successful ants
          bpl = inf;
          bp = [];
          for k=1:length(ants_list)
              if ants_list(k).found_goal
                  if ants_list(k).steps < bpl
                      bpl = ants_list(k).steps;
                      bp = ants_list(k).path;
                  end
                  c = Q / ants_list(k).steps;
                  for m=2:length(ants_list(k).path)
                      index = loc_list(ants_list(k).path(m, :), pheromone_edges, [1, 4]);
                      pheromone_edges(index, 5) = pheromone_edges(index, 5) + c;
                  end
              end
          end
          
          best_length_list = [best_length_list, bpl];
          if bpl <= min(best_length_list)
              best_path = bp;
          end
         
     end
     
    if ~isempty(best_path)
        best_path = flip(best_path, 1);
        path = extract_path(best_path, start);
        cost = sum(sqrt(sum(diff(path) .^ 2, 2)));
    end
end

%%
function h_val = h(node, goal)
% @breif: heuristic function (Manhattan distance)
    h_val = abs(node(1) - goal(1)) + abs(node(2) - goal(2));
end

function neighbors = getNeighbor(cur_node, map)
% @breif: Find neighbors of node.
    [x_range, y_range] = size(map);
    neighbors = [];
    % Reachable motion    
    motion = [-1, -1, 1.414; ...
                        0, -1, 1; ...
                        1, -1, 1.414; ...
                       -1, 0, 1; ...
                        1, 0, 1; ...
                       -1, 1, 1.414; ...
                        0, 1, 1; ...
                        1, 1, 1.414];
    motion_num = size(motion, 1);
    
    for i=1:motion_num
        nx = cur_node(1) + motion(i, 1);
        ny = cur_node(2) + motion(i, 2);
        if nx < 1 || ny < 1 || nx > x_range || ny > y_range ||  map(nx, ny) == 2
            continue
        end
        neighbors = [neighbors; nx, ny];
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
            if isequal(close(i, 1:2), close(index, 3:4))
                index = i;
                break
            end
        end
    end
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