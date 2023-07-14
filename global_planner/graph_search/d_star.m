function [path, flag, cost, EXPAND] = d_star(map, start, goal)
% @file: d_star.m
% @breif: D* motion planning
% @paper: Optimal and Efficient Path Planning for Partially-Known Environments.
% @author: Zhanyu Guo
% @update: 2023.7.13

%
%   ========= MAP =========
%   [x, y, t, h, k, px, py]
%   =======================
%   NEW = 0, OPEN = 1, CLOSED = 2
%

% initialize
EXPAND = [];

siz = size(map);
MAP = zeros(siz(1) * siz(2), 7);
for y = 1:cols
    for x = 1:rows
        ind = sub2ind(siz, x, y);
        MAP(ind, 1) = x;    % x
        MAP(ind, 2) = y;    % y
        MAP(ind, 3) = 0;    % t
        MAP(ind, 4) = Inf;  % h
        MAP(ind, 5) = Inf;  % k
    end
end

MAP = insert(goal, 0, MAP, siz);

start_ind = sub2ind(siz, start(1), start(2));

while 1
    [MAP, k_min] = processState(MAP, siz);
    if k_min == -1 || MAP(start_ind, 3) == 2
        break
    end
end


%%
function MAP = insert(node, h_new, MAP, siz)
%
%   ========= MAP =========
%   [x, y, t, h, k, px, py]
%   =======================
%   NEW = 0, OPEN = 1, CLOSED = 2
%
ind = sub2ind(siz, node(1), node(2));
if MAP(ind, 3) == 0
    MAP(ind, 5) = h_new;
elseif MAP(ind, 3) == 1
    MAP(ind, 5) = min(MAP(ind, 5), h_new);
elseif MAP(ind, 3) == 2
    MAP(ind, 5) = min(MAP(ind, 4), h_new);
end

MAP(ind, 4) = h_new;
MAP(ind, 3) = 1;
end

function [MAP, k_min] = processState(MAP, siz)
% get open
OPEN = MAP(MAP(:, 3) == 1, :);
if isempty(OPEN)
    k_min = -1;
    return
end

% get node with min k in open
[k_old, open_ind] = min(OPEN(:, 5));
map_ind = sub2ind(siz, OPEN(open_ind, 1), OPEN(open_ind, 2));

% set to closed
MAP(map_ind, 3) = 2;

% get neighbors
% TODO

if k_old < MAP(map_ind, 4)
    
end

if k_old == MAP(map_ind, 4)

else

end

% get open
OPEN = MAP(MAP(:, 3) == 1, :);
if isempty(OPEN)
    k_min = -1;
    return
end

% get node with min k in open
[k_min, ~] = min(OPEN(:, 5));

end
