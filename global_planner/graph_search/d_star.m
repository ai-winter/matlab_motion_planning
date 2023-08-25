function [path, flag, cost, EXPAND] = d_star(map, start, goal)
% @file: d_star.m
% @breif: D* motion planning
% @paper: Optimal and Efficient Path Planning for Partially-Known Environments.
% @author: Zhanyu Guo
% @update: 2023.7.15

%
%   ========= MAP =========
%   [x, y, t, h, k, px, py]
%   =======================
%   NEW = 0, OPEN = 1, CLOSED = 2
%

% initialize
path = [];
flag = false;
cost = 0;
EXPAND = [];

siz = size(map);
persistent MAP; % static variable
if isempty(MAP) % first plan
    MAP = zeros(siz(1) * siz(2), 7);
    for y = 1:siz(2)
        for x = 1:siz(1)
            cur_ind = sub2ind(siz, x, y);
            MAP(cur_ind, 1) = x; % x
            MAP(cur_ind, 2) = y; % y
            MAP(cur_ind, 3) = 0; % tag = NEW
            MAP(cur_ind, 4) = Inf; % h
            MAP(cur_ind, 5) = Inf; % key
        end
    end
    start_ind = sub2ind(siz, start(1), start(2));
    goal_ind = sub2ind(siz, goal(1), goal(2));
    MAP = insert(MAP, goal_ind, 0);

    while 1
        [MAP, EXPAND, k_min] = processState(MAP, EXPAND, siz, map);
        if k_min == -1
            return
        end

        if MAP(start_ind, 3) == 2
            flag = true;
            break
        end
    end
    % extract path
    [path, cost] = extract_path(MAP, siz, start_ind, map);
else
    start_ind = sub2ind(siz, start(1), start(2));
    cur_ind = start_ind;

    while 1
        if isequal(MAP(cur_ind, 6:7), [0, 0])
            break
        end
        par_ind = sub2ind(siz, MAP(cur_ind, 6), MAP(cur_ind, 7));
        if isCollision(MAP(cur_ind, :), MAP(par_ind, :), map)
            [MAP, EXPAND] = modify(cur_ind, par_ind, MAP, siz, map, EXPAND);
            continue
        end
        cur_ind = par_ind;
    end
    [path, cost] = extract_path(MAP, siz, start_ind, map);
end

i = 1;
expand_num = size(EXPAND, 1);
while i <= expand_num
    if map(EXPAND(i, 1), EXPAND(i, 2)) == 2
        EXPAND(i, :) = [];
        expand_num = expand_num - 1;
        continue
    end
    i = i + 1;
end

end

%%
function MAP = insert(MAP, ind, h_new)
%
%   ========= MAP =========
%   [x, y, t, h, k, px, py]
%   =======================
%   NEW = 0, OPEN = 1, CLOSED = 2
%
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

function [MAP, EXPAND, k_min] = processState(MAP, EXPAND, siz, map)
% get open list
OPEN = MAP(MAP(:, 3) == 1, :);

if isempty(OPEN)
    k_min = -1;
    return
end

% get node with min k in open
[k_old, open_ind] = min(OPEN(:, 5));
cur_ind = sub2ind(siz, OPEN(open_ind, 1), OPEN(open_ind, 2));

% set to closed
MAP(cur_ind, 3) = 2;

% add to expand
if ~loc_list(MAP(cur_ind, :), EXPAND, [1, 2])
    EXPAND = [EXPAND; MAP(cur_ind, 1:2)];
end

% get neighbors
motion = [-1, -1; ...
    0, -1; ...
    1, -1; ...
    -1, 0; ...
    1, 0; ...
    -1, 1; ...
    0, 1; ...
    1, 1];

motion_num = size(motion, 1);
neighbors = zeros(motion_num, 4);

for i = 1:motion_num
    neighbors(i, 1) = MAP(cur_ind, 1) + motion(i, 1);
    neighbors(i, 2) = MAP(cur_ind, 2) + motion(i, 2);

    neb_ind = sub2ind(siz, neighbors(i, 1), neighbors(i, 2));
    neighbors(i, 3) = neb_ind;
    neighbors(i, 4) = getCost(MAP(cur_ind, :), MAP(neb_ind, :), map);
end

if k_old < MAP(cur_ind, 4)
    for i = 1:motion_num
        neb_ind = neighbors(i, 3);
        if MAP(neb_ind, 3) ~= 0 ...
                && MAP(neb_ind, 4) <= k_old ...
                && MAP(cur_ind, 4) > MAP(neb_ind, 4) + neighbors(i, 4)
            MAP(cur_ind, 6) = MAP(neb_ind, 1);
            MAP(cur_ind, 7) = MAP(neb_ind, 2);
            MAP(cur_ind, 4) = MAP(neb_ind, 4) + neighbors(i, 4);
        end
    end
end

if k_old == MAP(cur_ind, 4)
    for i = 1:motion_num
        neb_ind = neighbors(i, 3);
        if MAP(neb_ind, 3) == 0 ...
                || ((MAP(neb_ind, 6) == MAP(cur_ind, 1) && MAP(neb_ind, 7) == MAP(cur_ind, 2)) && MAP(neb_ind, 4) ~= MAP(cur_ind, 4) + neighbors(i, 4)) ...
                || ((MAP(neb_ind, 6) ~= MAP(cur_ind, 1) || MAP(neb_ind, 7) ~= MAP(cur_ind, 2)) && MAP(neb_ind, 4) > MAP(cur_ind, 4) + neighbors(i, 4))
            MAP(neb_ind, 6) = MAP(cur_ind, 1);
            MAP(neb_ind, 7) = MAP(cur_ind, 2);
            MAP = insert(MAP, neb_ind, MAP(cur_ind, 4) + neighbors(i, 4));
        end
    end
else
    for i = 1:motion_num
        neb_ind = neighbors(i, 3);
        if MAP(neb_ind, 3) == 0 ...
                || ((MAP(neb_ind, 6) == MAP(cur_ind, 1) && MAP(neb_ind, 7) == MAP(cur_ind, 2)) && MAP(neb_ind, 4) ~= MAP(cur_ind, 4) + neighbors(i, 4))
            MAP(neb_ind, 6) = MAP(cur_ind, 1);
            MAP(neb_ind, 7) = MAP(cur_ind, 2);
            MAP = insert(MAP, neb_ind, MAP(cur_ind, 4) + neighbors(i, 4));
        elseif (MAP(neb_ind, 6) ~= MAP(cur_ind, 1) || MAP(neb_ind, 7) ~= MAP(cur_ind, 2)) ...
                && MAP(neb_ind, 4) > MAP(cur_ind, 4) + neighbors(i, 4)
            MAP = insert(MAP, cur_ind, MAP(cur_ind, 4));
        elseif (MAP(neb_ind, 6) ~= MAP(cur_ind, 1) || MAP(neb_ind, 7) ~= MAP(cur_ind, 2)) ...
                && MAP(cur_ind, 4) > MAP(neb_ind, 4) + neighbors(i, 4) ...
                && MAP(neb_ind, 3) == 2 ...
                && MAP(neb_ind, 4) > k_old
            MAP = insert(MAP, neb_ind, MAP(neb_ind, 4));
        end
    end
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

function flag = isCollision(node1, node2, map)
flag = false;

if map(node1(1), node1(2)) == 2 || map(node2(1), node2(2)) == 2
    flag = true;
    return
end

if node1(1) ~= node2(1) && node1(2) ~= node2(2)
    if node2(1) - node1(1) == node1(2) - node2(2)
        s1 = [min(node1(1), node2(1)), min(node1(2), node2(2))];
        s2 = [max(node1(1), node2(1)), max(node1(2), node2(2))];
    else
        s1 = [min(node1(1), node2(1)), max(node1(2), node2(2))];
        s2 = [max(node1(1), node2(1)), min(node1(2), node2(2))];
    end

    if map(s1(1), s1(2)) == 2 || map(s2(1), s2(2)) == 2
        flag = true;
    end
end

end

function cost = getCost(node1, node2, map)
if isCollision(node1, node2, map)
    cost = Inf;
else
    if abs(node1(1) - node2(1)) + abs(node1(2) - node2(2)) > 1
        cost = 1.414;
    else
        cost = 1;
    end
end

end

function [path, cost] = extract_path(MAP, siz, start_ind, map)
% @breif: Extract the path based on the CLOSED set.
path = [];
cost = 0;

cur_ind = start_ind;
path = [path; MAP(cur_ind, 1:2)];
while 1
    if isequal(MAP(cur_ind, 6:7), [0, 0])
        break
    end

    par_ind = sub2ind(siz, MAP(cur_ind, 6), MAP(cur_ind, 7));
    cost = cost + getCost(MAP(cur_ind, :), MAP(par_ind, :), map);

    cur_ind = par_ind;
    path = [path; MAP(cur_ind, 1:2)];
end

end

function [MAP, EXPAND] = modify(cur_ind, par_ind, MAP, siz, map, EXPAND)
if MAP(cur_ind, 3) == 2
    MAP = insert(MAP, cur_ind, MAP(cur_ind, 4));
end

if MAP(par_ind, 3) == 2
    MAP = insert(MAP, par_ind, MAP(par_ind, 4));
end

while 1
    [MAP, EXPAND, k_min] = processState(MAP, EXPAND, siz, map);
    if k_min >= MAP(cur_ind, 4)
        break
    end
end

end
