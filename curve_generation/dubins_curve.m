function path = dubins_curve(points)
% @file: dubins_curve.m
% @breif: Dubins curve generation
% @author: Winter
% @update: 2023.2.7

    % parameters
    param.step = 0.1;
    param.max_curv = 0.25;
    
    [num_pts, ~] = size(points);
    
    % generate curve
    path = [];
    for i=1:num_pts - 1
       [x_list, y_list, ~] = generation(...
            [points(i, 1), points(i, 2), points(i, 3)], ...
            [points(i + 1, 1), points(i + 1, 2), points(i + 1, 3)], ...
            param ...
       );
       path_i = [x_list', y_list'];
       path = [path; path_i];
    end
end

%%
function theta_mod = mod2pi(theta)
		theta_mod = theta - 2.0 * pi * floor(theta / pi / 2.0);
end

function theta_truncate = pi2pi(theta)
    while theta > pi
        theta = theta - 2.0 * pi;
    end
    while theta < -pi
        theta = theta + 2.0 * pi;
    end
    theta_truncate = theta;
end

function [segs, mode] = LSL(alpha, beta, dist)
    %{
    Left-Straight-Left generation mode.

    Parameters
    ----------
    alpha: float
        Initial pose of (0, 0, alpha)
    beta/dist: float
        Goal pose of (dist, 0, beta)

    Return
    ----------
    t/s/p: float
        Moving lenght of segments
    mode: list
        Motion mode
    %}  
    mode = ["L", "S", "L"];
    
    p_lsl = 2 + dist * dist - 2 * cos(alpha - beta) + 2 * dist * (sin(alpha) - sin(beta));
    if p_lsl < 0
        segs = [];
    else
        p_lsl = sqrt(p_lsl);
        t_lsl = mod2pi(-alpha + atan2(cos(beta) - cos(alpha), dist + sin(alpha) - sin(beta)));
        q_lsl = mod2pi(beta - atan2(cos(beta) - cos(alpha), dist + sin(alpha) - sin(beta)));
        segs = [t_lsl, p_lsl, q_lsl];
    end 
end

function [segs, mode] = RSR(alpha, beta, dist)
    %{
    Right-Straight-Right generation mode.

    Parameters
    ----------
    alpha: float
        Initial pose of (0, 0, alpha)
    beta/dist: float
        Goal pose of (dist, 0, beta)

    Return
    ----------
    t/s/p: float
        Moving lenght of segments
    mode: list
        Motion mode
    %}
    mode = ["R", "S", "R"];

    p_rsr = 2 + dist * dist - 2 * cos(alpha - beta) + 2 * dist * (sin(beta) - sin(alpha));
    if p_rsr < 0
        segs = [];
    else
        p_rsr = sqrt(p_rsr);
        t_rsr = mod2pi(alpha - atan2(cos(alpha) - cos(beta), dist - sin(alpha) + sin(beta)));
        q_rsr = mod2pi(-beta + atan2(cos(alpha) - cos(beta), dist - sin(alpha) + sin(beta)));
        segs = [t_rsr, p_rsr, q_rsr];
    end
end

function [segs, mode] = LSR(alpha, beta, dist)
    %{
    Left-Straight-Right generation mode.

    Parameters
    ----------
    alpha: float
        Initial pose of (0, 0, alpha)
    beta/dist: float
        Goal pose of (dist, 0, beta)

    Return
    ----------
    t/s/p: float
        Moving lenght of segments
    mode: list
        Motion mode
    %}
    mode = ["L", "S", "R"];
    
    p_lsr = -2 + dist * dist + 2 * cos(alpha - beta) + 2 * dist * (sin(alpha) + sin(beta));
    if p_lsr < 0
        segs = [];
    else
        p_lsr = sqrt(p_lsr);
        t_lsr = mod2pi(-alpha + atan2(-cos(alpha) - cos(beta), dist + sin(alpha) + sin(beta)) - atan2(-2.0, p_lsr));
        q_lsr = mod2pi(-beta + atan2(-cos(alpha) - cos(beta), dist + sin(alpha) + sin(beta)) - atan2(-2.0, p_lsr));
        segs = [t_lsr, p_lsr, q_lsr];
    end
end

function [segs, mode] = RSL(alpha, beta, dist)
    %{
    Right-Straight-Left generation mode.

    Parameters
    ----------
    alpha: float
        Initial pose of (0, 0, alpha)
    beta/dist: float
        Goal pose of (dist, 0, beta)

    Return
    ----------
    t/s/p: float
        Moving lenght of segments
    mode: list
        Motion mode
    %}
    mode = ["R", "S", "L"];

    p_rsl = -2 + dist * dist + 2 * cos(alpha - beta) - 2 * dist * (sin(alpha) + sin(beta));
    if p_rsl < 0
        segs = [];
    else
        p_rsl = sqrt(p_rsl);
        t_rsl = mod2pi(alpha - atan2(cos(alpha) + cos(beta), dist - sin(alpha) - sin(beta)) + atan2(2.0, p_rsl));
        q_rsl = mod2pi(beta - atan2(cos(alpha) + cos(beta), dist - sin(alpha) - sin(beta)) + atan2(2.0, p_rsl));
        segs = [t_rsl, p_rsl, q_rsl];
    end
end

function [segs, mode] = RLR(alpha, beta, dist)
    %{
    Right-Left-Right generation mode.

    Parameters
    ----------
    alpha: float
        Initial pose of (0, 0, alpha)
    beta/dist: float
        Goal pose of (dist, 0, beta)

    Return
    ----------
    t/s/p: float
        Moving lenght of segments
    mode: list
        Motion mode
    %}
	mode = ["R", "L", "R"];

    p_rlr = (6.0 - dist * dist + 2.0 * cos(alpha - beta) + 2.0 * dist * (sin(alpha) - sin(beta))) / 8.0;
    if abs(p_rlr) > 1.0
        segs = [];
    else
        p_rlr = mod2pi(2 * pi - acos(p_rlr));
        t_rlr = mod2pi(alpha - atan2(cos(alpha) - cos(beta), dist - sin(alpha) + sin(beta)) + p_rlr / 2.0);
        q_rlr = mod2pi(alpha - beta - t_rlr + p_rlr);
        segs = [t_rlr, p_rlr, q_rlr];
    end
end

function [segs, mode] = LRL(alpha, beta, dist)
    %{
    Left-Right-Left generation mode.

    Parameters
    ----------
    alpha: float
        Initial pose of (0, 0, alpha)
    beta/dist: float
        Goal pose of (dist, 0, beta)

    Return
    ----------
    t/s/p: float
        Moving lenght of segments
    mode: list
        Motion mode
    %}
    mode = ["L", "R", "L"];

    p_lrl = (6.0 - dist * dist + 2.0 * cos(alpha - beta) + 2.0 * dist * (sin(alpha) - sin(beta))) / 8.0;
    if abs(p_lrl) > 1.0
        segs = [];
    else
        p_lrl = mod2pi(2 * pi - acos(p_lrl));
        t_lrl = mod2pi(-alpha + atan2(-cos(alpha) + cos(beta), dist + sin(alpha) - sin(beta)) + p_lrl / 2.0);
        q_lrl = mod2pi(beta - alpha - t_lrl + p_lrl);
        segs = [t_lrl, p_lrl, q_lrl];
    end
end

function new_pt = interpolate(mode, length, init_pose, param)
    %{
    Planning path interpolation.

    Parameters
    ----------
    mode: str
        motion, e.g., L, S, R
    length: float
        Single step motion path length
    init_pose: tuple
        Initial pose (x, y, yaw)

    Return
    ----------
    new_pose: tuple
        New pose (new_x, new_y, new_yaw) after moving
    %}
    x = init_pose(1); y = init_pose(2); yaw = init_pose(3);

    if mode == "S"
        new_x   = x + length / param.max_curv * cos(yaw);
        new_y   = y + length / param.max_curv * sin(yaw);
        new_yaw = yaw;
    elseif mode == "L"
        new_x   = x + (sin(yaw + length) - sin(yaw)) / param.max_curv;
        new_y   = y - (cos(yaw + length) - cos(yaw)) / param.max_curv;
        new_yaw = yaw + length;
    elseif mode == "R"
        new_x   = x - (sin(yaw - length) - sin(yaw)) / param.max_curv;
        new_y   = y + (cos(yaw - length) - cos(yaw)) / param.max_curv;
        new_yaw = yaw - length;
    else
        new_x = 0; new_y = 0; new_yaw = 0;
    end
    new_pt = [new_x, new_y, new_yaw];
end

function [x_list, y_list, yaw_list] = generation(start_pose, goal_pose, param)
    %{
    Generate the Dubins Curve.

    Parameters
    ----------
    start_pose: tuple
        Initial pose (x, y, yaw)
    goal_pose: tuple
        Target pose (x, y, yaw)

    Return
    ----------
    x_list/y_list/yaw_list: list
        Trajectory
    %}
    sx = start_pose(1); sy = start_pose(2); syaw = start_pose(3);
    gx = goal_pose(1); gy = goal_pose(2); gyaw = goal_pose(3);

    % coordinate transformation
    gx = gx - sx; gy = gy - sy;
    theta = mod2pi(atan2(gy, gx));
    dist = hypot(gx, gy) * param.max_curv;
    alpha = mod2pi(syaw - theta);
    beta = mod2pi(gyaw - theta);

    % select the best motion
    planners = ["LSL", "RSR", "LSR", "RSL", "RLR", "LRL"];
    best_cost = inf;
    best_segs = [];
    best_mode = [];

    for i=1:length(planners)
        planner = str2func(planners(i));
        [segs, mode] = planner(alpha, beta, dist);
        if isempty(segs)
            continue
        end
        cost = (abs(segs(1)) + abs(segs(2)) + abs(segs(3)));
        if best_cost > cost
            best_segs = segs;
            best_mode = mode;
            best_cost = cost;
        end
    end
            
    % interpolation
    points_num = floor(sum(best_segs) / param.step) + length(best_segs) + 3;
    x_list = zeros(points_num);
    y_list = zeros(points_num);
    yaw_list = alpha * ones(points_num);

    i = 1;
    for j = 1:length(best_segs)
        m = best_mode(j);
        seg_length = best_segs(j);
        
        % path increment
         if seg_length > 0.0
            d_length = param.step;
         else
            d_length = -param.step;
         end
        x = x_list(i); y = y_list(i); yaw = yaw_list(i);
        
        % current path length
        l = d_length;
        while abs(l) <= abs(seg_length)
            i = i + 1;
            new_pt = interpolate(m, l, [x, y, yaw], param);
            x_list(i) = new_pt(1); y_list(i) = new_pt(2); yaw_list(i) = new_pt(3);
            l = l + d_length;
        end
        i = i + 1;
        new_pt = interpolate(m, seg_length, [x, y, yaw], param);
        x_list(i) = new_pt(1); y_list(i) = new_pt(2); yaw_list(i) = new_pt(3);
    end

    % remove unused data
    while length(x_list) >= 1 && x_list(end) == 0.0
        x_list(end) = [];
        y_list(end) = [];
        yaw_list(end) = [];
    end

    % coordinate transformation
    rot = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    converted_xy = rot * [x_list; y_list];
    x_list = converted_xy(1, :) + sx;
    y_list = converted_xy(2, :) + sy;
    for j=1:length(yaw_list)
        yaw_list(j) = pi2pi(yaw_list(j) + theta);
    end
end