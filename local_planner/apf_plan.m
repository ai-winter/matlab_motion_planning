function [pose, traj, flag] = apf_plan(start, goal, varargin)
%%
% @file: apf_plan.m
% @breif: Artificial Potential Field motion planning
% @paper: The Artificial Potential Field to Collision Avoidance
% @author: Winter
% @update: 2023.10.24

%%
    p = inputParser;           
    addParameter(p, 'path', "none");
    addParameter(p, 'map', "none");
    parse(p, varargin{:});

    if isstring(p.Results.path) || isstring(p.Results.map)
        exception = MException('MyErr:InvalidInput', 'parameter `path` or `map` must be set.');
        throw(exception);
    end
    
    % path
    path = flipud(p.Results.path);
    path_length = size(path, 1);
    plan_idx = 1;
    
    % map
    map = p.Results.map;

    % obstacle
    [m, ~] = size(map);
    obs_index = find(map==2);
    obstacle = [mod(obs_index - 1, m) + 1, fix((obs_index - 1) / m) + 1];
    
    % initial robotic state
    robot.x = start(1);
    robot.y = start(2);
    robot.theta = start(3);
    robot.v = 0;
    robot.w = 0;
    max_v = 0.4;
    
    % parameters
    zeta = 1.0;
    eta = 0.8;
    d_0 = 1.5;
    
    dt = 0.1;
    p_window = 0.5;
    p_precision = 0.5;
    o_precision = pi / 4;
    e_v_ = 0; i_v_ = 0;
    e_w_ = 0; i_w_ = 0;
    max_iter = 1000;
    
    % return value
    flag = false;
    pose = [];
    traj = [];
    
     iter = 0;
    % main loop
    while (1)
        iter = iter + 1;
        if (iter > max_iter)
            break;
        end

        % break until goal reached
        if (norm([robot.x, robot.y] - goal(:, 1:2)) < p_precision)
            flag = true;
            break;
        end
    
    % compute the tatget pose and force at the current step
    rep_force = getRepulsiveForce([robot.x, robot.y], obstacle, d_0);
    while (plan_idx <= path_length)
        tgt_pos = path(plan_idx, :);
        attr_force = getAttractiveForce([robot.x, robot.y], tgt_pos);
        net_force = zeta * attr_force + eta * rep_force;

        % in body frame
        b_x_d = path(plan_idx, 1) - robot.x;
        b_y_d = path(plan_idx, 2) - robot.y;

        if (norm([b_x_d, b_y_d]) > p_window)
            break;
        end
        plan_idx = plan_idx + 1;
    end
    
    new_v = [robot.v * cos(robot.theta), robot.v * sin(robot.theta)] + net_force;
    new_v = new_v  ./ norm(new_v);
    new_v = new_v .* max_v;
    
    theta_d = atan2(new_v(2), new_v(1));
    
    % calculate velocity command
    if (norm([robot.x, robot.y] - goal(:, 1:2)) < p_precision)
        if (abs(robot.theta - goal(3)) < o_precision)
            u = [0, 0];
        else
            [w, e_w_, i_w_] = angularController(robot, goal(3), dt, e_w_, i_w_);
            u = [0, w];
        end
    elseif (abs(theta_d - robot.theta) > pi / 2)
        [w, e_w_, i_w_] = angularController(robot, theta_d, dt, e_w_, i_w_);
        u = [0, w];
    else
        [v, e_v_, i_v_] = linearController(robot, norm(new_v), dt, e_v_, i_v_);
        [w, e_w_, i_w_] = angularController(robot, theta_d, dt, e_w_, i_w_);
        u = [v, w];
    end

    % input into robotic kinematic
    robot = f(robot, u, dt);
    pose = [pose; robot.x, robot.y, robot.theta];
    end
end

%%
function attr_force = getAttractiveForce(cur_pos, tgt_pos)
    attr_force = tgt_pos - cur_pos;
    if ~all(attr_force == 0)
        attr_force = attr_force ./ norm(attr_force);
    end
end

function rep_force = getRepulsiveForce(cur_pos, obstacle, d_0)
    D = dist(obstacle, cur_pos');
    rep_force = (1 ./ D - 1 / d_0) .* (1 ./ D) .^ 2 .* (cur_pos - obstacle);
    valid_mask = (1 ./ D - 1 / d_0) > 0;
    rep_force = sum(rep_force(valid_mask, :), 1);
    
    if ~all(rep_force == 0)
        rep_force = rep_force ./ norm(rep_force);
    end
end

function [v, e_v_, i_v_] = linearController(robot, v_d, dt, e_v_, i_v_)    
    e_v = v_d - robot.v;
    i_v_ = i_v_ + e_v * dt;
    d_v = (e_v - e_v_) / dt;
    e_v_ = e_v;

    k_v_p = 1.00;
    k_v_i = 0.00;
    k_v_d = 0.00;
    v_inc = k_v_p * e_v_ + k_v_i * i_v_ + k_v_d * d_v;

    v = robot.v + v_inc;
end

function [w, e_w_, i_w_] = angularController(robot, theta_d, dt, e_w_, i_w_)
    e_theta = theta_d - robot.theta;
    if (e_theta > pi)
        e_theta = e_theta - 2 * pi;
    elseif (e_theta < -pi)
        e_theta = e_theta + 2 * pi;
    end

    w_d = e_theta / dt / 10;
    e_w = w_d - robot.w;
    i_w_ = i_w_ + e_w * dt;
    d_w = (e_w - e_w_) / dt;
    e_w_ = e_w;

    k_w_p = 1.00;
    k_w_i = 0.00;
    k_w_d = 0.01;
    w_inc = k_w_p * e_w_ + k_w_i * i_w_ + k_w_d * d_w;

    w = robot.w + w_inc;
end

function robot = f(robot, u, dt)
%@breif: robotic kinematic
    F = [ 1 0 0 0 0
             0 1 0 0 0
             0 0 1 0 0
             0 0 0 0 0
             0 0 0 0 0];
 
    B = [dt * cos(robot.theta) 0
            dt * sin(robot.theta)  0
            0                                dt
            1                                 0
            0                                 1];
 
    x = [robot.x; robot.y; robot.theta; robot.v; robot.w];
    x_star = F * x + B * u';
    robot.x = x_star(1); robot.y = x_star(2); robot.theta = x_star(3);
    robot.v = x_star(4); robot.w = x_star(5);
end