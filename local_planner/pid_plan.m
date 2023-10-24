function [pose, traj, flag] = pid_plan(start, goal, varargin)
%%
% @file: pid_plan.m
% @breif: PID motion planning
% @paper: The PID to path tracking
% @author: Winter, Gzy
% @update: 2023.1.30

%%
    p = inputParser;           
    addParameter(p, 'path', "none");
    addParameter(p, 'map', "none");
    parse(p, varargin{:});

    if isstring(p.Results.path)
        exception = MException('MyErr:InvalidInput', 'parameter `path` must be set, using global planner.');
        throw(exception);
    end
    
    % path
    path = p.Results.path;
     
    % return value
    flag = false;
    pose = [];
    traj = [];

    % reverse path
    path = flipud(path);
    path_length = size(path, 1);
    plan_idx = 1;

    % params
    dt = 0.1;
    p_window = 0.5;
    o_window = pi / 2;
    p_precision = 0.5;
    o_precision = pi / 4;
    max_iter = 1000;

    % initial robotic state
    robot.x = start(1);
    robot.y = start(2);
    robot.theta = start(3);
    robot.v = 0;
    robot.w = 0;

    e_v_ = 0; i_v_ = 0;
    e_w_ = 0; i_w_ = 0;

    k_theta = 0.5;

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

        % find next tracking point
        while (plan_idx <= path_length)
            theta_err = atan2(path(plan_idx, 2) - robot.y, ...
                              path(plan_idx, 1) - robot.x);

            next_plan_idx = plan_idx + 1;
            if (next_plan_idx <= path_length)
                theta_trj = atan2(path(next_plan_idx, 2) - path(plan_idx, 2), ...
                                  path(next_plan_idx, 1) - path(plan_idx, 1));
            end

            if (abs(theta_err - theta_trj) > pi)
                if (theta_err > theta_trj)
                    theta_trj = theta_trj + 2 * pi;
                else
                    theta_err = theta_err + 2 * pi;
                end
            end
            theta_d = k_theta * theta_err + (1 - k_theta) * theta_trj;
            % in body frame
            b_x_d = path(plan_idx, 1) - robot.x;
            b_y_d = path(plan_idx, 2) - robot.y;
            b_theta_d = theta_d - robot.theta;

            if (norm([b_x_d, b_y_d]) > p_window || abs(b_theta_d) > o_window)
                break;
            end
            plan_idx = plan_idx + 1;
        end
        
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
            [v, e_v_, i_v_] = linearController(robot, b_x_d, b_y_d, dt, e_v_, i_v_);
            [w, e_w_, i_w_] = angularController(robot, theta_d, dt, e_w_, i_w_);
            u = [v, w];
        end

        % input into robotic kinematic
        robot = f(robot, u, dt);
        pose = [pose; robot.x, robot.y, robot.theta];
    end

end

function [v, e_v_, i_v_] = linearController(robot, b_x_d, b_y_d, dt, e_v_, i_v_)
    v_d = norm([b_x_d, b_y_d]) / dt / 10;
    
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
%
% @breif: robotic kinematic
% @param robot: current state of robot
% @param u: input, v and w
% @param dt: time interval
% 
    F = [
        1 0 0 0 0
        0 1 0 0 0
        0 0 1 0 0
        0 0 0 0 0
        0 0 0 0 0];
 
    B = [
        dt * cos(robot.theta) 0
        dt * sin(robot.theta) 0
        0                     dt
        1                     0
        0                     1];
 
    x = [robot.x; robot.y; robot.theta; robot.v; robot.w];
    x_star = F * x + B * u';

    robot.x = x_star(1);
    robot.y = x_star(2);
    robot.theta = x_star(3);
    robot.v = x_star(4);
    robot.w = x_star(5);
end
