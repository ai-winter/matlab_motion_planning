function [pose, traj, flag] = rpp_plan(start, goal, varargin)
%%
% @file: rpp_plan.m
% @breif: Regulated Pure Pursuit (RPP) motion planning
% @paper: Regulated Pure Pursuit for Robot Path Tracking
% @author: Winter
% @update: 2023.2.7
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
    
    % map
    map = p.Results.map;
    
    % initial robotic state
    robot.x = start(1);
    robot.y = start(2);
    robot.theta = start(3);
    robot.v = 0;
    robot.w = 0;
    
    % RPP parameters
    param.regulated_radius_min = 0.9;
    param.scaling_dist = 0.6;
    param.scaling_gain = 1.0;
    
    % common parameters
    param.dt = 0.1;
    param.max_iteration = 1500;
    param.goal_dist_tol = 0.5;
    param.rotate_tol = 0.5;
    param.lookahead_time = 1.0;
    param.min_lookahead_dist = 1.5;
    param.max_lookahead_dist = 2.5;
    param.max_v_inc = 0.5;
    param.max_v = 0.5;
    param.min_v = 0.0;
    param.max_w_inc = pi / 2;
    param.max_w = pi / 2;
    param.min_w = 0.0;
   
    % return value
    flag = false;
    pose = [];
    traj = [];
    
    % main loop
    iter = 0;
    while iter < param.max_iteration
        iter = iter + 1;
        
        % break until goal reached
        if shouldRotateToGoal([robot.x, robot.y], goal, param)
            flag = true;
            break;
        end
        
        % get the particular point on the path at the lookahead distance
        [lookahead_pt, ~, ~] = getLookaheadPoint(robot, path, param);
        
        % get the tracking curvature with goalahead point
        lookahead_k = 2 * sin( ...
            atan2(lookahead_pt(2) - robot.y, lookahead_pt(1) - robot.x) - robot.theta ...
        ) / getLookaheadDistance(robot, param);

        % calculate velocity command
        e_theta = regularizeAngle(robot.theta - goal(3)) / 10;
        if shouldRotateToGoal([robot.x, robot.y], goal, param)
            if ~shouldRotateToPath(abs(e_theta), 0.0, param)
                u = [0, 0];
            else
                u = [0, angularRegularization(robot, e_theta / param.dt, param)];
            end
        else
            e_theta = regularizeAngle( ...
                atan2(lookahead_pt(2) - robot.y, lookahead_pt(1) - robot.x) - robot.theta ...
            ) / 10;
            if shouldRotateToPath(abs(e_theta), pi / 4, param)
                u = [0, angularRegularization(robot, e_theta / param.dt, param)];
            else
                % apply constraints
                curv_vel = applyCurvatureConstraint(param.max_v, lookahead_k, param);
                cost_vel = applyObstacleConstraint(param.max_v, map, robot, param);
                v_d = min(curv_vel, cost_vel);
                u = [
                        linearRegularization(robot, v_d, param), ...
                        angularRegularization(robot, v_d * lookahead_k, param) ...
                ];
            end
        end

        % input into robotic kinematic
        robot = f(robot, u, param.dt);
        pose = [pose; robot.x, robot.y, robot.theta];
    end
end

%%
function robot = f(robot, u, dt)
    % robotic kinematic
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

function theta = regularizeAngle(angle)
    theta = angle - 2.0 * pi * floor((angle + pi) / (2.0 * pi));
end

function flag = shouldRotateToGoal(cur, goal, param)
    %{
    Whether to reach the target pose through rotation operation

    Parameters
    ----------
    cur: tuple
        current pose of robot
    goal: tuple
        goal pose of robot

    Return
    ----------
    flag: bool
        true if robot should perform rotation
    %}
    flag = hypot(cur(1) - goal(1), cur(2) - goal(2)) < param.goal_dist_tol;
end

function flag = shouldRotateToPath(angle_to_path, tol, param)
    %{
    Whether to correct the tracking path with rotation operation

    Parameters
    ----------
    angle_to_path: float 
        the angle deviation
    tol: float[None]
        the angle deviation tolerence

    Return
    ----------
    flag: bool
        true if robot should perform rotation
    %}
    if tol == 0.0
        flag = angle_to_path > param.rotate_tol;
    else
        flag = angle_to_path > tol;
    end
end

function w = angularRegularization(robot, w_d, param)
    %{
    Angular velocity regularization

    Parameters
    ----------
    w_d: float
        reference angular velocity input

    Return
    ----------
    w: float
        control angular velocity output
    %}
    w_inc = w_d - robot.w;
    if abs(w_inc) > param.max_w_inc
        w_inc =param.max_w_inc * sign(w_inc);
    end
    w = robot.w + w_inc;

    if abs(w) > param.max_w
        w = param.max_w * sign(w);
    end
    if abs(w) < param.min_w
        w = param.min_w * sign(w)  ;
    end
end

function v = linearRegularization(robot, v_d, param)
    %{
    Linear velocity regularization

    Parameters
    ----------
    v_d: float
        reference velocity input

    Return
    ----------
    v: float
        control velocity output
    %}
    v_inc = v_d - robot.v;
    if abs(v_inc) > param.max_v_inc
        v_inc = param.max_v_inc * sign(v_inc);
    end
    v = robot.v + v_inc;

    if abs(v) > param.max_v
        v = param.max_v * sign(v);
    end
    if abs(v) < param.min_v
        v = param.min_v * sign(v);
    end
end

function d = getLookaheadDistance(robot, param)
    d = robot.v * param.lookahead_time;
    if d < param.min_lookahead_dist
        d = param.min_lookahead_dist;
    end
    if d > param.max_lookahead_dist
        d = param.max_lookahead_dist;
    end
end

function [pt, theta, kappa] = getLookaheadPoint(robot, path, param)
    %{
    Find the point on the path that is exactly the lookahead distance away from the robot

    Return
    ----------
    lookahead_pt: tuple
        lookahead point
    theta: float
        the angle on trajectory
    kappa: float
        the curvature on trajectory
    %}

    % Find the first pose which is at a distance greater than the lookahead distance
    dist_to_robot = [];
    [pts_num, ~] = size(path);
    for i=1:pts_num
        dist_to_robot(end + 1) = hypot(path(i, 1) - robot.x, path(i, 2) - robot.y);
    end
    [~, idx_closest] = min(dist_to_robot);
    idx_goal = pts_num - 1; idx_prev = idx_goal - 1;
    
    lookahead_dist = getLookaheadDistance(robot, param);
    for i=idx_closest:pts_num
        if hypot(path(i, 1) - robot.x, path(i, 2) - robot.y) >= lookahead_dist
            idx_goal = i;
            break;
        end
    end

    if idx_goal == pts_num - 1
        % If the no pose is not far enough, take the last pose
        pt = [path(idx_goal, 1), path(idx_goal, 2)];
    else
        if idx_goal == 1
            idx_goal = idx_goal + 1;
        end
        % find the point on the line segment between the two poses
        % that is exactly the lookahead distance away from the robot pose (the origin)
        % This can be found with a closed form for the intersection of a segment and a circle
        idx_prev = idx_goal - 1;
        px = path(idx_prev, 1); py = path(idx_prev, 2);
        gx = path(idx_goal, 1); gy = path(idx_goal, 2);
        
        % transform to the robot frame so that the circle centers at (0,0)
        prev_p = [px - robot.x, py - robot.y];
        goal_p = [gx - robot.x, gy - robot.y];
        i_points = circleSegmentIntersection(prev_p, goal_p, lookahead_dist);
        pt = [i_points(1, 1) + robot.x, i_points(1, 2) + robot.y];
    end

    % calculate the angle on trajectory
    theta = atan2(path(idx_goal, 2) - path(idx_prev, 2), path(idx_goal, 1) - path(idx_prev, 1));

    % calculate the curvature on trajectory
    if idx_goal == 2
        idx_goal = idx_goal + 1;
    end
    idx_prev = idx_goal - 1;
    idx_pprev = idx_prev - 1;
    a = hypot(path(idx_prev, 1) - path(idx_goal, 1), path(idx_prev, 2) - path(idx_goal, 2));
    b = hypot(path(idx_pprev, 1) - path(idx_goal, 1), path(idx_pprev, 2) - path(idx_goal, 2));
    c = hypot(path(idx_pprev, 1) - path(idx_prev, 1), path(idx_pprev, 2) - path(idx_prev, 2));
    cosB = (a * a + c * c - b * b) / (2 * a * c);
    sinB = sin(acos(cosB));
    cross = (path(idx_prev, 1) - path(idx_pprev, 1)) * ...
            (path(idx_goal, 2) - path(idx_pprev, 2)) - ...
            (path(idx_prev, 2) - path(idx_pprev, 2)) * ...
            (path(idx_goal, 1) - path(idx_pprev, 1));
    kappa = 2 * sinB / b *sign(cross);
end

function i_points = circleSegmentIntersection(p1, p2, r)
    x1 = p1(1); x2 = p2(1);
    y1 = p1(2); y2 = p2(2);

    dx = x2 - x1; dy = y2 - y1;
    dr2 = dx * dx + dy * dy;
    D = x1 * y2 - x2 * y1;

    % the first element is the point within segment
    d1 = x1 * x1 + y1 * y1;
    d2 = x2 * x2 + y2 * y2;
    dd = d2 - d1;

    delta = sqrt(r * r * dr2 - D * D);
    if delta >= 0
        if delta == 0
            i_points = [D * dy / dr2, -D * dx / dr2];
        else
            i_points = [
                 (D * dy + sign(dd) * dx * delta) / dr2, ...
                (-D * dx + sign(dd) * dy * delta) / dr2; ...
                 (D * dy - sign(dd) * dx * delta) / dr2, ...
                 (-D * dx - sign(dd) * dy * delta) / dr2
            ];
        end
    else
        i_points = [];
    end
end

function v = applyCurvatureConstraint(raw_linear_vel, curvature, param)
    %{
    Applying curvature constraints to regularize the speed of robot turning.

    Parameters
    ----------
    raw_linear_vel: float
        the raw linear velocity of robot
    curvature: float
        the tracking curvature

    Return
    ----------
    reg_vel: float
        the regulated velocity
    %}
    radius = abs(1.0 / curvature);
    if radius < param.regulated_radius_min
        v = raw_linear_vel * (radius / param.regulated_radius_min);
    else
        v = raw_linear_vel;
    end
end

function v = applyObstacleConstraint(raw_linear_vel, map, robot, param)
    %{
    Applying obstacle constraints to regularize the speed of robot approaching obstacles.

    Parameters
    ----------
    raw_linear_vel: float
        the raw linear velocity of robot

    Return
    ----------
    reg_vel: float
        the regulated velocity
    %}
    [obs_x, obs_y] = find(map == 2);
    obstacles = [obs_x, obs_y];
    robot_to_obs = [robot.x, robot.y] - obstacles;
    obs_dist = min(sqrt(sum(robot_to_obs .^ 2, 2)));
    if obs_dist < param.scaling_dist
        v = raw_linear_vel * param.scaling_gain * obs_dist / param.scaling_dist;
    else
        v = raw_linear_vel;
    end
end