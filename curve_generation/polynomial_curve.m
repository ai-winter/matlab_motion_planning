function path = polynomial_curve(points)
% @file: polynomial_curve.m
% @breif: Polynomial curve generation
% @author: Winter
% @update: 2023.12.30

    % parameters
    param.max_acc = 1.0;    % Maximum acceleration
    param.max_jerk = 0.5;   % Maximum jerk
    param.t_min = 1;
    param.t_max = 30;
    param.step = 0.1;
    param.max_curv = 0.25;
    
    [num_pts, ~] = size(points);

    % generate velocity and acceleration constraints heuristically
    v = ones(num_pts);
    v(1) = 0;

    a = [];
    for i=1:num_pts - 1
        a = [a, (v(i + 1) - v(i)) / 5];
    end
    a = [a, 0];

    % generate curve
    path = [];
    for i=1:num_pts - 1
        path_i = generation(...
            [points(i, 1), points(i, 2), points(i, 3), v(i), a(i)], ...
            [points(i + 1, 1), points(i + 1, 2), points(i + 1, 3), v(i + 1), a(i + 1)], ...
            param ...
       );
       path = [path; path_i];
    end
end

%%
function path = generation(start_state, goal_state, param)
    sx = start_state(1); gx = goal_state(1);
    sy = start_state(2); gy = goal_state(2);
    syaw = start_state(3); gyaw = goal_state(3);
    sv = start_state(4); gv = goal_state(4);
    sa = start_state(5); ga = goal_state(5);
   
    sv_x = sv * cos(syaw);
    sv_y = sv * sin(syaw);
    gv_x = gv * cos(gyaw);
    gv_y = gv * sin(gyaw);

    sa_x = sa * cos(syaw);
    sa_y = sa * sin(syaw);
    ga_x = ga * cos(gyaw);
    ga_y = ga * sin(gyaw);
   
    traj_x = []; traj_y = [];
    traj_v = []; traj_a = []; traj_j = [];
    for T = param.t_min : param.step : param.t_max
        A = [power(T, 3), power(T, 4), power(T, 5);
                3 * power(T, 2), 4 * power(T, 3), 5 * power(T, 4);
                6 * T, 12 * power(T, 2), 20 * power(T, 3)];
        b = [gx - sx - sv_x * T - sa_x * T * T / 2;
                gv_x - sv_x - sa_x * T;
                ga_x - sa_x];
        X = A \ b;
        px = [sx, sv_x, sa_x / 2, X(1), X(2), X(3)];
            
        b = [gy - sy - sv_y * T - sa_y * T * T / 2;
                gv_y - sv_y - sa_y * T;
                ga_y - sa_y];
        X = A \ b;
        py = [sy, sv_y, sa_y / 2, X(1), X(2), X(3)];
        

        for t=0 : param.dt : T + param.dt
            traj_x = [traj_x, x(px, t)];
            traj_y = [traj_y, x(py, t)];
            
            vx = dx(px, t); vy = dx(py, t);
            traj_v = [traj_v, hypot(vx, vy)];
            
            ax = ddx(px, t); ay = ddx(py, t);
            a = hypot(ax, ay);
            if (length(traj_v) >= 2) && (traj_v(end) < traj_v(end - 1))
                a = -a;
            end
            traj_a = [traj_a, a];
            
            jx = dddx(px, t); jy = dddx(py, t);
            j = hypot(jx, jy);
            if (length(traj_a) >= 2) && (traj_a(end) < traj_a(end - 1))
                j = -j;
            end
            traj_j = [traj_j, j];
        end
        
        if ((max(abs(traj_a)) < param.max_acc) && (max(abs(traj_j)) < param.max_jerk))
            break;
        else
            traj_x = []; traj_y = [];
            traj_v = []; traj_a = []; traj_j = [];
        end
    end
    
    path = [traj_x', traj_y'];
end

function val = x(p, t)
    val = p(1) + p(2) * t + p(3) * power(t, 2) + p(4) * power(t, 3) + p(5) * power(t, 4) + p(6) * power(t, 5);
end

function val = dx(p, t)
    val = p(2) + 2 * p(3) * t + 3 * p(4) * power(t, 2) + 4 * p(5) * power(t, 3) + 5 * p(6) * power(t, 4);
end

function val = ddx(p, t)
    val = 2 * p(3) + 6 * p(4) * t + 12 * p(5) * power(t, 2) + 20 * p(6) * power(t, 3);
end

function val = dddx(p, t)
    val = 6 * p(4) + 24 * p(5) * t + 60 * p(6) * power(t, 2);
end



