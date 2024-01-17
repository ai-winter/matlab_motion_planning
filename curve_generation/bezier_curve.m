function path = bezier_curve(points)
% @file: bezier_curve.m
% @breif: Bezier curve generation
% @author: Winter
% @update: 2024.1.13

    % parameters
    param.step = 0.1;
    param.offset = 3.0;

    [num_pts, ~] = size(points);
    
    % generate curve
    path = [];
    for i=1:num_pts - 1
        path_i = generation(...
            [points(i, 1), points(i, 2), points(i, 3)], ...
            [points(i + 1, 1), points(i + 1, 2), points(i + 1, 3)], ...
            param ...
       );
       path = [path; path_i];
    end
end

%%
function curve = generation(start, goal, param)
    sx = start(1); sy = start(2);
    gx = goal(1); gy = goal(2);
    
    n_points =  hypot(sx - gx, sy - gy) / param.step;
    control_pts = getControlPoints(start, goal, param);
    
    curve = [];
    for t=0:1 / n_points:1
        curve = [curve; bezier(t, control_pts)];
    end
end

function curve_pt = bezier(t, control_pts)
    [m, ~] = size(control_pts);
    n = m - 1;
    pt_x = 0; pt_y = 0;
    for i=0:n
        pt_x = pt_x + nchoosek(n, i) * power(t, i) * power(1 - t, n - i) * control_pts(i + 1, 1);
        pt_y = pt_y + nchoosek(n, i) * power(t, i) * power(1 - t, n - i) * control_pts(i + 1, 2);
    end
    curve_pt = [pt_x, pt_y];
end

function control_pts = getControlPoints(start, goal, param)
    sx = start(1); sy = start(2); syaw = start(3);
    gx = goal(1); gy = goal(2); gyaw = goal(3);
    
    d = hypot(sx - gx, sy - gy) / param.offset;
    control_pts = [
        sx, sy; sx + d * cos(syaw), sy + d * sin(syaw);
        gx - d * cos(gyaw), gy - d * sin(gyaw); gx, gy
    ];
end


