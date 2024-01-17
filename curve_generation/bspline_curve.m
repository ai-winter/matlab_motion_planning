function path = bspline_curve(points)
% @file: bspline_curve.m
% @breif: B-spline curve generation
% @author: Winter
% @update: 2024.1.13

    % parameters
    param.step = 0.01;
    param.order = 3;
    param.param_mode = "PARAM_MODE_CENTRIPETAL";
    param.spline_mode = "SPLINE_MODE_INTERPOLATION";

    % generate curve
    parameters = paramSelection(points, param);
    knot = knotGeneration(parameters, length(parameters), param);
    
    control_points = [];
    if (param.spline_mode == "SPLINE_MODE_INTERPOLATION")
        control_points = interpolation(points, parameters, knot, param);
    else
        control_points = approximation(points, parameters, knot, param);
        parameters = paramSelection(control_points, param);
        knot = knotGeneration(parameters, length(control_points), param);
    end
    
    path =  generation(knot, control_points, param);
end

%%
function Nik_t = baseFunction(i, k, t, knot)
  % 1st order B-spline
  if (k == 0)
      if ((t >= knot(i)) && (t < knot(i + 1)))
          Nik_t =1.0;
      else
          Nik_t = 0.0;
      end
  
  % 2nd order and higher B-spline
  else
    length1 = knot(i + k) - knot(i);
    length2 = knot(i + k + 1) - knot(i + 1);

    % Handle the case where the denominator is 0 by replacing it with 1, defining 0/0 as 0
    if ((length1 == 0) && (length2 == 0))
      Nik_t = 0;
    elseif (length1 == 0)
      Nik_t = (knot(i + k + 1) - t) / length2 * baseFunction(i + 1, k - 1, t, knot);
    elseif (length2 == 0)
      Nik_t = (t - knot(i)) / length1 * baseFunction(i, k - 1, t, knot);
    else
      Nik_t = (t - knot(i)) / length1 * baseFunction(i, k - 1, t, knot) +...
              (knot(i + k + 1) - t) / length2 * baseFunction(i + 1, k - 1, t, knot);
    end
  end 
end

function parameters = paramSelection(points, param)
  [n, ~] = size(points);
  parameters = zeros(n, 1);

  if (param.param_mode == "PARAM_MODE_UNIFORMSPACED")
    parameters = (0:n - 1) / (n - 1);
  else
    parameters(1) = 0.0;
    s = zeros(n - 1);
    d_cumsum = 0.0;
    
    for i=1:n - 1
      d = 0.0;
      if (param.param_mode == "PARAM_MODE_CHORDLENGTH")
        d = dist(points(i, :), points(i + 1, :)');
      else
        d = power(dist(points(i, :), points(i + 1, :)'), 0.5);
      end
      d_cumsum = d_cumsum + d;
      s(i) = d_cumsum;
    end
    
    parameters(2:n) = s(1:n - 1) / s(n - 1);
  end
end

function knot = knotGeneration(parameters, n, param)
  m = n + param.order + 1;
  knot = zeros(m, 1);

  knot(n + 1:m) = 1.0;
  
  for i=param.order + 2:n
    knot(i) = sum(parameters(i - param.order:i - 1)) / param.order;
  end
end

function control_points = interpolation(points, parameters, knot, param)
  [n, ~] = size(points);
  N = zeros(n, n);
  D = points(:, 1:2); 

  for i=1:n
    for j=1:n
      N(i, j) = baseFunction(j, param.order, parameters(i), knot);
    end
  end
  
  N(n, n) = 1;

  control_points = N \ D;
end

function control_points = approximation(points, parameters, knot, param)
  [n, ~] = size(points);
  D = points(:, 1:2); 

  % heuristically setting the number of control points
  h = n - 1;
  N = zeros(n, h);
  for i=1:n
    for j=1:h
      N(i, j) = baseFunction(j, param.order, parameters(i), knot);
    end
  end

  N_ = N(2:n - 1, 2:h - 1);

  qk = zeros(n - 2, 2);
  for i=2:n - 1
    qk(i - 1, :) = D(i, :) - N(i, 1) * D(1, :) - N(i, h) * D(end, :);
  end

  Q = N_' * qk;
  P = (N_' * N_) \ Q;

  control_points = zeros(h, 2);
  control_points(2:h - 1, :) = P(1:h - 2, :);
  control_points(1, :) = D(1, :);
  control_points(h, :) = D(n, :);
end

function points = generation(knot, control_pts, param)
  n = ceil(1.0 / param.step);
  t = (0 : n - 1) / (n - 1);

  [m, ~] = size(control_pts);
  N = zeros(n, m);

  for i=1:n
    for j=1:m
      N(i, j) = baseFunction(j, param.order, t(i), knot);
    end
  end
  
  N(n, m) = 1.0;

  points = N * control_pts;
end
    


