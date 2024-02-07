function path = cubic_spline_curve(points)
% @file: cubic_spline_curve.m
% @breif: Cubic spline curve generation
% @author: Winter
% @update: 2024.1.21
    param.step = 0.1;

    ds = hypot(diff(points(:, 1)), diff(points(:, 2)));
    s = cumsum(ds);
    s = [0; s];
    t = 0:param.step:s(end);

    path = [spline(s, points(:, 1), t), spline(s, points(:, 2), t)];
end

%%
function p = spline(s_list, dir_list, t)
    % cubic polynomial functions
    a = dir_list;
    
    [num, ~] = size(s_list);
    
    h = diff(s_list);

    % calculate coefficient matrix
    A = zeros(num, num);
    for i=2:num - 1
        A(i, i - 1) = h(i - 1);
        A(i, i) = 2.0 * (h(i - 1) + h(i));
        A(i, i + 1) = h(i);
    end
  A(1, 1) = 1.0;
  A(num, num) = 1.0;

  B = zeros(num, 1);
  for i=2:num - 1
      B(i, 1) = 3.0 * (a(i + 1) - a(i)) / h(i) - 3.0 * (a(i) - a(i - 1)) / h(i - 1);
  end

  c = A \ B;
  
  b = zeros(num - 1, 1); d = zeros(num - 1, 1);
  for i=1:num - 1
    b(i) = (a(i + 1) - a(i)) / h(i) - h(i) * (c(i + 1) + 2.0 * c(i)) / 3.0;
    d(i) = (c(i + 1) - c(i)) / (3.0 * h(i));
  end

  % calculate spline value and its derivative
  p = [];
  for i =1:length(t)
      idx = find(s_list > t(i));
      if ~isempty(idx)
          id = idx(1) - 1;
          ds = t(i) - s_list(id);
          p = [p; a(id) + b(id) * ds + c(id) * power(ds, 2) + d(id) * power(ds, 3)];
      end
  end
end