function path = reeds_shepp_curve(points)
% @file: reeds_shepp_curve.m
% @breif: Reeds-Shepp curve generation
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
function [r, theta] = R(x, y)
    r = hypot(x, y);
    theta = atan2(y, x);
end

function theta_truncate = M(theta)
    while theta > pi
        theta = theta - 2.0 * pi;
    end
    while theta < -pi
        theta = theta + 2.0 * pi;
    end
    theta_truncate = theta;
end

function [tau, omega] = calTauOmega(u, v, xi, eta, phi)
    delta = M(u - v);
    A = sin(u) - sin(delta);
    B = cos(u) - cos(delta) - 1.0;

    t1 = atan2(eta * A - xi * B, xi * A + eta * B);
    t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;

    if t2 < 0
        tau = M(t1 + math.pi);
    else
        tau = M(t1);
    end
    omega = M(tau - u + v - phi);
end

function segs = SLS(x, y, phi)
    % Straight-Left-Straight generation mode.
    segs = [];
    
    phi = M(phi);
    if y > 0.0 && phi > 0.0 && phi < pi * 0.99
        xd = -y / tan(phi) + x;
        t = xd - tan(phi / 2.0);
        u = phi;
        v = hypot(x - xd, y) - tan(phi / 2.0);
        segs = [t, u, v];
    elseif y < 0.0 && phi > 0.0 && phi < pi * 0.99
        xd = -y / tan(phi) + x;
        t = xd - tan(phi / 2.0);
        u = phi;
        v = -hypot(x - xd, y) - tan(phi / 2.0);
        segs = [t, u, v];
    end
end

function segs = LRL(x, y, phi)
    % Left-Right-Left generation mode. (L+R-L-)
    segs = [];
    [r, theta] = R(x - sin(phi), y - 1.0 + cos(phi));
    if r <= 4.0
        u = -2.0 * asin(0.25 * r);
        t = M(theta + 0.5 * u + pi);
        v = M(phi - t + u);
        if t >= 0.0 && u <= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LSL(x, y, phi)
    % Left-Straight-Left generation mode. (L+S+L+)
    segs = [];
    
    [u, t] = R(x - sin(phi), y - 1.0 + cos(phi));
    if t >= 0.0
        v = M(phi - t);
        if v >= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LSR(x, y, phi)
    % Left-Straight-Right generation mode. (L+S+R+)
    segs = [];
    
    [r, theta] = R(x + sin(phi), y - 1.0 - cos(phi));
    r = r * r;
    if r >= 4.0
        u = sqrt(r - 4.0);
        t = M(theta + atan2(2.0, u));
        v = M(t - phi);
        if t >= 0.0 && v >= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LRLRn(x, y, phi)
    % Left-Right(beta)-Left(beta)-Right generation mode. (L+R+L-R-)
    segs = [];
    
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    rho = 0.25 * (2.0 + hypot(xi, eta));
    if rho <= 1.0
        u = acos(rho);
        [t, v] = calTauOmega(u, -u, xi, eta, phi);
        if t >= 0.0 && v <= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LRLRp(x, y, phi)
    % Left-Right(beta)-Left(beta)-Right generation mode. (L+R-L-R+)
    segs = [];
    
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if 0.0 <= rho && rho <= 1.0
        u = -acos(rho);
        if u >= -0.5 * pi
            [t, v] = calTauOmega(u, u, xi, eta, phi);
            if t >= 0.0 && v >= 0.0
                segs = [t, u, v];
            end
        end
    end
end

function segs = LRSR(x, y, phi)
    % Left-Right(pi/2)-Straight-Right generation mode. (L+R-S-R-)
    segs = [];
    
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    [rho, theta] = R(-eta, xi);

    if rho >= 2.0
        t = theta;
        u = 2.0 - rho;
        v = M(t + 0.5 * pi - phi);
        if t >= 0.0 && u <= 0.0 && v <= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LRSL(x, y, phi)
    % Left-Right(pi/2)-Straight-Left generation mode. (L+R-S-L-)
    segs = [];
    
    xi = x - sin(phi);
    eta = y - 1.0 + cos(phi);
    [rho, theta] = R(xi, eta);
    if rho >= 2.0
        r = sqrt(rho * rho - 4.0);
        u = 2.0 - r;
        t = M(theta + atan2(r, -2.0));
        v = M(phi - 0.5 * pi - t);
        if t >= 0.0 && u <= 0.0 && v <= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LRSLR(x, y, phi)
    % Left-Right(pi/2)-Straight-Left(pi/2)-Right generation mode. (L+R-S-L-R+)
    segs = [];
    
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    [r, ~] = R(xi, eta);
    if r >= 2.0
        u = 4.0 - sqrt(r * r - 4.0);
        if u <= 0.0
            t = M(atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = M(t - phi);
            if t >= 0.0 && v >= 0.0
                segs = [t, u, v];
            end
        end
    end
end

function path = RSPath(segs, ctypes)
    path.segs = segs;
    path.ctypes = ctypes;
    path.len = sum(abs(segs));
end
    
function paths = SCS(x, y, phi)
    %{
    # 2
    Straight-Circle-Straight generation mode(using reflect).

    Parameters
    ----------
    x/y: float
        Goal position
    phi: float
        Goal pose

    Return
    ----------
    paths: list
        Available paths
    %}
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    segs = SLS(x, y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["S", "L", "S"]);
    end
        
    segs = SLS(x, -y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["S", "R", "S"]);
    end
end

function paths = CCC(x, y, phi)
    %{
    # 8
    Circle-Circle-Circle generation mode(using reflect, timeflip and backwards).

    Parameters
    ----------
    x/y: float
        Goal position
    phi: float
        Goal pose

    Return
    ----------
    paths: list
        Available paths
    %}
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+R-L-
    segs = LRL(x, y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["L", "R", "L"]);
    end

    % timefilp: L-R+L+
    segs = LRL(-x, y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["L", "R", "L"]);
    end

    % reflect: R+L-R-
    segs = LRL(x, -y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["R", "L", "R"]);
    end

    % timeflip + reflect: R-L+R+
    segs = LRL(-x, -y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["R", "L", "R"]);
    end

    % backwards
    xb = x * cos(phi) + y * sin(phi);
    yb = x * sin(phi) - y * cos(phi);

    % backwards: L-R-L+
    segs = LRL(xb, yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, t], ["L", "R", "L"]);
    end

    % backwards + timefilp: L+R+L-
    segs = LRL(-xb, yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, -t], ["L", "R", "L"]);
    end

    % backwards + reflect: R-L-R+
    segs = LRL(xb, -yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, t], ["R", "L", "R"]);
    end

    % backwards + timeflip + reflect: R+L+R-
    segs = LRL(-xb, -yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, -t], ["R", "L", "R"]);
    end
end

function paths = CSC(x, y, phi)
    %{
    # 8
    Circle-Straight-Circle generation mode(using reflect, timeflip and backwards).

    Parameters
    ----------
    x/y: float
        Goal position
    phi: float
        Goal pose

    Return
    ----------
    paths: list
        Available paths
    %}
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+S+L+
    segs = LSL(x, y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["L", "S", "L"]);
    end

    % timefilp: L-S-L-
    segs = LSL(-x, y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["L", "S", "L"]);
    end

    % reflect: R+S+R+
    segs = LSL(x, -y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["R", "S", "R"]);
    end

    % timeflip + reflect: R-S-R-
    segs = LSL(-x, -y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["R", "S", "R"]);
    end

    % L+S+R+
    segs = LSR(x, y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["L", "S", "R"]);
    end

    % timefilp: L-S-R-
    segs = LSR(-x, y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["L", "S", "R"]);
    end

    % reflect: R+S+L+
    segs = LSR(x, -y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["R", "S", "L"]);
    end

    % timeflip + reflect: R+S+l-
    segs = LSR(-x, -y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["R", "S", "L"]);
    end
end

function paths = CCCC(x, y, phi)
    %{
    # 8
    Circle-Circle(beta)-Circle(beta)-Circle generation mode
    (using reflect, timeflip and backwards).

    Parameters
    ----------
    x/y: float
        Goal position
    phi: float
        Goal pose

    Return
    ----------
    paths: list
        Available paths
    %}
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+R+L-R-
    segs = LRLRn(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, u, -u, v], ["L", "R", "L", "R"]);
    end

    % timefilp: L-R-L+R+
    segs = LRLRn(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, -u, u, -v], ["L", "R", "L", "R"]);
    end

    % reflect: R+L+R-L-
    segs = LRLRn(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, u, -u, v], ["R", "L", "R", "L"]);
    end

    % timeflip + reflect: R-L-R+L+
    segs = LRLRn(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, -u, u, -v], ["R", "L", "R", "L"]);
    end

    % L+R-L-R+
    segs = LRLRp(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, u, u, v], ["L", "R", "L", "R"]);
    end

    % timefilp: L-R+L+R-
    segs = LRLRp(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, -u, -u, -v], ["L", "R", "L", "R"]);
    end

    % reflect: R+L-R-L+
    segs = LRLRp(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, u, u, v], ["R", "L", "R", "L"]);
    end

    % timeflip + reflect: R-L+R+L-
    segs = LRLRp(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, -u, -u, -v], ["R", "L", "R", "L"]);
    end
end

function paths = CCSC(x, y, phi)
    %{
    # 16
    Circle-Circle(pi/2)-Straight-Circle and Circle-Straight-Circle(pi/2)-Circle
    generation mode (using reflect, timeflip and backwards).

    Parameters
    ----------
    x/y: float
        Goal position
    phi: float
        Goal pose

    Return
    ----------
    paths: list
        Available paths
    %}
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+R-(pi/2)S-L-
    segs = LRSL(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, v], ["L", "R", "S", "L"]);
    end

    % timefilp: L-R+(pi/2)S+L+
    segs = LRSL(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, -v], ["L", "R", "S", "L"]);
    end

    % reflect: R+L-(pi/2)S-R-
    segs = LRSL(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, v], ["R", "L", "S", "R"]);
    end

    % timeflip + reflect: R-L+(pi/2)S+R+
    segs = LRSL(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, -v], ["R", "L", "S", "R"]);
    end

    % L+R-(pi/2)S-R-
    segs = LRSR(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, v], ["L", "R", "S", "R"]);
    end

    % timefilp: L-R+(pi/2)S+R+
    segs = LRSR(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, -v], ["L", "R", "S", "R"]);
    end

    % reflect: R+L-(pi/2)S-L-
    segs = LRSR(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, v], ["R", "L", "S", "L"]);
    end

    % timeflip + reflect: R-L+(pi/2)S+L+
    segs = LRSR(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, -v], ["R", "L", "S", "L"]);
    end

    % backwards
    xb = x * cos(phi) + y * sin(phi);
    yb = x * sin(phi) - y * cos(phi);

    % backwards: L-S-R-(pi/2)L+
    segs = LRSL(xb, yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, -0.5 * pi, t], ["L", "S", "R", "L"]);
    end

    % backwards + timefilp: L+S+R+(pi/2)L-
    segs = LRSL(-xb, yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, 0.5 * pi, -t], ["L", "S", "R", "L"]);
    end

    % backwards + reflect: R-S-L-(pi/2)R+
    segs = LRSL(xb, -yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, -0.5 * pi, t], ["R", "S", "L", "R"]);
    end

    % backwards + timefilp + reflect: R+S+L+(pi/2)R-
    segs = LRSL(-xb, -yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, 0.5 * pi, -t], ["R", "S", "L", "R"]);
    end

    % backwards: R-S-R-(pi/2)L+
    segs = LRSR(xb, yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, -0.5 * pi, t], ["R", "S", "R", "L"]);
    end

    % backwards + timefilp: R+S+R+(pi/2)L-
    segs = LRSR(-xb, yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, 0.5 * pi, -t], ["R", "S", "R", "L"]);
    end

    % backwards + reflect: L-S-L-(pi/2)R+
    segs = LRSR(xb, -yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, -0.5 * pi, t], ["L", "S", "L", "R"]);
    end

    % backwards + timefilp + reflect: L+S+L+(pi/2)R-
    segs = LRSR(-xb, -yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, 0.5 * pi, -t], ["L", "S", "L", "R"]);
    end
end

function paths = CCSCC(x, y, phi)
    %{
    # 4
    Circle-Circle(pi/2)-Straight--Circle(pi/2)-Circle generation mode (using reflect, timeflip and backwards).

    Parameters
    ----------
    x/y: float
        Goal position
    phi: float
        Goal pose

    Return
    ----------
    paths: list
        Available paths
    %}
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+R-(pi/2)S-L-(pi/2)R+
    segs = LRSLR(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, -0.5 * pi, v], ["L", "R", "S", "L", "R"]);
    end

    % timefilp: L-R+(pi/2)S+L+(pi/2)R-
    segs = LRSLR(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, 0.5 * pi, -v], ["L", "R", "S", "L", "R"]);
    end

    % reflect: R+L-(pi/2)S-R-(pi/2)L+
    segs = LRSLR(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, -0.5 * pi, v], ["R", "L", "S", "R", "L"]);
    end

    % timefilp + reflect: R-L+(pi/2)S+R+(pi/2)L-
    segs = LRSLR(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, 0.5 * pi, -v], ["R", "L", "S", "R", "L"]);
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
    Generate the Reeds-Shepp Curve.

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
    dx = gx - sx; dy = gy - sy; dyaw = gyaw - syaw;
    x = (cos(syaw) * dx + sin(syaw) * dy) * param.max_curv;
    y = (-sin(syaw) * dx + cos(syaw) * dy) * param.max_curv;

    % select the best motion
    planners = ["SCS", "CCC", "CSC", "CCCC", "CCSC", "CCSCC"];
    best_cost = inf;
    best_path = [];

    for i=1:length(planners)
        planner = str2func(planners(i));
        paths = planner(x, y, dyaw);
        for j=1:length(paths)
            if paths(j).len < best_cost
                best_path = paths(j);
                best_cost = paths(j).len;
            end
        end
    end
    
    % interpolation
    points_num = floor(best_cost / param.step) + length(best_path.segs) + 3;
    x_list_ = zeros(points_num);
    y_list_ = zeros(points_num);
    yaw_list_ = zeros(points_num);

    i = 1;
    for j = 1:length(best_path.segs)
        m = best_path.ctypes(j);
        seg_length = best_path.segs(j);
        
        % path increment
         if seg_length > 0.0
            d_length = param.step;
         else
            d_length = -param.step;
         end
        x = x_list_(i); y = y_list_(i); yaw = yaw_list_(i);
        
        % current path length
        l = d_length;
        while abs(l) <= abs(seg_length)
            i = i + 1;
            new_pt = interpolate(m, l, [x, y, yaw], param);
            x_list_(i) = new_pt(1); y_list_(i) = new_pt(2); yaw_list_(i) = new_pt(3);
            l = l + d_length;
        end
        i = i + 1;
        new_pt = interpolate(m, seg_length, [x, y, yaw], param);
        x_list_(i) = new_pt(1); y_list_(i) = new_pt(2); yaw_list_(i) = new_pt(3);
    end
    
    % remove unused data
    while length(x_list_) >= 1 && x_list_(end) == 0.0
        x_list_(end) = [];
        y_list_(end) = [];
        yaw_list_(end) = [];
    end
    
    % coordinate transformation
    x_list = []; y_list = []; yaw_list = [];
    for i=1:length(x_list_)
        x_list(end + 1) = cos(-syaw) * x_list_(i) + sin(-syaw) * y_list_(i) + sx;
        y_list(end + 1) = -sin(-syaw) * x_list_(i) + cos(-syaw) * y_list_(i) + sy;
        yaw_list(end + 1) = M(yaw_list_(i) + syaw);
    end
end