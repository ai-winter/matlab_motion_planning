function grid_map = grid_mapping(scan_ranges, scan_angles, scan_pose, param)
% @file: grid_mapping.m
% @breif: Construct occupancy grid map using 2D lidar
% @author: Winter
% @update: 2023.8.30

% parameter
resolution = param.resolution;      % the number of grids for 1 meter.
grid_map = zeros(param.size);      % grid map initialization.
origin = param.origin;                    % the origin of the map in pixels

% Log-odd parameters
lo_occ = param.lo_occ;                   
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(scan_pose, 2);
scans_num = size(scan_angles);
% for each time
for i = 1:N 
    x = scan_pose(1, i);
    y = scan_pose(2, i);
    theta = scan_pose(3, i);
    robot_pos = [ceil(x * resolution) + origin(1), ceil(y * resolution) + origin(2)];

    % Find grids hit by the rays (in the gird map coordinate)
    % NOTE: Given that the y-axis of the real-world coordinate system and the y-axis of the
    %             pixel matrix are inverted, the value of y is taken as its negative counterpart.
    %
    %       ↑                            O ——>
    %       |                               |
    %     O ——>                   ↓
    %   (real-world）   （pixel matrix）
    rays = scan_ranges(:, i);
    x_occ = rays .* cos(scan_angles + theta) + x;
    y_occ = -rays .* sin(scan_angles + theta) + y;
    occ_pos = [ceil(x_occ * resolution) + origin(1), ceil(y_occ * resolution) + origin(2)];

    % Find occupied-measurement cells and free-measurement cells
    occ_id = sub2ind(size(grid_map), occ_pos(:, 2), occ_pos(:, 1));

    free = [];
    for j = 1:scans_num
        [ix_free, iy_free] = bresenham(robot_pos, occ_pos(j, :));  
        free = [free; iy_free, ix_free];
    end
    free_id = sub2ind(size(grid_map), free(:, 1), free(:, 2));

    % Update the log-odds
    grid_map(occ_id) = grid_map(occ_id) + lo_occ;
    grid_map(free_id) = grid_map(free_id) - lo_free;

    % Saturate the log-odd values
    grid_map(grid_map > lo_max) = lo_max;
    grid_map(grid_map < lo_min) = lo_min;
end

