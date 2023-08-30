function show_lidar(ranges, angles)
% @file: show_lidar.m
% @breif: Visualize 2D lidar data in the robot body frame
% @author: Winter
% @update: 2023.8.30

    lidar_local = [ranges .* cos(angles), ranges .* sin(angles)];

    figure
    plot(0, 0, 'rs');
    hold on;
    plot(lidar_local(:, 1), lidar_local(:, 2), '.-'); 
    axis equal;
    xlabel('x'); ylabel('y');
    grid on;
    title('Lidar measurement in the body frame');
end

