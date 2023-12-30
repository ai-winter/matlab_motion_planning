function arrow = plot_arrow(x, y, theta, color)
%%
% @file: plot_arrow.m
% @breif: plot arrow in figure
% @author: Winter
% @update: 2023.12.30

%%
    l = 2.0;

    ref_vec = [l; 0];
    r_mat = [
            cos(theta), -sin(theta);
            sin(theta), cos(theta)
    ];
    end_pt = r_mat * ref_vec + [x; y];
    
    arrow = quiver(x, y, end_pt(1) - x, end_pt(2) - y, ...
       'MaxHeadSize', 5.5, 'AutoScaleFactor', 1, 'AutoScale', 'off', ...
       'LineWidth', 2, 'color', color, ...
       'Marker', 'o', 'MarkerSize', 4, 'MarkerFaceColor', color);
end

