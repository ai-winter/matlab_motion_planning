function [line_x, line_y] = bresenham_(point1, point2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    line_x = []; line_y = [];
    x1 = point1(1); y1 = point1(2);
    x2 = point2(1); y2 = point2(2);
    
    d_x = abs(x2 - x1);
    d_y = abs(y2 - y1);
    if  (x2 - x1) == 0
        s_x = 0;
    else
        s_x = (x2 - x1) / d_x;
    end
    if  (y2 - y1) == 0
        s_y = 0;
    else
        s_y = (y2 - y1) / d_y;
    end
    x = x1; y = y1; e = 0;
    
    % check if any obstacle exists between node1 and node2
    if d_x > d_y
        tao = (d_y - d_x) / 2;
        while x ~= x2
            if e > tao
                x = x + s_x;
                e = e - d_y;
            elseif e < tao
                y = y + s_y;
                e = e + d_x;
            else
                x = x + s_x;
                y = y + s_y;
                e = e + d_x - d_y;
            end
            line_x = [line_x; x];
            line_y = [line_y; y];
        end
    % swap x and y
    else
        tao = (d_x - d_y) / 2;
        while y ~= y2
            if e > tao
                y = y + s_y;
                e = e - d_x;
            elseif e < tao
                x = x + s_x;
                e = e + d_y;
            else
                x = x + s_x;
                y = y + s_y;
                e = e + d_y - d_x;
            end
            line_x = [line_x; x];
            line_y = [line_y; y];
        end
    end
end

