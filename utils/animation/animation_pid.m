function animation_pid(pose, traj, delta, record_video)
    hold on
    [frames, ~] = size(pose);

    if record_video
        process = VideoWriter('./animation/video/pid.avi');
        open(process);
        movie = moviein(frames);
    end

    for i=1:frames
        handler = plot_robot([pose(i, 2) + delta, pose(i, 1) + delta, pose(i, 3)], 0.8, 0.4, 'r');
%         handler2 = plot_trajectory(traj(i).info, delta);
        plot(pose(i, 2) + delta, pose(i, 1) + delta, 'Marker', '.', 'color', "#f00");
        drawnow;
        if record_video
            movie(:, i) = getframe;
            writeVideo(process, movie(:, i));
        end
        delete(handler);
%         delete(handler2);
    end
    
    if record_video
        close(process);
    end
end
