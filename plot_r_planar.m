function robot = plot_r_planar(n, lengths)
% This function creat a planar robot, initial joint values are all at
% zeros.
% Run update_r_planar to update the figure with given joint values

figure;

h = plot(0, 0, '-b*');
total_length = sum (lengths) + 0.1;
xlim([-total_length total_length]);
ylim([-total_length total_length]);
%axis equal

theta = zeros(1, n);
pts = r_planar(n, lengths, theta);

pts = [0 0; pts]; % add origin
set(h, 'XData', pts(:, 1), 'YData', pts(:, 2));

robot.h = h;
robot.n = n;
robot.lengths = lengths;
robot.theta = theta;
robot.pts = pts;
robot.ee = pts(robot.n+1, :);