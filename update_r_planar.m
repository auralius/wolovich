function robot = update_r_planar(robot, theta)

pts = r_planar(robot.n, robot.lengths, theta);
pts = [0 0; pts]; % add origin
set(robot.h, 'XData', pts(:, 1), 'YData', pts(:, 2));

robot.theta = theta;
robot.pts = pts;
robot.ee = pts(robot.n+1, :);

end
