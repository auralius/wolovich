function computational_ikine()

    clc;
    clear all;
    close all;
    
    % Time sampling
    dt = 0.001;

    % Define a 4-DOF planar robot, with revolute joints
    lengths = [0.1 0.1 0.1 0.1];
    robot = plot_r_planar(4, lengths);

    % Define target for the end effector to achieve: xd
    xd = [0.1 -0.2]';
    
    % End-efector initial position and joint values
    xs = robot.ee';
    theta_s = robot.theta';
    
    % The parameters K and A (see Fig 1 and 2 of the paper)
    K = diag(ones(1,4)).*5000;
    A = diag(ones(1,4)).*10;
    
    % The junction a and b as in Fig 1 and 2 of the paper
    a = [0 0 0 0]';
    b = [0 0 0 0]';
    
    % Loop until it converges
    while norm(xd-xs, 2) > 0.001
        jac = numeric_jacobian(@f, robot.theta);
        a = K * jac'* (xd-xs);
        
        b_dot = a - A * b;
        b = b + b_dot .* dt;
        
        theta_s = theta_s + b .* dt;
        
        theta_s_dot_dot = b_dot;
        theta_s_dot = b;
        
        robot = update_r_planar(robot, theta_s');
        
        xs = robot.ee';
        drawnow;
    end
end

%% Forward kinematics of the 4-R planar robot
function x = f(theta)
    n = 4;
    lengths = [0.1 0.1 0.1 0.1];
    pts = r_planar(n, lengths, theta);
    x = pts(end,:);
end