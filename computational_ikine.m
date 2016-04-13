function computational_ikine()

    clc;
    clear all;
    close all;
    
    dt = 0.001;

    lengths = [0.1 0.1 0.1 0.1];

    robot = plot_r_planar(4, lengths);

    xd = [0.1 -0.2]';
    xs = robot.ee';
    theta_s = robot.theta';
    
    K = diag(ones(1,4)).*5000;
    A = diag(ones(1,4)).*10;
    
    a = [0 0 0 0]';
    b = [0 0 0 0]';
    
    
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