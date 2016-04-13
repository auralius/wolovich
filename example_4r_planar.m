clc;
clear all;
close all;

l = [0.1 0.1 0.1 0.1];

robot = plot_r_planar(4, l);

robot = update_r_planar(robot, [0 0 0 pi/3]);
pause(1);
robot = update_r_planar(robot, [0 0 pi/3 pi/3]);
pause(1);
robot = update_r_planar(robot, [0 pi/3 pi/3 pi/3]);
pause(1);
robot = update_r_planar(robot, [pi/3 pi/3 pi/3 pi/3]);