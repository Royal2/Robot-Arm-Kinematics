% Visualize robot arm
clear all; close all; clc;
% Parameters ====================
%{
%Forward Kinematics
L0=3; L1=5; L2=7;
theta0=pi/3;
theta1=pi/12;
theta2=-pi/6;
%}
%Invser Kinematics
L0=10; L1=10; L2=10;
xe=6;
ye=12;
% ===============================
%[x1,y1,x2,y2,xe,ye]=ForwardKinematics(L0,L1,L2,theta0,theta1,theta2);
tic
[theta0, theta1, theta2]=InverseKinematics(L0,L1,L2,xe,ye);
toc
fprintf("theta0 is %d, theta1 is %d, theta2 is %d", theta0,theta1,theta2);