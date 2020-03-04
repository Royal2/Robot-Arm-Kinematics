%{
This function is supposed to implement forward kinematics for a robot arm
with 3 links constrained to move in 2-D. 



INPUTS:
l==>L
l0, l1, l2 : Lengths of the robot links 
theta0, theta1, theta2 : Robot joint angles


OUTPUTS:
[x_1,y_1]: location of the first joint
[x_2,y_2]: location of the second joint
[x_e,y_e]: location of the end effector

Note: Remember the zeroth joint x_0, y_0 is always at the origin [0,0]

(See illustration in the question)


%}

function [x_1,y_1,x_2,y_2,x_e,y_e] = ForwardKinematics(L0,L1,L2, theta0, theta1, theta2)
%cos(angle)=dot(a,b)/(norm(a)*norm(b))
%dot(a,b)=a*b'=b*a'
%J0 at origin (0,0)
    %reference to J0
    x_1=L0*cos(theta0);   %L0 projection onto x-axis
    y_1=L0*sin(theta0);
    %reference to J1
    x_2=L1*cos(theta0+theta1)+x_1;
    y_2=L1*sin(theta0+theta1)+y_1;
    %reference to J2
    x_e=L2*cos(theta0+theta1-theta2)+x_2;
    y_e=L2*sin(theta0+theta1-theta2)+y_2;
end