%{
Jacobian Method for inverse kinematics

INPUTS:
l==>L
l0, l1, l2: lengths of the robot links
x_e_target, y_e_target: Desired final position of the end effector 

OUTPUTS:
theta0_target, theta1_target, theta2_target: Joint angles of the robot that
take the end effector to [x_e_target,y_e_target]
%}
function [theta0_target, theta1_target, theta2_target] = InverseKinematics(L0,L1,L2,x_e_target,y_e_target)
    % Initialize the thetas
    theta0 = pi/3; theta1 = 0; theta2 = 0;
    % Obtain end effector position x_e, y_e for current thetas: 
    [x1,y1,x2,y2,x_e,y_e]=ForwardKinematics(L0,L1,L2,theta0,theta1,theta2);
    target=[x_e_target,y_e_target];
    estimated=[x_e,y_e];
    stack_x=java.util.Stack();
    stack_y=java.util.Stack();
    stack_x.push(x_e);
    stack_y.push(y_e);
    epsilom=0.1;
    while  norm(target-estimated)>epsilom
        % Calculating the Jacobian matrix for current values of theta:
        d11 = -L0*sin(theta0)-L1*sin(theta0+theta1)-L2*sin(theta0+theta1+theta2);
        d12 = -L1*sin(theta0+theta1)-L2*sin(theta0+theta1+theta2);
        d13 = -L2*sin(theta0+theta1+theta2);
        d21 = L0*cos(theta0)+L1*cos(theta0+theta1)+L2*cos(theta0+theta1+theta2);
        d22 = L1*cos(theta0+theta1)+L2*cos(theta0+theta1+theta2);
        d23 = L2*cos(theta0+theta1+theta2);
        J = [d11,d12,d13;d21,d22,d23];
        % Calculating the pseudo-inverse of the jacobian: 
        iJ=pinv(J);
        % Updating the values of thetas:
        alpha=0.1;
        delta=alpha*iJ*[x_e_target-x_e;y_e_target-y_e];
        theta0=theta0+delta(1);
        theta1=theta1+delta(2);
        theta2=theta2+delta(3);
        % Obtaining end effector position x_e, y_e for the updated thetas:
        [x1,y1,x2,y2,x_e,y_e]=ForwardKinematics(L0,L1,L2,theta0,theta1,theta2);
        estimated=[x_e,y_e];
        stack_x.push(x_e);
        stack_y.push(y_e);
        % Visualizing robot arm movement
        drawRobot(x1,y1,x2,y2,x_e,y_e)     
        pause(0.00001)  % delay to visualize arm movement
    end
    % Setting theta_target values:
    theta0_target = theta0;
    theta1_target = theta1;
    theta2_target = theta2;
    % Effector history tracker
    N=stack_x.size();
    x_hist=zeros(1,N);
    y_hist=zeros(1,N);
    for i=1:1:N
        x_hist(i)=stack_x.pop();
        y_hist(i)=stack_y.pop();
    end
    figure;
    plot(x_hist,y_hist); title('End effector position through all iterations');
end
