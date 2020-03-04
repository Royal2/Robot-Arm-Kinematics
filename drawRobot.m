%Use this function for drawing your robot:
%{
    INPUTS: 
    x_1,y_1: location of the first joint
    x_2,y_2: location of the second joint
    x_e,y_e: location of the end effector

    Note: Remember the zeroth joint x_0, y_0 is always at the origin [0,0]


%}

function drawRobot(x_1,y_1,x_2,y_2,x_e,y_e)
    figure(1)
    plot([0 x_1 x_2 x_e],[0 y_1 y_2 y_e],'lineWidth',3,'Marker','o','MarkerSize',5,'MarkerEdgeColor','black','MarkerFaceColor','black');
    pbaspect([1 1 1]);
    xlim([-20 20]);
    ylim([-20 20]);
    set(gca,'Xtick',-20:2:20)
    set(gca,'Ytick',-20:2:20)
    grid on
end













