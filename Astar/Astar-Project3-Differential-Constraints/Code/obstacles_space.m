function [] = obstacles_space()


%% Intialize figure window
figure('units','normalized','outerposition',[0 0 0.75 0.75])
hold on
title('Select start and goal points:: Avoid area near obstacles as they are padded');
axis([0 15 0 10])
ax = gca;
ax.XTick = 0:0.5:15;
ax.YTick = 0:0.5:10;


%% Squares - x coordinates
square_x(1,:)= [1.575,1.575,2.375,2.375];
square_x(2,:)= [2.775,2.775,3.575,3.575];
square_x(3,:)= [12.05,12.05,13.65,13.65];
square_x(4,:)= [14.05,14.05,14.85,14.85];
square_x(5,:)= [14.05,14.05,14.85,14.85];
square_x(6,:)= [5.5525,5.5525,7.1525,7.1525];
square_x(7,:) = [9.3,9.3,10.9,10.9];

%% Squares y- coordinates
square_y(1,:)=[7.375,9.375,9.375,7.375];
square_y(2,:)=[7.375,9.375,9.375,7.375];
square_y(3,:)=[8.55,9.65,9.65,8.55];
square_y(4,:)=[4.275,6.275,6.275,4.275];
square_y(5,:)=[2.275,4.275,4.275,2.275];
square_y(6,:)=[4.2,5.8,5.8,4.2];
square_y(7,:)=[4.2,5.8,5.8,4.2];

%% Drawing Squares
for i= 1:7
    fill(square_x(i,:),square_y(i,:),'k');
end

%% Draw Circles
r= 0.8;
center_x = [6.3525,6.3525,10.1,10.1];
center_y = [5.8,4.2,5.8,4.2];

for i=1:4
    filledCircle([center_x(1,i),center_y(1,i)],r,1000,'k'); 
end

end