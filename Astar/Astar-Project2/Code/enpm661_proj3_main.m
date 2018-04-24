%% Abhishek Kathpal(akathpal) - Astar Implementation

clc;
clear;
close all;

for i = 1:2
        open(i).parent = [];
        open(i).local_cost = inf;
        open(i).global_cost = inf;
        node_open(i,:) = [0,0];
        close_list(i).parent = [];
        close_list(i).local_cost = inf;
        close_list(i).global_cost = inf;
        node_close(i,:) = [0,0];
end

%% Drawing Graphical Interface
Space = ones(151,251);
Space(37:81,56:105) = 0;
[columns,rows] = meshgrid(1:251, 1:151);
circlePixels = (rows - 31).^2 + (columns - 181).^2 <= 15.^2;
y = [55,51,89,51,14,14];
x = [120,158,165,188,168,145];
mask1 = poly2mask(x+1,151-y,151,251);
Space(mask1) = 0;
Space(circlePixels) = 0;

imshow(Space);

hold on

start_in = round(ginput(1));
goal_in = round(ginput(1));


tic;

start(1) = start_in(1) - 1;
start(2) = 151 - start_in(2);
goal(1) = goal_in(1) - 1;
goal(2) = 151 - goal_in(2);


i=1;
open(i).parent = [];
node_open(i,:) = start;
open(i).local_cost = 0;
open(i).global_cost = heuristic(start,goal);

           
b=1;
while 1
    
    [~,Ind] = min(arrayfun (@(x) x.global_cost, open)) ;
    curr_node = open(Ind);
    
    %% Left
    [Status, NewNode] = ActionMoveLeft(node_open(Ind,:));
    if Status 
        collision = obstacle(Space,NewNode);
        if ~collision && ~member(NewNode,node_close)
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 10;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 10;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
                %plot(NewNode(1)+1,151-NewNode(2),'y.');
                %pause(1e-100);
            end
        end
    end
    
   
    %% Right
    [Status, NewNode] = ActionMoveRight(node_open(Ind,:));
    if Status 
        collision = obstacle(Space,NewNode);
        if ~collision && ~member(NewNode,node_close)
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 10;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 10;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
                %plot(NewNode(1)+1,151-NewNode(2),'y.');
                %pause(1e-100);
            end
        end
    end
    
    %% Up
    [Status, NewNode] = ActionMoveUp(node_open(Ind,:));
    if Status 
        collision = obstacle(Space,NewNode);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 10;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 10;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
                %plot(NewNode(1)+1,151-NewNode(2),'y.');
                %pause(1e-100);
            end
        end
    end
    
    %% Down
    [Status, NewNode] = ActionMoveDown(node_open(Ind,:));
    if Status 
        collision = obstacle(Space,NewNode);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 10;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 10;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
                %plot(NewNode(1)+1,151-NewNode(2),'y.');
                %pause(1e-100);
            end
        end
    end
    
    %% UpLeft
    [Status, NewNode] = ActionMoveUpLeft(node_open(Ind,:));
    if Status 
        collision = obstacle(Space,NewNode);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 14;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 14;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
                %plot(NewNode(1)+1,151-NewNode(2),'y.');
                %pause(1e-100);
            end
        end
    end
    
   
    %% UpRight
    [Status, NewNode] = ActionMoveUpRight(node_open(Ind,:));
    if Status 
        collision = obstacle(Space,NewNode);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 14;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 14;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
                %plot(NewNode(1)+1,151-NewNode(2),'y.');
                %pause(1e-100);
            end
        end
    end
    
    %% DownLeft
    [Status, NewNode] = ActionMoveDownLeft(node_open(Ind,:));
    if Status 
        collision = obstacle(Space,NewNode);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 14;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 14;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
                %plot(NewNode(1)+1,151-NewNode(2),'y.');
                %pause(1e-100);
            end
        end
    end
    
    %% DownRight
    [Status, NewNode] = ActionMoveDownRight(node_open(Ind,:));
    if Status 
        collision = obstacle(Space,NewNode);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 14;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 14;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
                %plot(NewNode(1)+1,151-NewNode(2),'y.');
                %pause(1e-100);
            end
        end
    end
    
    %% Update
    close_list(b) = curr_node;
    node_close(b,:) = node_open(Ind,:);
    open(Ind) = [];
    node_open(Ind,:)=[];
    i=i-1;
    %disp(close_list(b));
    if node_close(b,:) == goal
        break;
    end
    b = b+1;
end


temp.list = close_list(b);
temp.node = node_close(b,:);

 

while 1
    
    x1 = temp.node(1)+1;
    y1 = 151-temp.node(2);

    if isempty(temp.list.parent)
        break;
    end
    x2 = temp.list.parent(1)+1;
    y2 = 151-temp.list.parent(2);
    
    a = ismember(temp.list.parent,node_close,'rows');
    if a==0
        disp("here");
        [~,pos_1] = member(temp.list.parent,node_open);
        temp.node = node_open(pos_1,:);
        temp.list = open(pos_1);
    else
        [~,pos] = member(temp.list.parent,node_close);
        temp.node = node_close(pos,:);
        temp.list = close_list(pos);
    end
    plot([x1 x2],[y1 y2],'r-o');
    pause(1e-100);
end
toc;

answer = questdlg('Do you want to plan again','Yes','No');
switch answer
    case 'Yes'
        disp([answer ':Choose Start and Goal Points']);
        run('enpm661_proj3_main.m');
    case 'No'
        disp([answer ':Bye']);
end

