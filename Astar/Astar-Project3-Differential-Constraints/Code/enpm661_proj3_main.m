clc;
clear;
close all;

%% if your changing scaling factor change it in differential function also
%%scaling factor is for changing speed at 1 it is ul and ur 100,100 at 2 it is 50,50
scaling =2;

res = 1/(scaling);

obstacles_space();

for i = 1:2
        open(i).parent = [];
        open(i).local_cost = inf;
        open(i).global_cost = inf;
        node_open(i,:) = [inf,inf,0,0,0,0];
        close_list(i).parent = [];
        close_list(i).local_cost = inf;
        close_list(i).global_cost = inf;
        node_close(i,:) = [inf,inf,0,0,0,0];
end


start = ginput(1);
goal = ginput(1);

collision_start = obstacle(start);
collision_goal = obstacle(start);
if collision_start || collision_goal
answer = questdlg('Error - start or goal in padded region of object : Do you want to plan again','Yes','No');
switch answer
    case 'Yes'
        disp([answer ':Choose Start and Goal Points']);
        run('enpm661_proj3_main.m');
    case 'No'
        disp([answer ':Bye']);
end
end

tic;
theta = 0;
i=1;
open(i).parent = [];
node_open(i,:) = [start,theta,0,0,0];
open(i).local_cost = 0;
open(i).global_cost = heuristic(start,goal);

           
b=1;
%% Node Generation : 8 actions- depending on different velocity combinations
while 1
    
    [~,Ind] = min(arrayfun (@(x) x.global_cost, open)) ;
    curr_node = open(Ind);
    
    %% [100,100]/scaling
    c = 0.1833*res;
    [Status, NewNode] = ActionMove1(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode);
        if ~collision && ~member(NewNode,node_close)
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + c;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + c;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-100);
            end
        end
    end
    
   
    %% [50,50]/scaling
    c = 0.0916*res;
    [Status, NewNode] = ActionMove2(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode);
        if ~collision && ~member(NewNode,node_close)
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + c;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + c;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-100);
            end
        end
    end
    
    %% [100,50]/scaling
    c = 0.1374*res;
    [Status, NewNode] = ActionMove3(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + c;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + c;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-100);
            end
        end
    end
    
    %% [50,0]/scaling
    c = 0.0458*res;
    [Status, NewNode] = ActionMove4(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + c;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + c;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-100);
            end
        end
    end
    
    %% [100,0]/scaling
    c = 0.0916*res;
    [Status, NewNode] = ActionMove5(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + c;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + c;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-100);
            end
        end
    end
    
   
    %% [0,100]/scaling
    c = 0.0916*res;
    [Status, NewNode] = ActionMove6(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + c;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + c;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-100);
            end
        end
    end
    
    %% [0,50]/scaling
    c = 0.0458*res;
    [Status, NewNode] = ActionMove7(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + c;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + c;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-100);
            end
        end
    end
    
    %% [50,100]/scaling
    c = 0.1374*res;
    [Status, NewNode] = ActionMove8(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + c;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + c;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-100);
            end
        end
    end
    
    %% Update
    close_list(b) = curr_node;
    node_close(b,:) = node_open(Ind,:);
    open(Ind) = [];
    node_open(Ind,:)=[];
    i=i-1;
    
    %% Loop break condition
    if abs(node_close(b,1) - goal(1)) <= 0.1 && abs(node_close(b,2) - goal(2)) <= 0.1
        break;
    end
    b = b+1;
end


%% plotting the final output
temp.list = close_list(b);
temp.node = node_close(b,:);
k=1;
while 1
    
    x1 = temp.node(1);
    y1 = temp.node(2);
    %As my data is at 1hz so i am repeating the same v_x and v_y and theta
    %2 times to get data at 2hz
    v_x(1,k) = temp.node(1,4);
    v_y(1,k) = temp.node(1,5);
    theta(1,k) = temp.node(1,6);
    v_x(1,k+1) = temp.node(1,4);
    v_y(1,k+1) = temp.node(1,5);
    theta(1,k+1) = temp.node(1,6);
    k = k+2;
    if isempty(temp.list.parent)
        break;
    end
    
    x2 = temp.list.parent(1);
    y2 = temp.list.parent(2);
    
    
    
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

%% saving the data 
save v_x;
save v_y;
save theta;

toc;

answer = questdlg('Do you want to plan again','Yes','No');
switch answer
    case 'Yes'
        disp([answer ':Choose Start and Goal Points']);
        run('enpm661_proj3_main.m');
    case 'No'
        disp([answer ':Bye']);
end


