
function [final_path] = turtlebot_astar(start,res)

obstacles_space();

for i = 1:2
        open(i).parent = [];
        open(i).local_cost = inf;
        open(i).global_cost = inf;
        node_open(i,:) = [inf,inf];
        close_list(i).parent = [];
        close_list(i).local_cost = inf;
        close_list(i).global_cost = inf;
        node_close(i,:) = [inf,inf];
end

goal = floor((ginput(1))/res)*res;
start = floor(start/res)*res;

i=1;
open(i).parent = [];
node_open(i,:) = start;
open(i).local_cost = 0;
open(i).global_cost = heuristic(start,goal);

           
b=1;

while 1
    
    [~,Ind] = min(arrayfun (@(x) x.global_cost, open)) ;
    curr_node = open(Ind);
    
      %% UpLeft
    [Status, NewNode] = ActionMoveUpLeft(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode,res);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + sqrt(2)*res;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + sqrt(2)*res;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-10);
            end
        end
    end
    
   
    
     %% DownLeft
    [Status, NewNode] = ActionMoveDownLeft(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode,res);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + sqrt(2)*res;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + sqrt(2)*res;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-10);
            end
        end
    end
    
    
     %% DownRight
    [Status, NewNode] = ActionMoveDownRight(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode,res);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + sqrt(2)*res;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + sqrt(2)*res;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-10);
            end
        end
    end
  
   
    
   
   
    
    %% UpRight
    [Status, NewNode] = ActionMoveUpRight(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode,res);
        if ~collision && ~member(NewNode,node_close) 
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + sqrt(2)*res;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + sqrt(2)*res;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-10);
            end
        end
    end
    
    %% Up
    [Status, NewNode] = ActionMoveUp(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode,res);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 1*res;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 1*res;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-10);
            end
        end
    end
     %% Left
    [Status, NewNode] = ActionMoveLeft(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode,res);
        if ~collision && ~member(NewNode,node_close)
            a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 1*res;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 1*res;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-10);
            end
        end
    end
    %% Down
    [Status, NewNode] = ActionMoveDown(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode,res);
        if ~collision && ~member(NewNode,node_close) 
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 1*res;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 1*res;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-10);
            end
        end
    end
     %% Right
    [Status, NewNode] = ActionMoveRight(node_open(Ind,:),res);
    if Status 
        collision = obstacle(NewNode,res);
        if ~collision && ~member(NewNode,node_close)
           a = ismember(NewNode,node_open,'rows');
            if a
                [~,loc] = member(NewNode,node_open);
                t = curr_node.local_cost + 1*res;
                if t < open(loc).local_cost
                    open(loc).local_cost = t;
                    open(loc).global_cost = t + heuristic(node_open(loc,:),goal);
                    open(loc).parent = node_open(Ind,:);
                end
            else
                node_open(i+1,:) = NewNode;
                open(i+1).parent = node_open(Ind,:);
                open(i+1).local_cost = curr_node.local_cost + 1*res;
                open(i+1).global_cost = open(i+1).local_cost + heuristic(NewNode,goal);
                i = i+1;
%                 plot(NewNode(1),NewNode(2),'y.');
%                 pause(1e-10);
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
    if abs(node_close(b,1) - goal(1)) <= res/10 && abs(node_close(b,2) - goal(2)) <= res/10
        break;
    end
    b = b+1;
end

temp.list = close_list(b);
temp.node = node_close(b,:);
Final_path(1,:) = temp.node;
k=2;
while 1
    
    x1 = temp.node(1);
    y1 = temp.node(2);

    if isempty(temp.list.parent)
        break;
    end
    x2 = temp.list.parent(1);
    y2 = temp.list.parent(2);
    
    Final_path(k,:) = temp.list.parent;
    k= k+1;
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

final_path = flipud(Final_path);


end