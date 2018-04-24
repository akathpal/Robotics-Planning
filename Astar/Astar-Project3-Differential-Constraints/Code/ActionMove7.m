function [Status, NewNode] = ActionMove7(CurrentNode,res)
    NewNode = CurrentNode;
    ul = 0;
    ur = 50;
    x_prev = NewNode(1);
    y_prev = NewNode(2);
    theta_prev = NewNode(3);
    
    Status = status_check(NewNode);
    if Status
        NewNode = differential(ul,ur,x_prev,y_prev,theta_prev,res);
    end
end