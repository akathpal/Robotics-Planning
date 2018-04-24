function [Status_u, NewNode_u] = ActionMoveUp(CurrentNode)
    NewNode_u = CurrentNode;
    [X0,Y0] = find(NewNode_u==0); 
    if X0>1
        Status_u = 1;
        tmp = NewNode_u(X0-1,Y0);
        NewNode_u(X0-1,Y0)= 0;
        NewNode_u(X0,Y0) = tmp;
    else
        Status_u = 0;
    end
    