function [Status_l, NewNode_l] = ActionMoveLeft(CurrentNode)
    NewNode_l = CurrentNode;
    [X0,Y0] = find(NewNode_l==0); 
    if Y0>1
        Status_l = 1;
        tmp = NewNode_l(X0,Y0-1);
        NewNode_l(X0,Y0-1)= 0;
        NewNode_l(X0,Y0) = tmp; 
    else
        Status_l = 0;  
    end
    