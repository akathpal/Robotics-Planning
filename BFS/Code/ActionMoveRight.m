function [Status_r, NewNode_r] = ActionMoveRight(CurrentNode)
    NewNode_r = CurrentNode;
    [X0,Y0] = find(NewNode_r==0); 
    if Y0<3
        Status_r = 1;
        tmp = NewNode_r(X0,Y0+1);
        NewNode_r(X0,Y0+1)= 0;
        NewNode_r(X0,Y0) = tmp;
    else
        Status_r = 0;
    end
    