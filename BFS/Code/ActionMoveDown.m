function [Status_d, NewNode_d] = ActionMoveDown(CurrentNode)
    NewNode_d = CurrentNode;
    [X0,Y0] = find(NewNode_d==0); 
    if X0<3
        Status_d = 1;
        tmp = NewNode_d(X0+1,Y0);
        NewNode_d(X0+1,Y0)= 0;
        NewNode_d(X0,Y0) = tmp;   
    else
        Status_d = 0;
    end
    