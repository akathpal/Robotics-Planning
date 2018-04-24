function [Status_u, NewNode_u] = ActionMoveUp(CurrentNode)
    NewNode_u = CurrentNode;
    if NewNode_u(2)<150
        Status_u = 1;
        NewNode_u(2)= NewNode_u(2) + 1;
    else
        Status_u = 0;
    end
    