function [Status_u, NewNode_u] = ActionMoveUp(CurrentNode,res)
    NewNode_u = CurrentNode;
    if NewNode_u(2)<9.8
        Status_u = 1;
        NewNode_u(2)= NewNode_u(2) + res;
    else
        Status_u = 0;
    end
    