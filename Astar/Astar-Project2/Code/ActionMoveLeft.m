function [Status_l, NewNode_l] = ActionMoveLeft(CurrentNode)
    NewNode_l = CurrentNode;
    if NewNode_l(1)>0
        Status_l = 1;
        NewNode_l(1)= NewNode_l(1) - 1;
    else
        Status_l = 0;  
    end
    