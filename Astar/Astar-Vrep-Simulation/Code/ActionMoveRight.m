function [Status_r, NewNode_r] = ActionMoveRight(CurrentNode,res)
    NewNode_r = CurrentNode;
    if NewNode_r(1)<14.8
        Status_r = 1;
        NewNode_r(1)= NewNode_r(1) + res;
    else
        Status_r = 0;
    end
    